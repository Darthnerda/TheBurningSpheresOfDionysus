#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)
#include "al/ui/al_Parameter.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "al/spatial/al_HashSpace.hpp"
#include "al/graphics/al_Shapes.hpp"

//#include "Darthnerda/assignment/distributed/libfreenect/include/libfreenect.h"
//#include "Darthnerda/assignment/distributed/libfreenect/wrappers/c_sync/libfreenect_sync.h"

#include "libfreenect.h"
#include "libfreenect_sync.h"

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace al;
using namespace std;
#include <vector>

float* LoadVertexMatrix()
{
    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    float mat[16] = {
        1/fx,     0,  0, 0,
        0,    -1/fy,  0, 0,
        0,       0,  0, a,
        -cx/fx, cy/fy, -1, b
    };
    return mat;
}


// This matrix comes from a combination of nicolas burrus's calibration post
// and some python code I haven't documented yet.
float* LoadRGBMatrix()
{
    float mat[16] = {
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
    };
    return mat;
    //glMultMatrixf(mat);
}

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
}

// figuring out libfreenect
void DrawGLScene() {
    short *depth = 0;
    char *rgb = 0;
    uint32_t ts;
    if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
	no_kinect_quit();
    if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
	no_kinect_quit();

    static unsigned int indices[480][640];
    static short xyz[480][640][3];
    int i,j;
    for (i = 0; i < 480; i++) {
        for (j = 0; j < 640; j++) {
            xyz[i][j][0] = j;
            xyz[i][j][1] = i;
            xyz[i][j][2] = depth[i*640+j];
            indices[i][j] = i*640+j;
        }
    }

    //cout << rgb[0] << endl;

    cout << rgb[0] << endl;
    

    // set the projection from the XYZ to the texture image
    //float[16] foo = LoadRGBMatrix() * LoadVertexMatrix() * Vec3f(1/640.0f, 1/480.0f, 1);
    
}

string slurp(string fileName);

struct ShareAgent {
  Vec3f pos;
  al::Color col;
};

const int trailLength = 40;
unsigned trailIdx = 0;
const int N = 6000;
HashSpace space(4, N);
unsigned maxradius = space.maxRadius();
HashSpace::Query qmany(100);
float floatDim = (float)space.dim();
Vec3f center = Vec3f(space.dim()/2, space.dim()/2, space.dim()/2);


struct SharedState {
  ShareAgent agent[N];
};

enum AgentType {BOID, THREEDVIDEO};

struct Agent {
	Vec3f velocity;
	Vec3f acceleration;
	double mass;
	AgentType agentType;
};

struct MyApp : public DistributedAppWithState<SharedState> {
  Mesh mesh;
  Mesh trails[N];
  Mesh sphere;
  Texture tex;

  // simulation state
  Agent agents[N];

  Parameter timeStep{ "/timeStep", "", 0.1, "", 0.0000001, 0.1 };
  Parameter pointSize{ "/pointSize", "", 1.0, "", 0.0, 40.0 };
  Parameter sepDistThresh{ "/sepDistThresh","",0.01,"",0.00001,1.0 };
  Parameter cohDistThresh{ "/cohDistThresh","",0.13,"",0.000001,1.0 };
  Parameter alignDistThresh{ "/alignDistThresh","",0.01,"",0.000001,1.0 };
  Parameter cohesionModifier{ "/cohesionModifier","",0.60,"",0.1,10 };
  Parameter separationModifier{ "/separationModifier","",0.7,"",0.1,10 };
  Parameter alignmentModifier{ "/alignmentModifier","",0.96,"",0,10 };
  
  ControlGUI gui;
  ShaderProgram pointShader; 

  std::shared_ptr<CuttleboneStateSimulationDomain<SharedState>> cuttleboneDomain;

  void onCreate() override {
	// load cuttleboan
	cuttleboneDomain = CuttleboneStateSimulationDomain<SharedState>::enableCuttlebone(this);
	if(!cuttleboneDomain){
		std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
		quit();
	}

	// create semi transparent texture for sphere
	tex.create2D(256, 256, Texture::RGBA8);
	int Nx = tex.width();
	int Ny = tex.height();
	vector<Colori> pix;
	pix.resize(Nx * Ny);

	for (unsigned j = 0; j < Ny; ++j) {
		for (unsigned i = 0; i < Nx; ++i) {
			Color c = RGB(1,0,0);
			c.a = 0.5;
			pix[j * Nx + i] = c;
		}
	}
	tex.submit(pix.data());

	
	
	// compile shaders
	pointShader.compile(slurp("../point-vertex.glsl"),
		slurp("../point-fragment.glsl"),
		slurp("../point-geometry.glsl"));

	// pipe in gui stuff
	gui << pointSize << sepDistThresh << cohDistThresh << alignDistThresh << cohesionModifier << separationModifier << alignmentModifier << timeStep;
	gui.init();

	// pipe in gui stuff to the parameter server
	parameterServer()  << pointSize << sepDistThresh << cohDistThresh << alignDistThresh << cohesionModifier << separationModifier << alignmentModifier << timeStep;

	// prep navigation stuff
	navControl().useMouse(false);
	nav().pos(center);
	//nav().pos(0,0,0);

	// initialize the boid sphere
	sphere.primitive(Mesh::TRIANGLES);
	//cout << "foo" << maxradius << endl;
	addIcosphere(sphere, maxradius, 4);
	sphere.decompress();
	sphere.generateNormals();
	vector<Vec3f>& verts(sphere.vertices());
	for (unsigned i = 0; i < verts.size(); i++) {		
		//sphere.color(1,0,0,0.2);
		verts[i] += center;
		//cout << verts[i] << endl;
	}

//	DrawGLScene();

	// initialize all the agents
	mesh.primitive(Mesh::POINTS);
	for (unsigned id = 0; id < N; id++) {
		if(cuttleboneDomain->isSender()){
			Vec3f initialPos = Vec3f(rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1).normalize(rnd::uniform() * maxradius) + center;
			space.move(id, initialPos);
			HashSpace::Object& o = space.object(id);
		//	cout << o.pos << endl;
			mesh.vertex(o.pos);
			mesh.color(0, 1, 0);
			
			agents[id].mass = rnd::uniform() * 10 + 1;
			agents[id].velocity = Vec3f(0,0,0);
			agents[id].acceleration = Vec3f(0,0,0);
			agents[id].agentType = BOID;

			mesh.texCoord(pow(agents[id].mass,1/3), 0);

			for (unsigned j = 0; j < trailLength; j++) {
				trails[id].vertex(0,0,0);
			}
		}else{
			mesh.vertex(0,0,0);
			mesh.color(0,1,0);
			mesh.texCoord(5,0);
		}
	}

  }

  void onAnimate(double dt) override {
	  dt = timeStep;
	  if(cuttleboneDomain->isSender()){
		  // calculate physics
		  vector<Vec3f>& position(mesh.vertices());
		  vector<al::Color>& colors(mesh.colors());
		  for (unsigned id = 0; id < N; id++) {	
			HashSpace::Object& o = space.object(id);
			qmany.clear();
			int flockMates = qmany(space, o.pos, cohDistThresh * space.dim());
			Vec3f boidForce = Vec3f(0,0,0);
			for(unsigned i = 1; i < flockMates; i++) {			
				Vec3f vec2other = qmany[i]->pos - o.pos;
				Vec3f vec2mid = vec2other / 2;
				boidForce += vec2mid * cohesionModifier;
				boidForce -= vec2other * separationModifier;
				Agent& B = agents[qmany[i]->id];
				Agent& A = agents[id];
				Vec3f force = Vec3f((B.velocity - A.velocity).normalize(alignmentModifier));
				boidForce += force;
				//if (id == 0) {cout << qmany[i]->id << endl;}
			}
			agents[id].acceleration += boidForce / agents[id].mass;
			// integration
			agents[id].velocity += agents[id].acceleration * dt;
			position[id] += agents[id].velocity * dt;
			
			Vec3f vec2Origin = center - position[id];
			if(vec2Origin.mag() >= maxradius) { position[id] += vec2Origin * 1.95; }
			
			space.move(id, position[id]);
			// clearing acceleration
			agents[id].acceleration.zero();
			// updating shareState agents
			state().agent[id].pos = position[id];
			state().agent[id].col = colors[id];

			// update trail
			//Vec3f& trailPosition(trails[id].vertices()[trailIdx]);
			//trailPosition = o.pos;
		  }
	  }else{
		// record boid position and color data from shared state
		vector<Vec3f>& position(mesh.vertices());
		vector<al::Color>& colors(mesh.colors());
		for(int i = 0; i < N; i++){
			//Vec3f oldPos = position[i];
			position[i] = state().agent[i].pos;
			colors[i] = state().agent[i].col;
		}
		//cout << position[0] << endl; 	
	  }
	  trailIdx += 1;

  }

  void onDraw(Graphics& g) override {
    //
	g.clear(0.0);

	gl::blending(true);
	gl::blendTrans();

	static Light light;
	g.lighting(true);
	g.light(light);

	tex.bind();
	g.texture();
	g.draw(sphere);
	tex.unbind();	

	g.shader(pointShader);
	g.shader().uniform("pointSize", pointSize.get() / 100);
	//gl::depthTesting(true);
	g.draw(mesh);
	//g.meshColor();
	
	/*
	for (unsigned id = 0; id < N; id++) {
		g.draw(trails[id]);
	}
	*/
	
	
	if(isPrimary()){
		gui.draw(g);
	}
  }

  bool onKeyDown(const Keyboard& k) override {
    //
    //printf("got here\n");
    return false;
  }

  bool onMouseMove(const Mouse& m) override {
    //
    //printf("%d,%d\n", m.x(), m.y());
    return false;
  }
};

int main() {
  MyApp app;
  app.start();
}

string slurp(string fileName) {
	fstream file(fileName);
	string returnValue = "";
	while (file.good()) {
		string line;
		getline(file, line);
		returnValue += line + "\n";
	}
	return returnValue;
}
