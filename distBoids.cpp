#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)
#include "al/ui/al_Parameter.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
using namespace al;
using namespace std;
#include <vector>

string slurp(string fileName);

struct ShareAgent {
  Vec3f pos;
  al::Color col;
};

const int N = 1000;
struct SharedState {
  ShareAgent agent[N];
};

struct MyApp : public DistributedAppWithState<SharedState> {
  Mesh mesh;

  // simulation state
  vector<Vec3f> velocity;
  vector<Vec3f> acceleration;
  vector<double> mass;

  Parameter pointSize{ "/pointSize", "", 1.0, "", 0.0, 40.0 };
  Parameter sepDistThresh{ "/sepDistThresh","",0.53,"",0.01,0.9 };
  Parameter cohDistThresh{ "/cohDistThresh","",0.5,"",0.01,0.9 };
  Parameter alignDistThresh{ "/alignDistThresh","",0.1,"",0.01,0.9 };
  Parameter cohesionModifier{ "/cohesionModifier","",0.5,"",0.1,10 };
  Parameter separationModifier{ "/separationModifier","",0.32,"",0.1,10 };
  Parameter alignmentModifier{ "/alignmentModifier","",1.2,"",0,10 };
  
  ControlGUI gui;
  ShaderProgram pointShader; 

  std::shared_ptr<CuttleboneStateSimulationDomain<SharedState>> cuttleboneDomain;

  void onCreate() override {
        cuttleboneDomain = CuttleboneStateSimulationDomain<SharedState>::enableCuttlebone(this);

	// compile shaders
	pointShader.compile(slurp("../point-vertex.glsl"),
		slurp("../point-fragment.glsl"),
		slurp("../point-geometry.glsl"));


	if(!cuttleboneDomain){
		std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
		quit();
	}

    //
	gui << pointSize << sepDistThresh << cohDistThresh << alignDistThresh << cohesionModifier << separationModifier << alignmentModifier;
	gui.init();

	parameterServer()  << pointSize << sepDistThresh << cohDistThresh << alignDistThresh << cohesionModifier << separationModifier << alignmentModifier;

	navControl().useMouse(false);
	nav().pos(0, 0, 10);

	mesh.primitive(Mesh::POINTS);
	for (int _ = 0; _ < N; _++) {
		if(cuttleboneDomain->isSender()){
			mesh.vertex(rnd::uniform(), rnd::uniform(), rnd::uniform());
			mesh.color(0, 1, 0);
			mass.push_back(rnd::uniform() * 10 + 1);
			mesh.texCoord(pow(mass.back(),1/3), 0);
			velocity.push_back(Vec3f(0,0,0));
			acceleration.push_back(Vec3f(0, 0, 0));
		}
		else{
			mesh.vertex(0,0,0);
			mesh.color(0,1,0);
		}
	}

  }

  void onAnimate(double dt) override {
	  if(cuttleboneDomain->isSender()){
	  // calculate boids interactions
	  vector<Vec3f> idealDirection; // will be a vector of size I
	  for (int i = 0; i < velocity.size(); i++) {
		  Vec3f cohResultant;
		  Vec3f sepResultant;
		  Vec3f alignResultant;
		  int cohCount = 1; // at the very least it is in a flock of one, itself; Also, this is necessary to prevent divide by zero
		  int sepCount = 1;
		  int alignCount = 1;
		  for (int j = i + 1; j < velocity.size(); j++) {
			  double distance = dist(mesh.vertices()[i], mesh.vertices()[j]);

			  if (distance < cohDistThresh) {
				  cohCount += 1;
				  cohResultant += mesh.vertices()[i] + mesh.vertices()[j];
			  }

			  if (distance < sepDistThresh) {
				  sepCount += 1;
				  sepResultant += mesh.vertices()[i] + mesh.vertices()[j];
			  }

			  if (distance < alignDistThresh) {
				  alignCount += 1;
				  alignResultant += velocity[j].normalized() - velocity[i].normalized();
			  }
		  }

		  // calculate this boid's flock averages
		  Vec3f averageCohResultant = cohResultant / cohCount;
		  Vec3f averageSepResultant = sepResultant / sepCount;
		  Vec3f averageAlignResultant = alignResultant / alignCount;
		  Vec3f cohesionDirection = averageCohResultant.normalized();
		  Vec3f alignmentDirection = averageAlignResultant.normalized();

		  // calculate cohesion force
		  double cohesionForce = averageCohResultant.mag() / cohesionModifier;
		  acceleration[i] += (cohesionDirection * cohesionForce) / mass[i];

		  // calculate separation force
		  double separationForce = averageSepResultant.mag() / separationModifier;
		  acceleration[i] -= (cohesionDirection * separationForce) / mass[i];

		  // calculate alignment force
		  double alignForce = averageAlignResultant.mag() / alignmentModifier;
		  acceleration[i] += (alignmentDirection * alignForce) / mass[i];

		  //cout << separationForce << " " << endl;
	  }

	  // integration
	  vector<Vec3f>& position(mesh.vertices());
	  for (int i = 0; i < velocity.size(); i++) {
		  // "backward" Euler integration
		  velocity[i] += acceleration[i] * dt;
		  position[i] += velocity[i] * dt;
	  }

	  // clear all accelerations
	  for (auto& a : acceleration) a.zero();
	  
	  // store boid position and color data in shared state
	  for(int i = 0; i < N; i++){
	  	state().agent[i].pos = mesh.vertices()[i];
		state().agent[i].col = mesh.colors()[i];
	  }
	  }else{
	 	// record boid position and color data from shared state
		vector<Vec3f>& position(mesh.vertices());
		vector<al::Color>& colors(mesh.colors());
		for(int i = 0; i < N; i++){
			Vec3f oldPos = position[i];
			position[i] = state().agent[i].pos;
			colors[i] = state().agent[i].col;
		} 	
	  }

  }

  void onDraw(Graphics& g) override {
    //
	g.clear(0.0);
	//g.shader(pointShader);
	//g.shader().uniform("pointSize", pointSize.get() / 100);
	//g.blending(true);
	//g.blendModeTrans();
	//g.depthTesting(true);
	
	//cout << mesh.vertices()[0] << endl;
	//cout << nav().uf() << endl;
	g.draw(mesh);
	
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
