// Starter code, slurp function, and much advice by Karl Yerkes
// Kinect calibration matrices from libfreenect glpcview.cpp example

// includes
#include "al/app/al_App.hpp"
#include "al/math/al_Random.hpp"
#include "al/ui/al_ControlGUI.hpp"  // gui.draw(g)
#include "al/ui/al_Parameter.hpp"
#include "al/ui/al_PresetSequencer.hpp"
#include "al/ui/al_SequenceRecorder.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "al/spatial/al_HashSpace.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "libfreenect.h"
#include "libfreenect_sync.h"
#include <vector>
using namespace al;
using namespace std;

// tweak zone
const float  minDistance = -10;
const float scalingFactor = 0.00021;
const int trailLength = 4;
const int NBoids = 6000;
const int NSpheres = 10;
const int sphereRingR = 35;
const int kinectW = 640;
const int kinectH = 480;
const int maxBoidNeighbors = 100;
const bool kinectInitiallyStreaming = false;
const unsigned initialKinectSphere = 1;
const float initialShellTransparency = 0.027;
const float initialRotateSpeed = 0.001;
const int NBoidifySeeds = 10;
const int sphereSubdivisions = 3;
const int NAttractorVerts = 40000;
// end tweak zone

// calculate some constants
const int NKinect = kinectW * kinectH;
const int boidsPerSphere = NBoids / NSpheres;
const int boidifyStride = ceil(NKinect / boidsPerSphere);
const int NSphereVerts = pow(sphereSubdivisions, 2) * 10 + 2;
HashSpace::Query qmany(maxBoidNeighbors);

// early globals declarations
unsigned maxradius;
float floatDim;
Vec3f hashCenter;
bool kinectStreaming = kinectInitiallyStreaming;
unsigned trailIdx = 0;
float bubbleRingPhase = 0;

// early function declarations
string slurp(string fileName);

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
}

bool isBoid(unsigned id);

// possibly unused calibration matrices
const Matrix4f vertexCalibrate(
        1/ 594.21f,     0,  0, 0,
        0,    -1/ 591.04f,  0, 0,
        0,       0,  0, -0.0030711f,
        -339.5f/594.21f, 242.7f/ 591.04f, -1,3.3309495f
);
const Matrix4f rgbCalibrate(
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
);

// data structures
enum sphereType {BOID, THREEDVIDEO};

struct ShareSphere {
  Vec3f origin;
  float vertBrightnesses[4000];  //[15360];
};

struct SharedState {
  // Every boid voxel will use 6 floats (3 pos, 3 col), and every two kinect voxels will use 1 float (depth).
  // If we're in boids-only mode then there will be NBoids, which will always be smaller than NKinect.
  // If we're in kinect mode then we will be using all of our memory. boidsPerSphere counts the number of boids additional to kinect voxels because only one sphere will be producing boids in this mode.
  // since the kinect depth data is in shorts, we can combine two shorts into one int, cast that to float, then cast to int and split into two shorts again on the other side
  float voxData[(boidsPerSphere * 6) + (NKinect / 2)]; 
  ShareSphere shareSpheres[NSpheres];
  Vec3f attractorVox[NAttractorVerts];
  al::Color backgroundColor;
  Vec3d navPos;
  Quatd navQuat;
};

struct Agent {
  Vec3f velocity;
  Vec3f acceleration;
  bool boidified;
  double mass;
};

struct attractorAgent {
  double rho = 28.0;
  double sigma = 10.0;
  double beta = 8.0 / 3.0;

  Vec3f getAttractorDerivative(Vec3f cursor) {
	return Vec3f(sigma * (cursor.y - cursor.x), cursor.x * (rho - cursor.z) - cursor.y, cursor.x * cursor.y - beta * cursor.z);
  }
};

struct Bubble {
  Vec3f center;
  Vec3f origin;
  HashSpace space;
  sphereType type;
  Agent agents[boidsPerSphere];
  vector<Vec3f> shellVerts;
};

struct boidifySeed {
  Vec3f pos;
  float affectiveRadius;
  float expansionRate;
};

struct MyApp : public DistributedAppWithState<SharedState> {
  // declare meshes and textures
  Mesh boids;
  Mesh trails[NBoids];
  Mesh spheres[NSpheres];
  Mesh kinectVoxels;
  Mesh aiAttractor;

  // declare simulation structs
  Bubble bubbles[NSpheres];
  boidifySeed boidifySeeds[NBoidifySeeds];
  attractorAgent theAI;

  Parameter timeStep{ "/timeStep", "", 0.1, "", 0.0000001, 0.1 };
  Parameter pointSize{ "/pointSize", "", 1.749, "", 0.0, 40.0 };
  Parameter KDist{ "/KDist","",0.23,"",0.000001,1.0 };
  Parameter cohesionModifier{ "/cohesionModifier","",0.60,"",0.1,10 };
  Parameter separationModifier{ "/separationModifier","",0.8,"",0.1,10 };
  Parameter alignmentModifier{ "/alignmentModifier","",0.96,"",0,10 };
  Parameter minimumDisparity{ "/minimumDisparity", "", 500, "", 0, 1000 };
  Parameter maximumDisparity{ "/maximumDisparity", "", 1100, "", 0, 5000 };
  ParameterInt kinectSphere{ "/kinectSphere", "", initialKinectSphere, "", 0, NSpheres - 1 };
  Parameter shellTransparency{ "/shellTransparency", "", initialShellTransparency, "", 0, 1 };
  Parameter rotateSpeed{ "/rotateSpeed", "", initialRotateSpeed, "", 0.0, 0.05 };
  ParameterBool boidifying{ "/boidifying", "", 0, "", 0, 1.0 };
  Parameter backgroundBrightness{ "/backgroundBrightness", "", 0.0, "", 0, 1.0};
  Parameter rho{ "/rho", "", 28.0, "", 0, 100};
  Parameter sigma{ "/sigma", "", 10.0, "", 0, 100};
  Parameter beta{ "/beta", "", 7.0, "", 0, 100};
  Parameter attractorScale{ "/attractorScale", "", 0.003, "", 0.00001, 0.01};

  // declare the gui
  ControlGUI gui;

  // declare the sequencer and recorder for scripted parameter adjustments
  //PresetSequencer sequencer;
  //SequenceRecorder recorder;

  // declare the point shader
  ShaderProgram pointShader; 

  // declare a pointer that is shared between the simulation machine and the renderers
  std::shared_ptr<CuttleboneStateSimulationDomain<SharedState>> cuttleboneDomain;

  void onCreate() override {
	// load cuttlebone
	cuttleboneDomain = CuttleboneStateSimulationDomain<SharedState>::enableCuttlebone(this);
	if(!cuttleboneDomain){
		std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
		quit();
	}

	// compile shaders
	pointShader.compile(slurp("../point-vertex.glsl"),
		slurp("../point-fragment.glsl"),
		slurp("../point-geometry.glsl"));

	// register the directory that holds the scripted sequence data with the sequencer and recorder
	//sequencer.setDirectory("presets");
	//recorder.setDirectory("presets");

	// register parameters with sequencer and recorder
	//sequencer << boidifying; // example
	//recorder << boidifying; // example

	// pipe parameters into the gui
	gui.init();
	gui << pointSize << KDist << cohesionModifier << separationModifier << alignmentModifier << timeStep << minimumDisparity << maximumDisparity << kinectSphere << shellTransparency << rotateSpeed << boidifying << backgroundBrightness << rho << sigma << beta << attractorScale;
	//gui << sequencer << recorder;

	// pipe parameters to the shared parameter server
	parameterServer() << pointSize << KDist << cohesionModifier << separationModifier << alignmentModifier << timeStep << minimumDisparity << maximumDisparity << kinectSphere << shellTransparency << rotateSpeed << boidifying << backgroundBrightness << rho << sigma << beta << attractorScale;

	// prep navigation stuff
	navControl().useMouse(false);
	nav().pos(Vec3f(0,0,25));

	// initialize the boid spheres
	for (unsigned i = 0; i < NSpheres; i++) {
		// initialize the bubble object stuff
		bubbles[i].space = HashSpace(4, boidsPerSphere);
		bubbles[i].center = Vec3f(bubbles[i].space.dim()/2, bubbles[i].space.dim()/2, bubbles[i].space.dim()/2);
		bubbles[i].type = i == 0 ? sphereType::THREEDVIDEO : sphereType::BOID;
		bubbles[i].origin = Vec3f(sphereRingR * cos(i * M_2PI/NSpheres), 0, sphereRingR * sin(i * M_2PI/NSpheres));		

		// create the sphere shell
		spheres[i].primitive(Mesh::TRIANGLES);
		addIcosphere(spheres[i], bubbles[i].space.maxRadius(), sphereSubdivisions);
		spheres[i].decompress();
		spheres[i].generateNormals();
	
		// move the sphere to its origin point
		vector<Vec3f>& verts(spheres[i].vertices());
		bubbles[i].shellVerts = verts;
//		cout << verts.size() << endl;
		for (unsigned j = 0; j < verts.size(); j++) {		
			verts[j] += bubbles[i].origin + bubbles[i].center;
			spheres[i].color((float)j / verts.size(), 0, 0, shellTransparency);
		}
	}

	// Figure out some hashspace related globals
	maxradius = bubbles[0].space.maxRadius();
	floatDim = (float)bubbles[0].space.dim();
	hashCenter = Vec3f(floatDim/2, floatDim/2, floatDim/2);

	// initialize all the points in the boid mesh
	boids.primitive(Mesh::POINTS);
	if(cuttleboneDomain->isSender()){
		// work out all the boids in all the spheres
		for (unsigned i = 0; i < NSpheres; i++) {
			for (unsigned id = 0; id < boidsPerSphere; id++) {
				// put the boid position in the hash space
				Vec3f initialPos = Vec3f(rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1).normalize(rnd::uniform() * maxradius) + hashCenter; // changed bubbles[i].center to hashCenter	
				// hide this fresh boid if we're doing kinectstreaming
				if (kinectStreaming && i != kinectSphere) { initialPos = Vec3f(100,100,100); }
				bubbles[i].space.move(id, initialPos);
				HashSpace::Object& o = bubbles[i].space.object(id);	
				
				// store some simulation state for the agent
				bubbles[i].agents[id].mass = rnd::uniform() * 30 + 1;
				bubbles[i].agents[id].velocity = Vec3f(0,0,0);
				bubbles[i].agents[id].acceleration = Vec3f(0,0,0);
				if (kinectStreaming) { bubbles[i].agents[id].boidified = false; } else { bubbles[i].agents[id].boidified = true; }

				// put the boid position, color, and size in the mesh
				boids.vertex(o.pos+bubbles[i].origin);
				boids.color(0.8, 0.2, 0);
				boids.texCoord(pow(bubbles[i].agents[id].mass,1/3), 0);

				// create the trail mesh for this boid
				for (unsigned j = 0; j < trailLength; j++) {
					trails[i * boidsPerSphere + id].vertex(o.pos + bubbles[i].origin);
				}
			}
		}

		// work out all the kinect voxels
		kinectVoxels.primitive(Mesh::POINTS);
		for (unsigned i = 0; i < NKinect; i++) {
			// fill the kinect voxels with default x and y coordinates which won't really change
			kinectVoxels.vertex(floor(i / kinectW),i % kinectW,0);
			kinectVoxels.color(0.5,0,0);
		}

		// work out the seeds from which the kinect data will be boidified when the time comes
		for (unsigned i = 0; i < NBoidifySeeds; i++) {
			boidifySeeds[i].pos = Vec3f(rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1, rnd::uniform() * 2 - 1).normalize(rnd::uniform() * maxradius) + hashCenter;
			boidifySeeds[i].affectiveRadius = 0;
			boidifySeeds[i].expansionRate = rnd::uniform() * 0.9 + 0.1;
		}

		// create a pool of vertices for the aiAttractor body
		aiAttractor.primitive(Mesh::LINE_STRIP);
		aiAttractor.vertex(1.0,1.0,1.0); // set the initial point of the attractor
		aiAttractor.color(0.0, 1, 0);
		vector<Vec3f>& positions(aiAttractor.vertices());
		for (unsigned i = 1; i < NAttractorVerts; i++) {
			aiAttractor.vertex(positions[i-1] + theAI.getAttractorDerivative(positions[i-1]) * 0.01); // i acts as time and 0.01 acts as the time increment
			aiAttractor.color(HSV(i/NAttractorVerts, 1, 1));
		}
	}else{
		// on the receiving end we will need a place for all the 300,000 or so kinect voxels
		for (unsigned i = 0; i < NBoids; i++) {
			boids.vertex(0,0,0);
			boids.color(0,0,0);
			boids.texCoord(5,0);
		}
		kinectVoxels.primitive(Mesh::POINTS);
		for (unsigned i = 0; i < NKinect; i++) {
			kinectVoxels.vertex(0, 0, 0);
			kinectVoxels.color(0.5,0,0);
			//kinectVoxels.texCoord(2,0);
		}
		// and the ai attractor
		aiAttractor.primitive(Mesh::LINE_STRIP);
		for (unsigned i = 0; i < NAttractorVerts; i++) {
			aiAttractor.vertex(0, 0, 0);
			aiAttractor.color(HSV(i/NAttractorVerts, 1, 1));
		}
	}
  }

  double t = 0.0;
  void onAnimate(double dt) override {
	  dt = timeStep;
	  if(cuttleboneDomain->isSender()){ 
		  // if any of the spheres are marked for kinect data, change the mesh data according to that
		  if (kinectStreaming) {
			// if we're currently boidifying this sphere, then update the radius and determine boidified this frame 
			if (boidifying) {
				for (unsigned i = 0; i < NBoidifySeeds; i++) { 
					// update the affective radius
					boidifySeeds[i].affectiveRadius += dt * boidifySeeds[i].expansionRate;

					// find this seed's affected neighbors
					qmany.clear();
					int affected = qmany(bubbles[kinectSphere].space, boidifySeeds[i].pos, boidifySeeds[i].affectiveRadius); // may need to add on [ * floatDim ] to normalize the affective radius in the query appropriately. May not be needed.
					// set their status to boidified if not already
					for (unsigned j = 1; j < affected; j++) {
						bubbles[kinectSphere].agents[qmany[j]->id].boidified = true;
					}
				}
			}
			
			// grab depth and rgb data from kinect
			short *depth = 0;
			//char *rgb = 0;
			uint32_t ts;
			if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
				no_kinect_quit();
			//if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
				//no_kinect_quit();

			// update mesh to reflect kinect point cloud
			kinectVoxels.reset();
			int j,k;
			int n = 0;
			unsigned m = 0;
			for (j = 0; j < kinectH; j++) {
				for (k = 0; k < kinectW; k++) {	
					// calculate the position in local bubble space
					//double z = 0.1236 * tan((double)depth[i*kinectW+j] / 2842.5 + 1.1863);
					double z = ((double)depth[j*kinectW+k] - minimumDisparity) * (floatDim/(maximumDisparity - minimumDisparity));
					Vec3f pos(k / (float)kinectW * floatDim, j / (float)kinectW * floatDim, z);
					
					if ((pos - hashCenter).mag() > maxradius) { 
						// if the voxel is outside the sphere then hide this kinect data at the hashspace center
						pos = hashCenter; 
					}
					else {
						if(bubbles[kinectSphere].agents[m/boidifyStride].boidified) {
							// if its inside the sphere but is a boid then hide this kinect point
							pos = hashCenter;
						}
						else {
							// if in the sphere, and not a boid, then update a corresponding point in the boid space;
							// since there are way fewer boids in a sphere than kinect voxels, then we have to select them with a stride;
							if (m % boidifyStride == 0) { bubbles[kinectSphere].space.move(m / boidifyStride, pos); }
						}
					}
	
					// find the point's coordinate in world space
					Vec3f worldPos(pos + bubbles[kinectSphere].origin);
				
					// update the kinect voxels.
					kinectVoxels.vertex(worldPos);
					kinectVoxels.color(0.5, 0, 0);
					kinectVoxels.texCoord(5, 0);

					// Right now we aren't really setting the color of kinect voxels, due to bandwidth constraints
					//kinectVoxels.color(((float)rgb[n++])/255, ((float)rgb[n++])/255, ((float)rgb[n++])/255, 1.0);

					// update shared state every second depth value, combining this value and the last into one float value
					//state().kinectVoxels[m].depth = depth[j*kinectW+k];
					if (m % 2 == 0) { state().voxData[m] = (float)((uint32_t)depth[j*kinectW+k] << 16 | (uint32_t)depth[j*kinectW+k-1]); }
					
					// update m, which is the state voxel index
					m = m + 1;
				}
			}
		}

		// update each sphere's little world
		vector<Vec3f>& position(boids.vertices());
		vector<al::Color>& colors(boids.colors());
		// if we're streaming kinect, then the boids are tacked on to the end. If we're doing boids-only, then start from the beginning
		unsigned voxStateIdx = kinectStreaming ? NKinect / 2 : 0;
		for (unsigned i = 0; i < NSpheres; i++){
			// update the sphere's origin location by increasing phase
			bubbles[i].origin = Vec3f(sphereRingR * cos(i * M_2PI/NSpheres + bubbleRingPhase), 0, sphereRingR * sin(i * M_2PI/NSpheres + bubbleRingPhase));
			state().shareSpheres[i].origin = bubbles[i].origin; // update state
			bubbleRingPhase += dt * rotateSpeed;

			// update the colors in the shells
			vector<al::Color>& sphereColors(spheres[i].colors()); 
			for (unsigned j = 0; j < sphereColors.size(); j++) {
				float adjustment = 0;
			  	if(rnd::prob(0.1)) { adjustment = rnd::uniform() * 2 - 1; sphereColors[j] += adjustment; sphereColors[j].a = shellTransparency; }
				// add adjustment to state for effect to be recreated on the other end.
				state().shareSpheres[i].vertBrightnesses[j] = adjustment;
			}

			// skip this sphere's boid updates if the kinect is streaming and there's no kinect data there
			if (kinectStreaming && i != kinectSphere) { continue; }

			for (unsigned id = 0; id < boidsPerSphere; id++) {		
				// pass this boid by if we're streaming and the the boid isn't boidified
				if (kinectStreaming && !bubbles[i].agents[id].boidified) { continue; }				

				// collect boid from hashspace
				HashSpace::Object& o = bubbles[i].space.object(id);

				// find flock mates
				qmany.clear();
				int flockMates = qmany(bubbles[i].space, o.pos, KDist * floatDim);
				
				// accumulate boid forces
				Vec3f boidForce = Vec3f(0,0,0);
				for(unsigned j = 1; j < flockMates; j++) {			
					Vec3f vec2other = qmany[j]->pos - o.pos;
					Vec3f vec2mid = vec2other / 2;
					boidForce += vec2mid * cohesionModifier; // cohesion
					boidForce -= vec2other * separationModifier; // separation
					Agent& B = bubbles[i].agents[qmany[j]->id];
					Agent& A = bubbles[i].agents[id];
					boidForce += Vec3f((B.velocity - A.velocity).normalize(alignmentModifier)); // alignment
				}
				
				// integrate forces into mesh position
				bubbles[i].agents[id].acceleration += boidForce / bubbles[i].agents[id].mass;
				bubbles[i].agents[id].velocity += bubbles[i].agents[id].acceleration * dt;
				bubbles[i].space.move(id, o.pos + (bubbles[i].agents[id].velocity * dt));

				// clearing acceleration
				bubbles[i].agents[id].acceleration.zero();

				// modulo space check
				Vec3f vec2Origin = hashCenter - o.pos;
				//if(vec2Origin.mag() >= maxradius) { bubbles[i].space.move(id, o.pos + vec2Origin * 1.95); }
				if(vec2Origin.mag() >= maxradius) { bubbles[i].space.move(id, o.pos + vec2Origin * 0.05); bubbles[i].agents[id].velocity *= -1; }

				// get index of boid in mesh
				unsigned idx = i * boidsPerSphere + id;

				// get world position
				Vec3f worldPos(o.pos + bubbles[i].origin);

				// update mesh position to hashspace + origin
				position[idx] = worldPos;

				// update the state
				for (unsigned j = 0; j < 3; j++) {
					state().voxData[voxStateIdx++] = position[idx][j];
				}
				for (unsigned j = 0; j < 3; j++) {
					state().voxData[voxStateIdx++] = colors[idx][j];
				}
				
				// update trail
				//Vec3f& trailPosition(trails[idx].vertices()[trailIdx]);
				//trailPosition = worldPos;
			} 
		}

		// update the AI attractor
		//theAI.rho = rho.get();
		theAI.sigma = sigma.get();
		//theAI.beta = beta.get();
		theAI.beta = sin(t++ * 0.002) * (10 - 5) + 5;
		theAI.rho = sin(t * 0.001) * (55 - 29) + 29;
		vector<Vec3f>& aiVerts(aiAttractor.vertices());
		aiVerts[0] = Vec3f(1.0,1.0,1.0); // reset first vertice
		for (unsigned i = 1; i < NAttractorVerts; i++) {
			aiVerts[i] = aiVerts[i-1] + theAI.getAttractorDerivative(aiVerts[i-1] * 0.01);
			state().attractorVox[i] = aiVerts[i]; // updating the state
		}

		// Update the background color state
		state().backgroundColor = Color(backgroundBrightness, backgroundBrightness, backgroundBrightness);

		// Update the nav pose state
		state().navPos = nav().pos();
		state().navQuat = nav().quat();
	}
	else { // if we're a renderer then
		// update local state according to shared state

		// first update boid state
		unsigned N = kinectStreaming ? boidsPerSphere + (NKinect / 2) : NBoids; // if we're streaming, then only send one sphere's worth of boids at the end of the thing
		vector<Vec3f>& boidVerts(boids.vertices());
		vector<al::Color>& boidColors(boids.colors());
		for (unsigned i = kinectStreaming ? NKinect : 0; i < N; i++) {
			boidVerts[i] = Vec3f(state().voxData[i*6+0], state().voxData[i*6+1], state().voxData[i*6+2]);
			boidColors[i] = Color(state().voxData[i*6+3], state().voxData[i*6+4], state().voxData[i*6+5]);
			//boidVerts[i] = state().boidVoxels[i].pos;
			//boidColors[i] = state().boidVoxels[i].col;
		}
		
		// then update sphere state
		for (unsigned i = 0; i < NSpheres; i++) {
			bubbles[i].origin = state().shareSpheres[i].origin;
			vector<al::Color>& sphereColors(spheres[i].colors());
			for (unsigned j = 0; j < sphereColors.size(); j++) {
				sphereColors[j] += state().shareSpheres[i].vertBrightnesses[j];
				sphereColors[j].a = shellTransparency;
			}
		}

		// then update kinect state	
		unsigned M = kinectStreaming ? NKinect : 0; // if we're streaming, then send all the kinect data
		kinectVoxels.reset();
		for (unsigned i = 0; i < M; i++) {
			// Since every Ith element in state().voxData contains the shorts for two voxels, then every odd index in this loop needs to look backwards (i - 1) to find
			// the element its looking for. For evens, the short is in the two higher significant bytes, so we have to shift right two bytes
			// In both cases, we have to mask off the leftmost bits
			uint32_t d = (i % 2 == 0) ? ((uint32_t)state().voxData[i / 2] >> 16) & 0xFFFF : (uint32_t)state().voxData[(i - 1) / 2] & 0xFFFF;
			//if (i % 2 == 0) { d = ; } else { d =  } // take the appropriate parts of the stuff
			float x = floor(i / kinectW);
			float y = i % kinectW;
			float z = ((float)d - minimumDisparity) * (floatDim/(maximumDisparity - minimumDisparity));	
			Vec3f pos(x / (float)kinectW * floatDim, y / (float)kinectW * floatDim, z);
			if ((pos - hashCenter).mag() > maxradius) { 
				// if the voxel is outside the sphere then hide this kinect data at the hashspace center
				pos = hashCenter; 
			}
			kinectVoxels.vertex(pos + bubbles[kinectSphere].origin);
			kinectVoxels.color(0.5, 0, 0);
			kinectVoxels.texCoord(0.5, 0);
		}

		vector<Vec3f>& aiVerts(aiAttractor.vertices());
		for (unsigned i = 0; i < NAttractorVerts; i++) {
			aiVerts[i] = state().attractorVox[i];
		} 

		// we don't need to update any local background color state on the renderer side, we'll just use the state directly
	}

	// update things on both the simulator and the renderer
	// update the vertices of bubble shells	
	for (unsigned i = 0; i < NSpheres; i++) {
		vector<Vec3f>& verts(spheres[i].vertices());
		for (unsigned j = 0; j < verts.size(); j++) {
			verts[j] = bubbles[i].shellVerts[j] + hashCenter + bubbles[i].origin;
		}
	}

	// update the ring buffer idx for trails - currently unused
	trailIdx += 1;
	trailIdx = trailIdx > trailLength * NBoids ? 0 : trailIdx;
  }

  void onDraw(Graphics& g) override {
	// refresh the screen to some background color over which everything is drawn
	g.clear(state().backgroundColor);

	// allow alpha values to work as intended
	gl::blending(true);
	gl::blendTrans();

	// create and enable a light
	static Light light;
	g.lighting(true);
	g.light(light);

	// draw all the spheres, with their mesh colors - use depth testing
	for (unsigned i = 0; i < NSpheres; i++) {
		gl::depthTesting(true);
		g.meshColor();
		g.draw(spheres[i]);
	}

	g.pushMatrix();
	g.scale(attractorScale, attractorScale, attractorScale);
	g.translate(-1,-1,-1);
	g.draw(aiAttractor);
	g.popMatrix();

	// disable depth testing so now everything is painterly
	gl::depthTesting(false);
	
	// bind the shader that will render all the points
	g.shader(pointShader);
	g.shader().uniform("pointSize", pointSize.get() / 100);
	
	// if we're streaming the kinect, then draw the kinectvoxels
	if (kinectStreaming) { g.draw(kinectVoxels); }

	// whether we're streaming the kinect or not, always draw the boids
	g.draw(boids);
	
	// draw the trails - currently unused
	for (unsigned id = 0; id < NBoids; id++) {
		//g.draw(trails[id]);
	}
	
	// draw the gui if we're the graphics context on the simulator machine
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
//printf("%lu\n", sizeof(SharedState)); return 0;
//Mesh m ; addIcosphere(m, 1, 4); printf("%d\n", m.vertices().size()) ; return 1;
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
