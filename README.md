# The Burning Spheres of Dionysus
As it stands, this is a virtual kinetic sculpture designed to work in the Allosphere. It supports live streaming of 3D point clouds created from Kinect data, boids simuated with hashSpace kNN, and a lorentz attractor simulation.
## Dependencies
 * Allollib Playground
 * libfreenect (for the Kinect)
## How to Install
Ensure libfreenect is installed on your machine, and that the allolib_playground repo is cloned. Clone this repo into the Allolib_Playground directory.
## How to Run
1) CD to allolib_playground, then use the command `./run.sh TheBurningSpheresOfDionysus/burningSpheres.cpp`.
2) In order to see the distributed state work, open a second terminal and repeat the step above. This will open a second display that replicates the multi-screen effect in the Allosphere.
**To Enable Kinect Streaming** edit `MAT201B/burningSpheres/burningSpheres.cpp` and on line 30 in the `//tweak zone` change the `kinectInitiallyStreaming` value to `true`. Ensure the kinect is plugged in, then repeat the first two steps.
