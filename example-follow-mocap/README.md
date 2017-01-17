
##Follow Mocap Example


######This example shows you how to control the robot using a motion capture system.

![screenshot](screengrab-mocap.gif)

####What You'll Learn
_Follow Mocap_ goes over:
  *  Create 3D Paths using an Optitrack motion capture markers
  *  Move & reiorient 3D Paths using motion capture markers
  *  Move & reiorient the robot to follow paths using a PathController
  *  Move & reiorient the robot to follow a rigid body
  

####Calibrating the Worlds
To develop this example, we used an 6 camera OptiTrack setup, running [Motive:Tracker](https://www.optitrack.com/products/motive/tracker/)Tracker 1.9.0.

There's a bit of set up you'll need to do in Motive:Tracker before running the example:

1. Calibrate the Motion Capture volume.
2. Create a Rigid Body.
  * Give ~300m offset to the rigid body pivot point. This will keep the robot a set distance away from you as you move around.
3. Stream your Rigid Body or Marker data from Motive:Tracker to the openFrameworks app using [`ofxNatNet`](https://github.com/satoruhiga/ofxNatNet).
4. Align the motion capture coordinate system with the robot's coordinate system.
  * If you place a rigid body on the robot's TCP, you should see it moving & reorienting in the app the same as in real life.


In the app, use 'F' to have the robot start or stop following the rigid body.

![derp](derp.gif)

####Other Usage Notes
We rely on hotkeys for triggering a lot of the UI functionality. Below are the hotkeys we use that work across all the example projects included in ofxRobotArm.

KeyPressed commands for controlling the robot:
- m: move

KeyPressed commands for controlling gizmo:
- r: rotate
- g: translate
- s: scale

KeyPressed commands for viewport navigation:
- 1: Top View
- 2: Front View
- 3: Side View
- 4: Perspective View



