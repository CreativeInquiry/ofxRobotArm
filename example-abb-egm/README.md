# Basic ABB EGM Communications Example

#### Tested On:

- osx
- oF v.0.11.2 
- RobotStudio <-- NOT YET (INCLUDE A PACK-N-GO FILE IN THE REPO)


#### ofxRobotArm Dependencies

- ofxAssimpModelLoader
- ofxYAML
- ofxXmlSettings
- ofxTiming


## Overview

`example-abb-egm` is a minimal example to test connect, reading, and writing to an ABB robot using their EGM protocol.

1. Open a RobotStudio Solution with an ABB IRB120 robot and Virtual Controller — or plug directly into a real robot. 
2. Run `example-abb-egm` and press the  `CONNECT` button to connect / disconnect via EGM.
3. You can press the `GET CURRENT JOINTS` button to sync the `Joint Position` sliders with the Actual Joint Positions.
4. Move the `Joint Position` sliders to manually update each joint position of the actual robot.

> `ABBDriver` wraps `abb_libegm`, the nice ABB library that handles communication interface between PC and Robot.
