# ofxRobotArm LINUX ONLY
###### An openFrameworks addon for controlling and interacting with robot arms.

  - [About](#about)
  - [Dependencies](#dependencies)
  - [Overview](#overview)
  - [Getting Started](#getting-started)
  - [Examples](#examples)
  - [Licensing](#licensing)
  - [Contact Info](#contact-info)
  - [Additional Resources](#additional-resources)
  - [Known Issues](#known-issues)


## About
`ofxRobotArm` is an openFrameworks addon for doing creative things with robot arms. The goal of the addon is to remove as many technical barriers as possible to get up and running with robots. We've included a number of examples that show you how different ways of controlling and interacting with a robot arm, including direct manipulation, geometry-based manipulation, motion capture-based interactions, and keyframe animation. By developing `ofxRobotArm` for openFrameworks, we hope to help you extend human-robot interaction in new and diverse ways.

Currently the addon is configured to work with Universal Robot's and ABB robot arms (untested).  KUKA Support is pending. 

Development for `ofxRobotArm` was sponsored by [The Frank-Ratchye STUDIO for Creative Inquiry](http://studioforcreativeinquiry.org/). 
Technical development was lead by [Dan Moore](http://makeitdoathing.com) and [Madeline Gannon](https://atonaton.com).

![KinematicModel](data/ezgif.com-video-to-gif%20(1).gif)

## Dependencies
After download, you can run the `installAddons.sh` script to clone all the external addons used in the `ofxRobotArm` examples and the core of `ofxRobotArm`.  

- ofxGizmo
- ofxEasing
- ofxTiming
- ofxIKArm

You will need to download the other precompiled dependencies via GIT LSF

- libabb_libegm
- libabb_librws
- protobuf
- relaxedIK

You will need to install boost

apt install libboost-all-dev


## Overview
`ofxRobotArm` is structured in multiple parts: [Controllers](/src/controllers), [Drivers](/src/drivers), [Kinematics](/src/kinematics), [Path](/src/path), [Utils](/src/utils), and [World](/src/world).  Controller contain the main RobotController.  Drivers contains the drivers for each manufacture ofxRobotArm supports: Universal Robots, ABB, KUKA, and many more.  Kinematics contains the kinematic model for each arm, inverse kinematic sovlers, and the relaxedIK sovler thread. Path contains helper classes for interfacing with paths.  Utils contain many useful utilities. World contains useful classes for describing the working enviroment. 


## Examples
We've also included a number of example projects that show you the most common ways of programming 6-axis robots:

**1. Direct Manipulation**
 - [`example-linux`](/example-linux) lets you use your mouse to drag around and rotate your robot.
 

## Licensing
`ofxRobotArm` is licensed under the [GPLv3](LICENSE) 


## Contact Info
**Dan Moore** | [Make It Do A Thing!](http://www.makeitdoathing.com ) | [@danzeeeman](https://github.com/danzeeeman)

**Madeline Gannon** | [A TON A TON](http://atonaton.com) | [@madelinegannon](https://github.com/madelinegannon)

**The Frank-Ratchye STUDIO for Creative Inquiry** | [studioforcreativeinquiry.org](http://studioforcreativeinquiry.org) | [@creativeinquiry](https://github.com/CreativeInquiry)


## Additional Resources
Here are some of the references and resources that have made `ofxRobotArm` possible:

- [ur_modern_driver](https://github.com/ThomasTimm/ur_modern_driver)
- [Script Manual](https://s3-eu-west-1.amazonaws.com/ur-support-site/18679/scriptmanual_en.pdf)
- [UR Kinematics](https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf)
- [UR Report](http://orbit.dtu.dk/files/117833332/Universal_Robot_report.pdf)


## Known Issues
 There are several known issues:

 

## Future Development

* Collision Detection
* More Robot Arms!


