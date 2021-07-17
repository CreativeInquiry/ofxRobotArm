# ofxRobotArm V2021.BETA.1
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

![IRB120](data/ezgif-6-9aa1f3bbb920%20(1).gif)

## About
`ofxRobotArm` is an openFrameworks addon for doing creative things with robot arms. The goal of the addon is to remove as many technical barriers as possible to get up and running with robots. We've included an example of how to get up and running with ofxRobotArm and your robot arm.  By developing `ofxRobotArm` for openFrameworks, we hope to help you extend human-robot interaction in new and diverse ways.

Currently the addon is configured to work with Universal Robot's and ABB robot arms (untested).  KUKA and xArm Support is pending. 

Development for `ofxRobotArm` was sponsored by [The Frank-Ratchye STUDIO for Creative Inquiry](http://studioforcreativeinquiry.org/). 
Technical development was lead by [Dan Moore](http://makeitdoathing.com) and [Madeline Gannon](https://atonaton.com).

### For Mac and Linux Only (Pending Windows support the URDriver needs to be rewritten)

![KinematicModel](data/ezgif.com-video-to-gif%20(1).gif)

## Dependencies
After download, you can run the `installAddons.sh` script to clone all the external addons used in the `ofxRobotArm` examples and the core of `ofxRobotArm`.  

Non Core Addon depedencies
- ofxGizmo
- ofxEasing
- ofxTiming
- ofxYAML
- ofxSTL

You will need to download the other precompiled dependencies via GIT LSF

- libabb_libegm
- libabb_librws
- protobuf
- relaxedIK

On Linux You will need to install boost

apt install libboost-all-dev

On Mac you will need to unzip the boost.zip and replace the version shipped with openFrameworks. 


## Overview
`ofxRobotArm` is structured in multiple parts: [Controllers](/src/controllers), [Drivers](/src/drivers), [Kinematics](/src/kinematics), [Models](/src/models), [Path](/src/path), [Utils](/src/utils), and [World](/src/world).  [Controllers](/src/controllers) contain the main RobotController and other useful controllers.  [Drivers](/src/drivers) contains the drivers for each manufacturer ofxRobotArm supports: Universal Robots, ABB, xArm, and many more are coming.  [Kinematics](/src/kinematics) contains the forward and inverse kinematic solvers.  [Model](/src/model) contains the representation of the robot arm loaded from a URDF.  [Path](/src/path) contains helper classes for interfacing with paths.  [Utils](/src/utils) contain many useful utilities. [World](/src/world) contains useful classes for describing the working enviroment. 


## Examples
We've also included an [example project](example-urdf) that show you the most common ways of programming 6-axis robots:

    **1. FOLLOW_GIZMO 
    The Robot follows the end effector goal position and prientation represented as a gizmo. This allows for direct manipulation of the end effector pose.
    **2. LOOK_AT_TARGET
    The orientation of the end effector is set by the look-at transform from the end effector Pose to the look at target gizmo.  
    **3. FOLLOW_CIRCLE
    This mode follows a sprial path that is generated at run time.  The orientation is set by the end effector goal gizmo. 

## Licensing
`ofxRobotArm` is licensed under the [THE ANTI-CAPITALIST SOFTWARE LICENSE (v 1.4)](LICENSE) 


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
 There are several known issues but please report anything you see out of place or weird.  There are known weirdnesses with the initial orientation with some bots, requiring the invsere of the read orientation.  If you would like to help out with this issue please go [here](https://github.com/CreativeInquiry/ofxRobotArm/issues/32)
 
 ## Building on Big Sur and XCode Version 12.5.1 (12E507) is not working at the moment!
 

## Future Development

* CollisionIK
* More Robot Arms!


