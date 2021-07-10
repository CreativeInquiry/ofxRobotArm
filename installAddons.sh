#!/bin/bash

# Dependencies for robotArmRepo
#make sure you are in the robotArmRepo project when you run this script

cd ../../addons

PREFIX="git clone http://github.com/"
${PREFIX}CreativeInquiry/ofxTiming
${PREFIX}NickHardeman/ofxGizmo
${PREFIX}arturoc/ofxEasing
${PREFIX}michaelbaisch/ofxYAML
${PREFIX}danzeeeman/ofxSTL
cd ../addons/ofxRobotArm
