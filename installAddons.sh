#!/bin/bash

# Dependencies for robotArmRepo
#make sure you are in the robotArmRepo project when you run this script

cd ../../addons

PREFIX="git clone http://github.com/"
${PREFIX}CreativeInquiry/ofxTiming
${PREFIX}kylemcdonald/ofxCv
${PREFIX}CreativeInquiry/ofxNatNet
${PREFIX}NickHardeman/ofxGizmo
${PREFIX}CreativeInquiry/ofxIKArm
${PREFIX}arturoc/ofxEasing


cd ../addons/ofxRobotArm
