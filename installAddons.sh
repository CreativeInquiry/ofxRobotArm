#!/bin/bash

# Dependencies for robotArmRepo
#make sure you are in the robotArmRepo project when you run this script

cd ../../addons

PREFIX="git clone http://github.com/"
${PREFIX}CreativeInquiry/ofxTiming
${PREFIX}CreativeInquiry/ofxCV
${PREFIX}CreativeInquiry/ofxNatNet
${PREFIX}CreativeInquiry/ofxGML
${PREFIX}CreativeInquiry/ofxGizmo
${PREFIX}CreativeInquiry/ofxTimeline
# ${PREFIX}CreativeInquiry/ofxURDriver
${PREFIX}CreativeInquiry/ofxPtf

cd ../addons/ofxRobotArm
