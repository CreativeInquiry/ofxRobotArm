#!/bin/bash

# Dependencies for robotArmRepo
#make sure you are in the robotArmRepo project when you run this script

cd ../../addons

PREFIX="git clone http://github.com/"
${PREFIX}kylemcdonald/ofxTiming
${PREFIX}kylemcdonald/ofxCV
${PREFIX}CreativeInquiry/ofxNatNet
${PREFIX}danzeeeman/ofxGML
${PREFIX}danzeeeman/ofxGizmo
${PREFIX}danzeeeman/ofxTimeline

cd ../apps/robotArmRepo
