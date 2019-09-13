#!/bin/bash

# Dependencies for robotArmRepo
#make sure you are in the robotArmRepo project when you run this script

cd ../../addons

PREFIX="git clone http://github.com/"
${PREFIX}CreativeInquiry/ofxTiming
${PREFIX}kylemcdonald/ofxCV
${PREFIX}CreativeInquiry/ofxNatNet
${PREFIX}NickHardeman/ofxGizmo
${PREFIX}CreativeInquiry/ofxTimeline
${PREFIX}CreativeInquiry/ofxURDriver
${PREFIX}CreativeInquiry/ofxPtf
${PREFIX}CreativeInquiry/ofxIKArm
${PREFIX}arturoc/ofxEasing
${PREFIX}moebiussurfing/ofxGuiExtended2

# Dependencies for ofxTimeline
${PREFIX}YCAMInterlab/ofxTimecode.git
${PREFIX}obviousjim/ofxTween.git
${PREFIX}obviousjim/ofxMSATimer.git
${PREFIX}elliotwoods/ofxTextInputField.git
echo "If you're using linux, please make sure you checkout the develop branch of ofxTextInputField"
${PREFIX}Flightphase/ofxRange.git
${PREFIX}prisonerjohn/ofxAudioDecoder.git

cd ../addons/ofxRobotArm
