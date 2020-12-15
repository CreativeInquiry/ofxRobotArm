//
//  URToolHead.h
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
// Copyright (c) 2016,2020 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
namespace ofxRobotArm{
    struct Tool{
        ofMesh mesh;
        ofVec3f bBoxMin;
        ofVec3f bBoxMax;
    };

    class ToolHead{
    public:
        ToolHead(){
            
        };
        ~ToolHead(){
            
        };
        void setOrientation(ofQuaternion orientation);
        ofMatrix4x4 getMatrix();
        void setup();
        void update();
        void draw();
        void setTool(Tool t);
        Tool getCurrentTool();
    protected:
        Tool currentTool;
        vector<Tool> availableTools;
        ofMatrix4x4 rot;
    };
}
