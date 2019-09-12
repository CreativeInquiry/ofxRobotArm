//  Created by Dan Moore on 7/7/13.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

// Deprecated as of 09/12/2019
// ofxGML not compatible with OF 0.10+

/**
#pragma once
#include "ofxGML.h"
#include "Path.h"
namespace ofxRobotArm{
    class GMLPath : public Path{
    public:
        GMLPath(){};
        ~GMLPath(){};
        
        void setup();
        
        void setup(float x, float y, float width, float height);
        void loadFile(string _filepath);
        void draw();
        vector<ofPolyline> getPath(float scale);
        tagReader reader;
        vector<ofPolyline> polys;
        vector<ofPolyline> scaledLines;
        float aspectRatio;
        
    protected:
        string filepath;
    };
}
*/
