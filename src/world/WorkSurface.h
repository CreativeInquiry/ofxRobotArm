//Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#pragma once
#include "ofMain.h"
//#include "ofxTiming.h"
//#include "UR5KinematicModel.h"
#include "RobotParameters.h"
#include "Path3D.h"
#include "PathController.h"
#include "ofxTiming.h"
namespace ofxRobotArm{
    class WorkSurface{
    public:
        
        WorkSurface(){};
        ~WorkSurface(){};
        
        enum CORNER{
            UL = 0,
            UR,
            LL,
            LR
        };
        
        virtual void setup(RobotParameters * parameters){};
        virtual void update(Pose _currentTCP){};
        virtual void update(){};
        virtual void draw(){};
        virtual void draw(bool showNormals){};
        virtual void keyPressed(int key){};
        virtual vector<Path *> getPaths(){};
        
        virtual void transform(ofVec3f pos){};
        virtual void transform(ofMatrix4x4 m44){};
        
        virtual Pose getTargetPoint(float t){};
        virtual void addPoint(ofVec3f pt){};
        virtual void addStroke(ofPolyline stroke){};
        virtual void setCorners(vector<ofPoint> pts){};
        virtual void setCorner(CORNER i, ofPoint pt){};
        virtual void addStrokes(vector<ofPolyline> strokes, float retractDist = 1){};
        ofParameterGroup workSurfaceParams;
        
        vector<Path> paths;
        
    protected:
        
        RobotParameters * parameters;
        ofParameter<ofVec3f> position;
        ofParameter<ofVec3f> rotation;
        ofParameter<float> retractDistance;
        ofParameter<float> drawingScale;
        ofParameter<float> rotateDrawing;
        ofParameter<ofVec3f> drawingOffset;
        ofParameter<float> feedRate;
        
        ofMesh surfaceMesh;
        Pose currentTCP;
        Pose targetTCP;
        
        ofQuaternion orientation;
        vector<ofPolyline> lines;
        vector<ofPolyline> strokes_original;
        
        PathController pathController;
        
        
        ofPolyline workArea;
        float targetIndex;
        ofVec3f normal;
        float startTime;
        ofNode toolPoint;
        Pose targetToolPoint;
        RateTimer timer;
    };
}
