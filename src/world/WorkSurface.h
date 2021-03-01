//Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#pragma once
#include "ofMain.h"
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
        
        virtual void setup(RobotParameters * parameters)=0;
        virtual void update(Pose _currentTCP)=0;
        virtual void update()=0;
        virtual void draw()=0;
        virtual void draw(bool showNormals)=0;
        virtual void keyPressed(int key)=0;
        virtual vector<Path *> getPaths() = 0;
        
        virtual void transform(ofVec3f pos)=0;
        virtual void transform(ofMatrix4x4 m44)=0;
        
        virtual Pose getTargetPoint(float t)=0;
        virtual void addPoint(ofVec3f pt)=0;
        virtual void addStroke(ofPolyline stroke)=0;
        virtual void setCorners(vector<ofPoint> pts)=0;
        virtual void setCorner(CORNER i, ofPoint pt)=0;
        virtual void addStrokes(vector<ofPolyline> strokes, float retractDist = 1)=0;
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
