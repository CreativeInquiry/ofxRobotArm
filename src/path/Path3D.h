#pragma once
#include "ofMain.h"
#include "ofxPtf.h"
#include "Path.h"
class Path3D : public Path{
public:
    /* Path Generator */
    void setup();
//    void setup(ofPolyline &polyline, vector<ofMatrix4x4> &m44);
    void set(ofPolyline &polyline);
    ofVec3f getNextNormal();
    ofMatrix4x4 getNextPose();
    ofMatrix4x4 getPoseAt(int index);
    void draw();
    void keyPressed(int key);
   
    /// \brief returns the number of perp frames in path
    /// \note this is 2 less than the number of points in the path
    int size();
    
    ofPoint centroid;
    bool pause;

    int getPtIndex();
    void setPtIndex(int index);
    
    bool reverse;
    int direction;
    
    /// \brief Creates a periodic 3D path.
    /// Adapted from: <a href="http://openframeworks.cc/ofBook/chapters/lines.html">ofBook/chapters/lines.html</a>
    ofPolyline buildPath();
    
    /// \brief polyline path
    ofPolyline path;
    
    /// \brief Perpendicular Frame Generator
    ofxPtf ptf;
    
    /// \brief Make the z-axis of the perp frame the forward-facing axis
    ///
    /// Note: by default the X-Axis is the forward-facing axis
    ofMatrix4x4 zForward(ofMatrix4x4 originalMat);
    bool makeZForward;
    
    /// \brief Make the z-axis of the perp frame the outwards-facing axis.
    ///
    /// Note: by default the X-Axis is the forward-facing axis
    ofMatrix4x4 zOut(ofMatrix4x4 originalMat);
    bool makeZOut;
    
    ofMatrix4x4 flip(ofMatrix4x4 originalMat);
    
    /// \brief orientation of current perp frame
    ofMatrix4x4 orientation;
    
    /// \brief Creates the 2D polygon to loft along the path
    /// \param radius radius of polygon
    /// \param res resolution of polygon
    ofPolyline buildProfile(float radius, int res);
    
    /// \brief Creates perpendicular frames on a path
    /// \param polyline path to create frames on
    void buildPerpFrames(ofPolyline polyline);
    
    /// \brief polygonal profile to loft
    ofPolyline profile;
    
    float feedRate;
    
    void parsePts(string filename, ofPolyline &polyline);
};
