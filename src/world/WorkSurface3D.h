//
//  WorkSurface3D.h
//  example-surface-following
//
//  Created by mad on 4/26/16.
//
//

#pragma once
#include "ofMain.h"
#include "WorkSurface.h"
#include "Path3D.h"

class WorkSurface3D : public WorkSurface{
public:
    WorkSurface3D();
    ~WorkSurface3D();
    
    void setup();
    void setup(ofMesh mesh);
    void setup(ofMesh mesh, vector<ofPolyline> polylines2D);
    
    void draw(bool showSrf, bool showWireframe, bool showNormals, bool showPaths);
    void keyPressed(int key);
    
    void setMesh(ofMesh mesh, vector<ofPolyline> polylines2D);
    ofMesh getMesh();
    
    void setPaths(vector<ofPolyline> polylines3D);
    vector<Path *> getPaths();
    
    vector<Path3D> paths;
    
    void transform(ofVec3f p);
    void transform(ofMatrix4x4 m44);
  
private:
    void project(ofMesh & mesh, vector<ofPolyline> &paths2D, vector<ofPolyline> &paths, float srfOffset);
    
    
    
};
