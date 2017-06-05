//
//  WorkSurface3D.cpp
//  example-surface-following
//
//  Created by mad on 4/26/16.
//
//
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "WorkSurface3D.h"
namespace ofxRobotArm{
    WorkSurface3D::WorkSurface3D(){
        
    }
    WorkSurface3D::~WorkSurface3D(){
        
    }
    
    void WorkSurface3D::setup(){
        ofMesh m;
        surfaceMesh = m;
    }
    
    void WorkSurface3D::setup(ofMesh mesh){
        
        surfaceMesh = mesh;
        
        // scale surface to meters and reposition for robot
        ofVec3f offset = ofVec3f(0,.5,0);
        for (auto &v : surfaceMesh.getVertices()){
            v /= 100;
            v+=offset;
        }
        
    }
    
    void WorkSurface3D::setup(ofMesh mesh, vector<ofPolyline> polylines){
        
        surfaceMesh = mesh;
        
        // scale surface to meters and reposition for robot
        ofVec3f offset = ofVec3f(0,.5,0);
        for (auto &v : surfaceMesh.getVertices()){
            v /= 100;
            v+=offset;
        }
        
        // center polylines onto mesh
        for (auto &pl : polylines){
            for (auto &v : pl.getVertices())
                v+=offset;
        }
        
        
        vector<ofPolyline> polylines3D;
        project(surfaceMesh, polylines, polylines3D,0);
        
        for (auto &pl : polylines3D){
            Path3D p;
            p.set(pl);
            paths.push_back(p);
        }
        
    }
    
    void WorkSurface3D::draw(bool showSrf=true, bool showWireframe=true, bool showNormals=false, bool showPaths=true){
        ofPushMatrix();
        ofPushStyle();
        ofScale(1000, 1000, 1000); // scale from meters to mm when drawing
        
        ofSetColor(255, 0, 255, 100);
        if (showSrf)
            surfaceMesh.draw();
        if (showWireframe){
            ofSetColor(ofColor::aqua, 100);
            surfaceMesh.drawWireframe();
        }
        if (showNormals){
            ofSetColor(ofColor::aqua, 100);
            for (auto &face : surfaceMesh.getUniqueFaces()){
                ofVec3f n = face.getFaceNormal();
                n /= -100; // scale to meters & flip
                ofVec3f pos = (face.getVertex(0) + face.getVertex(1) + face.getVertex(2)) / 3;
                ofDrawLine(pos, pos+n);
            }
        }
        if (showPaths){
            ofSetLineWidth(3);
            ofSetColor(0, 255, 255, 200);
            for (auto &p : paths){
                p.draw();
            }
        }
        
        ofPopStyle();
        ofPopMatrix();
    }
    
    
    void WorkSurface3D::keyPressed(int key){
        
        ofVec3f offset;
        offset.set(0,0,0);
        
        float step = .01;
        
        // move the work surface
        if (key == OF_KEY_LEFT)
            offset.x -= step;
        else if (key == OF_KEY_RIGHT)
            offset.x += step;
        else if (key == OF_KEY_DOWN)
            offset.y -= step;
        else if (key == OF_KEY_UP)
            offset.y += step;
        
        transform(offset);
        
        // rotate the work surface (in place)
        if (key == 'R'){
            ofMatrix4x4 m44;
            ofVec3f centroid = getMesh().getCentroid();
            
            // rotate about the x-axis 90º
            m44.setRotate(ofQuaternion(sqrt(.5),0,0,sqrt(.5)));
            
            // move to origin to transform
            m44.setTranslation(-1*centroid);
            // apply tranformation
            transform(m44);
            // move back to origin
            transform(centroid);
        }
    }
    
    void WorkSurface3D::setMesh(ofMesh mesh, vector<ofPolyline> polylines2D){
        surfaceMesh = mesh;
        
        setPaths(polylines2D);
    }
    
    ofMesh WorkSurface3D::getMesh(){
        return surfaceMesh;
    }
    
    void WorkSurface3D::setPaths(vector<ofPolyline> polylines2D){
        
        vector<ofPolyline> polylines3D;
        project(surfaceMesh, polylines2D, polylines3D, 0);
        
        paths.clear();
        for (auto &pl : polylines3D){
            Path3D p;
            p.set(pl);
            paths.push_back(p);
        }
    }
    
    vector<Path *> WorkSurface3D::getPaths(){
        
        vector<Path *> pPaths;
        for (auto &path : paths){
            
            pPaths.push_back(&path);
            
        }
        return pPaths;
    }
    
    
    void WorkSurface3D::project(ofMesh & mesh, vector<ofPolyline> &paths2D, vector<ofPolyline> &paths, float srfOffset){
        for (int i=0; i<paths2D.size(); i++){
            ofPolyline pl = paths2D[i];
            
            ofPolyline temp3D;
            
            // find the closest face to the 2D path point
            for (auto &v : pl.getVertices()){
                float zHeight = v.z; // save the z height of the vertex
                v.z = 0;             // make point into 2D point
                for (int i=0; i<mesh.getUniqueFaces().size(); i++){
                    
                    // re-make the face as a 2D polyline so we can check
                    // if the toolpath point is inside ... hacky, but it works!
                    ofPolyline f;
                    f.addVertex(mesh.getFace(i).getVertex(0));
                    f.addVertex(mesh.getFace(i).getVertex(1));
                    f.addVertex(mesh.getFace(i).getVertex(2));
                    f.close();
                    f.getVertices()[0].z = 0;
                    f.getVertices()[1].z = 0;
                    f.getVertices()[2].z = 0;
                    
                    
                    // project the 2D point onto the 2D mesh face
                    if (f.inside(v)){
                        auto face = mesh.getFace(i);
                        
                        // find the distance between our toolpath point and the mesh face
                        ofVec3f facePos = (mesh.getFace(i).getVertex(0)+mesh.getFace(i).getVertex(1)+mesh.getFace(i).getVertex(2))/3;
                        ofVec3f face2toolPt = v - facePos;
                        float projectedDist = face2toolPt.dot(face.getFaceNormal().getNormalized());
                        
                        // use the distance as the length of a vertical projection vector
                        ofVec3f length = ofVec3f(0,0,-projectedDist - srfOffset);
                        
                        // get point on surface
                        ofVec3f projectedPt = v-length;
                        
                        // preserve any 3D offset normal to the surface
                        ofVec3f nOffset = face.getFaceNormal();
                        nOffset.scale(zHeight+srfOffset);
                        
                        projectedPt -= nOffset;
                        
                        // save the projected point and face normal
                        temp3D.addVertex(projectedPt);
                    }
                }
                
            }
            temp3D.close();
            paths.push_back(temp3D);
        }
        
    }
    
    void WorkSurface3D::transform(ofVec3f p){
        
        for (auto &v: surfaceMesh.getVertices()){
            v += p;
        }
        for (auto &path : paths){
            // move the path
            for (auto &v : path.path.getVertices())
                v += p;
            // update the perp frames
            path.buildPerpFrames(path.path);
        }
    }
    
    void WorkSurface3D::transform(ofMatrix4x4 m44){
        
        for (auto &v: surfaceMesh.getVertices()){
            v += m44.getTranslation();
            v  = v * m44.getRotate();
        }
        for (auto &path : paths){
            // move the path
            for (auto &v : path.path.getVertices()){
                v += m44.getTranslation();
                v  = v * m44.getRotate();
            }
            // update the perp frames
            path.buildPerpFrames(path.path);
        }
        
        
    }
}
