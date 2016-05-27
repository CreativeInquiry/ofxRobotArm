#include "WorkSurface.h"
void WorkSurface::setup(){
    workSurfacePrarms.setName("Work Surface");
    workSurfacePrarms.add(position.set("WS Position", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    workSurfacePrarms.add(size.set("WS size", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    workSurfacePrarms.add(rotation.set("WS Euler", ofVec3f(0, 0, 0), ofVec3f(-360, -360, -360), ofVec3f(360, 360, 360)));
    workSurfacePrarms.add(qAxis.set("qAxis", ofVec3f(0, 0, 0), ofVec3f(-360, -360, -360), ofVec3f(360, 360, 360)));
    workSurfacePrarms.add(qAngle.set("qAngle",0, 0, 360));
    workSurfacePrarms.add(fixOrientationX.set("Fix X", 0, 0, 360));
    workSurfacePrarms.add(fixOrientationY.set("Fix Y", 0, 0, 360));
    workSurfacePrarms.add(fixOrientationZ.set("Fix Z", 0, 0, 360));
    for(int i = 0; i < 4; i++){
        targetPoints.push_back(ofParameter<ofPoint>());
        workSurfacePrarms.add(targetPoints.back().set("TP-"+ofToString(i), ofPoint(1/(i+1), 1/(i+1), 1/(i+1)), ofPoint(-1, -1, -1), ofPoint(1, 1, 1)));
    }
    warp.setup();
    corners.assign(4, ofPoint(0, 0, 0));
    
    plane.setPosition(0, 0, 0);
    plane.setWidth(1);
    plane.setHeight(1);
    targetIndex = 0;
    
}
void WorkSurface::setCorners(vector<ofPoint> pts){
    corners = pts;
    
}
void WorkSurface::setCorner(CORNER i, ofPoint pt){
    switch (i) {
        case UL:
            corners[0] = pt;
            targetPoints[0] = corners[0];
            break;
        case UR:
            corners[1] = pt;
            targetPoints[1] = corners[1];
            break;
        case LL:
            corners[2] = pt;
            targetPoints[2] = corners[2];
            break;
        case LR:
            corners[3] = pt;
            targetPoints[3] = corners[3];
            break;
    }
}

void WorkSurface::update(){
    
    // update the worksurface mesh
    if (mesh.getVertices().size() == 0){
        for (int i=0; i<targetPoints.size(); i++)
            mesh.addVertex(targetPoints[i].get());
            mesh.addTriangle(0, 1, 2);
            mesh.addTriangle(0, 2, 3);
    }
    else{
        mesh.setVertex(0, targetPoints[0]);
        mesh.setVertex(1, targetPoints[1]);
        mesh.setVertex(2, targetPoints[2]);
        mesh.setVertex(3, targetPoints[3]);
    }

    // update the mesh normal as the average of its two face normals
    ofVec3f n = (mesh.getFace(0).getFaceNormal() + mesh.getFace(1).getFaceNormal())/2;
    
    orientation.set(n);
    
    // realign the local axis of the worksurface
    //      this is a bit hacky ... I don't think it works for everything
    ofQuaternion conj = orientation.conj();
    orientation *= conj;
    orientation.makeRotate(-45, 0, 0, -1);
    orientation *= conj.conj();
    
    conj = orientation.conj();
    orientation *= conj;
    orientation.makeRotate(-90, 1, 0, 0);
    orientation *= conj.conj();

    
//    ofPoint diffOne = targetPoints[0].get() - targetPoints[1].get();
//    ofPoint diffTwo = targetPoints[0].get() - targetPoints[3].get();
//    
//    position = targetPoints[2].get().getMiddle(targetPoints[0].get());
//    diffOne.normalize();
//    crossed = diffOne.cross(diffTwo);
//    crossed.normalize();
//    orientation.makeRotate(ofPoint(0, 0, -1), crossed);
    
    // assign new orientation
    ofVec3f axis;
    float angle;
    orientation.getRotate(angle, axis);
    rotation = orientation.getEuler();
    qAxis = axis;
    qAngle = angle;
    
    // update the position
    ofVec3f centroid;
    for (auto &p : targetPoints)
        centroid += p;
    position = centroid/4;//targetPoints[2].get().getMiddle(targetPoints[0].get());
    
    // update GML
    if (strokes_original.size() != 0)
        addStrokes(strokes_original);
    

}
void WorkSurface::addPoint(ofVec3f pt){
    
}
void WorkSurface::addStroke(ofPolyline stroke){
    
}
void WorkSurface::addStrokes(vector<ofPolyline> strokes){
    
    // store the original linework
    if (strokes_original.size() == 0){
        ofPolyline pl;
        for (auto &stroke : strokes){
            for (auto &v : stroke.getVertices())
                pl.addVertex(v);
        }
        strokes_original.push_back(pl);
    }
    
    
    // get the centroid of the line drawing
    ofVec3f centroid;
    for (auto &pl : strokes)
       centroid += pl.getCentroid2D();
    centroid /= strokes.size();

    // scale & align linework
    float height = (targetPoints[0].get() - targetPoints[3].get()).length();
    float width = (targetPoints[0].get() - targetPoints[1].get()).length();
    float multiply = MAX(height, width);
    lines.clear();
    ofMatrix4x4 mat;
    mat.makeRotationMatrix(orientation);
    mat.setTranslation(position);//targetPoints[3]); // center stroke on canvas
    for(int i = 0; i < strokes.size(); i++){
        ofPolyline fooLine;
        for(int j = 0; j < strokes[i].getVertices().size(); j++){
            // center stroke on canvas
            strokes[i].getVertices()[j] -= centroid;
            
            fooLine.addVertex(strokes[i].getVertices()[j]*multiply*mat);
        }
        lines.push_back(fooLine);
    }
}

Joint WorkSurface::getTargetPoint(float t){
    Joint foo;
    if(lines.size() > 0){
        float length = lines[targetIndex].getLengthAtIndex(lines[targetIndex].getVertices().size()-1)/0.05;
        t = fmodf(t, length)/length;
        
        float indexAtLenght = lines[targetIndex].getIndexAtPercent(t);
        ofPoint p = lines[targetIndex].getPointAtIndexInterpolated(indexAtLenght);
        foo.position = p;
        foo.rotation = orientation;
        if(1.0-t < 0.01 || t == 0.0){
            targetIndex++;
            if(targetIndex >=lines.size()){
                targetIndex = 0;
            }
        }
    }
    return foo;
}
void WorkSurface::draw(){
    ofPushMatrix();
    ofTranslate(position.get()*1000);
    float angle;
    ofVec3f axis;
    orientation.getRotate(angle, axis);
    ofRotate(angle, axis.x, axis.y, axis.z);
    ofDrawAxis(100);
    ofPopMatrix();
    ofPushMatrix();
    ofScale(1000, 1000, 1000);
    for(int i = 0; i < lines.size(); i++){
        lines[i].draw();
    }
    mesh.drawWireframe();
    ofPopMatrix();
  
    ofPushMatrix();
    ofSetColor(255, 0, 255);
    ofDrawSphere(targetPoints[0], 10);
    ofDrawLine(targetPoints[0].get()*1000,targetPoints[1].get()*1000);
    ofSetColor(255, 255, 0);
    ofDrawSphere(targetPoints[1], 10);
    ofDrawLine(targetPoints[1].get()*1000,targetPoints[2].get()*1000);
    ofSetColor(255, 0, 255);
    ofDrawSphere(targetPoints[2], 10);
    ofDrawLine(targetPoints[2].get()*1000,targetPoints[3].get()*1000);
    ofSetColor(255, 255, 0);
    ofDrawSphere(targetPoints[3], 10);
    ofDrawLine(targetPoints[3].get()*1000,targetPoints[0].get()*1000);
    ofPopMatrix();
}
void WorkSurface::setRotationX(float x){
    orientationX.makeRotate(x, 1, 0, 0);
}
void WorkSurface::setRotationY(float y){
    orientationY.makeRotate(y, 0, 1, 0);
}
void WorkSurface::setRotationZ(float z){
    orientationZ.makeRotate(z, 0, 0, 1);
}
void WorkSurface::setRotation(float x, float y, float z){
    orientationX.makeRotate(x, 1, 0, 0);
    orientationY.makeRotate(y, 0, 1, 0);
    orientationZ.makeRotate(z, 0, 0, 1);
}