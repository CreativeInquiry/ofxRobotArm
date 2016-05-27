#include "Path3D.h"
void Path3D::setup(){
    // set the Z axis as the forward axis by default
    makeZForward = true;
    
    ptIndex = 0;
    centroid = ofPoint(.5,.25,.25); // all coordinates are in meters
    
    // load/create different paths
    parsePts("path_XZ.txt", path_XZ);
    parsePts("path_YZ.txt", path_YZ);
    parsePts("path_SPIRAL.txt", path_SPIRAL);
    path_PERIODIC = buildPath();
    
    ofPath p;
    p.setCircleResolution(2000);
    p.circle(0., 0., 0.3, 0.325);

    vector<ofPolyline> fooCircle = p.getOutline();
    
    // assign path and make profile
    profile = buildProfile(.025,4);
//    path = fooCircle[0];
    path = path_XZ;
    buildPerpFrames(path);
    
//    perpFrames.clear();
//    for (int i=0; i<ptf.framesSize(); i++){
//        perpFrames.push_back(ptf.frameAt(i));
//    }
    
    reverse = false;
    
    direction = 1;
}

//void Path3D::setup(ofPolyline &polyline, vector<ofMatrix4x4> &m44){
//    
//    
//    
//    ptIndex = 0;
//    
//    // ignore the first and last points for the centroid
//    for (int i=0; i<polyline.getVertices().size(); i++){
//        centroid += polyline.getVertices()[i];
//    }
//    centroid /= polyline.getVertices().size();
//
//    path = polyline;
//    perpFrames.clear();
//    perpFrames = m44;
//    
//}

void Path3D::set(ofPolyline &polyline){
    
    setup(); // incase path hasn't been set up yet
    
    ptIndex = 0;
    reverse = true;
    direction = 1;
    
    // ignore the first and last points for the centroid
    for (int i=1; i<polyline.getVertices().size()-1; i++){
        centroid += polyline.getVertices()[i];
    }
    centroid /= polyline.getVertices().size()-2;
    
    // assign path and make profile
//    profile = buildProfile(.025,4);
    path = polyline;
    buildPerpFrames(path);
    

}


void Path3D::keyPressed(int key){
        float step = .01;   // 10 millimeters
        
        if (key == OF_KEY_UP){
            for (auto &p : path.getVertices())
                p.y += step;
            buildPerpFrames(path);
        }else if(key == OF_KEY_DOWN){
            for (auto &p : path.getVertices())
                p.y -= step;
            buildPerpFrames(path);
        }else if(key == OF_KEY_RIGHT){
            for (auto &p : path.getVertices())
                p.x += step;
            buildPerpFrames(path);
        }else if(key == OF_KEY_LEFT){
            for (auto &p : path.getVertices())
                p.x -= step;
            buildPerpFrames(path);
        }else if (key == '!'){
            makeZOut = false;
            makeZForward = true;
        }
        else if (key == '@'){
            makeZForward = false;
            makeZOut = true;
        }
        else if (key == '#'){
            makeZForward = false;
            makeZOut = false;
        }
        
        else if (key == '$'){
            path = path_XZ;
            buildPerpFrames(path);
        }
        else if (key == '%'){
            path = path_YZ;
            buildPerpFrames(path);
        }
        else if (key == '^'){
            path = path_SPIRAL;
            buildPerpFrames(path);
        }
        else if (key == '&'){
            path = path_PERIODIC;
            buildPerpFrames(path);
        }
}


ofMatrix4x4 Path3D::getNextPose(){
    
    reverse = true;
    if(ptf.framesSize()>0){
        
        // go back-and-forth along a path
        if (reverse && (ptIndex == 0 || ptIndex == ptf.framesSize()-2))
            direction *= -1;
        
        ptIndex = (ptIndex + direction) % ptf.framesSize();
        
        orientation = ptf.frameAt(ptIndex);
        
//        if (makeZForward)
//            orientation = zForward(orientation);
//        else if (makeZOut)
//            orientation = zOut(orientation);
//        else
//            orientation = flip(orientation);
        
//        return flip(orientation);
        return orientation;
    }
}

ofMatrix4x4 Path3D::getPoseAt(int index){
    
    
    return ptf.frameAt(index);
    
}

int Path3D::getPtIndex(){
    return ptIndex;
};

void Path3D::setPtIndex(int index){
    ptIndex = index;
};


void Path3D::draw(){

    // show the current orientation plane
    ofSetColor(ofColor::lightYellow);
    ofSetLineWidth(3);
    ofPushMatrix();
    ofMultMatrix(orientation);
    profile.draw();
    ofDrawAxis(.010);
    ofPopMatrix();
    
    // show all the frames
    ofSetColor(ofColor::aqua,80);
    for (auto frame : ptf.getFrames()){
        ofPushMatrix();
        ofMultMatrix(frame);
        profile.draw();
//        ofDrawAxis(.010);
        ofPopMatrix();
    }
    
    // show the target point
    ofSetColor(ofColor::yellow);
    if (path.size() > 0){
//        ofDrawSphere(path.getVertices()[ptIndex], .003);
        ofDrawSphere(getPoseAt(ptIndex).getTranslation(), .003);
    }
    
    // show the 3D path
    ofSetLineWidth(3);
    ofSetColor(ofColor::aqua);
    path.draw();
    
}

int Path3D::size(){
    return ptf.framesSize();
}

//--------------------------------------------------------------
void Path3D::parsePts(string filename, ofPolyline &polyline){
    ofFile file = ofFile(ofToDataPath(filename));
    
    if(!file.exists()){
        ofLogError("The file " + filename + " is missing");
    }
    ofBuffer buffer(file);
    
    //Read file
    for (ofBuffer::Line it = buffer.getLines().begin(), end = buffer.getLines().end(); it != end; ++it) {
        string line = *it;
        
        float scalar = 10;
        
        ofVec3f offset;
        if (filename == "path_XZ.txt"){
            offset = ofVec3f(0, 0, 0);
            scalar = 6;
        }
        else if (filename == "path_YZ.txt"){
            offset = ofVec3f(0, 0, 0);
            scalar = 6;
        }
        else{
            offset = ofVec3f(0, 0, .25);
            scalar = 6;
        }
        
        line = line.substr(1,line.length()-2);              // remove end { }
        vector<string> coords = ofSplitString(line, ", ");  // get x y z coordinates
        
        ofVec3f p = ofVec3f(ofToFloat(coords[0])*scalar,ofToFloat(coords[1])*scalar,ofToFloat(coords[2])*scalar);
        p += offset;
        
        polyline.addVertex(p);
    }
    
    // interpolate points to smooth
    ofPolyline temp;
    
    for (int i=0; i<polyline.getVertices().size()-1; i++){
        
        ofVec3f p0 = polyline.getVertices()[i];
        ofVec3f p1 = polyline.getVertices()[i+1];
        
        for (int j=1; j<4; j++){
            float t = j/4.0;
            temp.addVertex(p0.interpolate(p1, t));
        }
        
    }
    
    polyline.clear();
    polyline = temp;
}

//--------------------------------------------------------------
ofPolyline Path3D::buildPath(){
    
    ofPolyline temp;
    
    ofNode n0;
    ofNode n1;
    ofNode n2;
    
    n0.setPosition(centroid.x,centroid.y,centroid.z);
    n1.setParent(n0);
    n1.setPosition(0,0,.1);
    n2.setParent(n1);
    n2.setPosition(0,.0015,0);
    
    float totalRotation = 0;
    float step = .25;
    while (totalRotation < 360){
        
        n0.pan(step);
        n1.tilt(2);
        n2.roll(1);
        
        ofPoint p = n2.getGlobalPosition().rotate(90, ofVec3f(1,0,0));
        
        // and point to path
        temp.addVertex(p);
        
        // add point to perp frames
        ptf.addPoint(p);
        
        totalRotation += step;
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------
void Path3D::buildPerpFrames(ofPolyline polyline){
    
    // reset the perp frames
    ptf.clear();
    
    for (auto &p : polyline){
        ptf.addPoint(p);
    }
    

    
}

//--------------------------------------------------------------
ofPolyline Path3D::buildProfile(float radius, int res){
    ofPolyline temp;
    
    // make a plane
    if (res == 4){
        temp.addVertex(ofVec3f(-radius/2, radius/2,0));
        temp.addVertex(ofVec3f( radius/2, radius/2,0));
        temp.addVertex(ofVec3f( radius/2,-radius/2,0));
        temp.addVertex(ofVec3f(-radius/2,-radius/2,0));
    }
    // make a polygon
    else{
        float theta = 360/res;
        for (int i=0; i<res; i++){
            ofPoint p = ofPoint(0,0,radius);
            temp.addVertex(p.rotate(theta*i, ofVec3f(1,0,0)));
        }
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------
ofMatrix4x4 Path3D::flip(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f z = originalMat.getRowAsVec3f(2);   // local y-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(180, z.x, z.y, z.z);     // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}


//--------------------------------------------------------------
ofMatrix4x4 Path3D::zForward(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f y = originalMat.getRowAsVec3f(1);   // local y-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(90, y.x, y.y, y.z);     // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}


//--------------------------------------------------------------
ofMatrix4x4 Path3D::zOut(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f x = originalMat.getRowAsVec3f(0);   // local x-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(-90, x.x, x.y, x.z);      // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}
