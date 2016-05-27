//
//  GMLPath.cpp
//  Lemur
//
//  Created by Dan Moore on 7/7/13.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "GMLPath.h"

void GMLPath::setup(){
    setup(0, 0, 1, 1);
}
void GMLPath::addPoint(ofVec3f pt){
    
}
void GMLPath::addPath(vector<ofVec3f> pts){
    
}
void GMLPath::addPath(ofPolyline line){
    
}
void GMLPath::addPaths(vector<ofPolyline> lines){
    
}

void GMLPath::setup(float x, float y, float width, float height){
    reader.setup(x, y, width, height);
//    reader.useTime(false);
}

void GMLPath::loadFile(string path){
    reader.loadFile(path);
    if(reader.tags.size() > 0){
        for(int i = 0; i < reader.tags.back().strokes.size(); i++){
            polys.push_back(reader.tags.back().strokes[i]);
        }
        reader.centerAndNormalize(polys);
    }
}

void GMLPath::draw(){
    ofPushMatrix();
    ofTranslate(ofGetWidth()-200, ofGetHeight()-200);
    ofScale(200, 200);
    for(int i = 0; i < scaledLines.size(); i++){
        scaledLines[i].draw();
    }
    ofPopMatrix();
}

vector<ofPolyline> GMLPath::getPath(float scale){
    vector<ofPolyline> lines = polys;
    reader.scale(lines, scale, scale);
    return lines;
}
