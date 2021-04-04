
#include "URDFModel.h"
using namespace ofxRobotArm;
URDFModel::URDFModel(){
    
}
URDFModel::~URDFModel(){

}
void URDFModel::load(string filepath){
    ofxXmlSettings file;
    file.load(filepath);

    int numLinks = file.getNumTags("link");
    int numJoints = file.getNumTags("joints");
    joints.resize(numJoints);
    jointMin.resize(numJoints);
    jointMax.resize(numJoints);
    jointNodes.resize(numJoints);
    poseRadians.resize(numJoints);

    for(int i = 0 ; i < numJoints; i++){
        file.pushTag("joint", i);
        string xyz  = file.getAttribute("origin", "xyz");
        jointMin[i] = file.getAttribute("limit", "lower");
        jointMax[i] = file.getAttribute("limit", "upper");
        string axis = file.getAttribute("axis", "xyz");
        file.popTag();
        Pose p;
        ofNode node = ofNode();
        vector<string> pos = ofSplitString(xyz, ' ');
        p.position = ofVec3f(ofToFloat(pos[0]), ofToFloat(pos[1]), ofToFloat(pos[2]));
        node.setPosition(p.position);
        vector<string> ax = ofSplitString(axis, ' ');
        p.axis = ofVec3f(ofToFloat(ax[0]), ofToFloat(ax[1]), ofToFloat(ax[2]));
        if(i > 0){
            p.offset = p.position - pose[i-1].position;
            node.setParent(jointNodes[i-1]);
        }
        pose[i] = p;
        nodes[i] = node;
   
    }


    for(int i = 0 ; i < numLinks; i++){

    }
}
void URDFModel::draw()
{

}
void URDFModel::drawSKeleton(){
    ofPushStyle();
    {
        int i = 0;
        float dist = 0;
        for (auto joint : nodes) {
            
            
            ofVec3f p = joint.getGlobalPosition();
            
            // draw each link
            ofPushStyle();
            float t = i / float(nodes.size());
            ofColor colorOne = ofColor(ofColor::aqua);
            ofColor colorTwo = ofColor(ofColor::magenta);
            ofSetColor(colorOne.getLerped(colorTwo, t));
            if (i != 0) {
                // draw each joint
                joint.draw();
            }
            ofSetLineWidth(5);
            
            if (i != 0) {
                ofDrawLine(nodes[i - 1].getGlobalPosition(), p);
                dist = p.distance(nodes[i - 1].getGlobalPosition());
            }
            ofPopStyle();
            
            // show length of each link
            ofSetColor(255, 200);
            if (i == 0)
                ofDrawBitmapString(dist, p.getInterpolated(ofVec3f(), .5));
            else
                ofDrawBitmapString(dist, p.getInterpolated(nodes[i - 1].getGlobalPosition(), .5));
            
            // show joint id
            ofSetColor(255,200);
            ofDrawBitmapString(ofToString(i), p.x + 5, p.y, p.z + 5);
            
            // show angle at joint
            ofDrawBitmapString("angle: " + ofToString(pose[i].rotation), p + ofVec3f(0, 0, 20));
            if(ofGetKeyPressed(OF_KEY_CONTROL) && i == 5){
                ofSetColor(colorOne);
                ofDrawBitmapString("pos: " + ofToString(p),  p + ofVec3f(0, 0, 40));
            }
            
            if (i == 5) {
                ofSetColor(colorOne, 100);
                toolNode.draw();
                
                ofSetColor(colorOne, 100);
                tcpNode.draw();
                ofVec3f tcp = tcpNode.getGlobalPosition();
                ofVec3f endJoint = nodes[i].getGlobalPosition();
                p = tcp - endJoint;
                dist = p.length();
                ofSetColor(colorOne, 100);
                ofDrawLine(endJoint, tcp);
                ofDrawBitmapString("TCP Desired Pose", tcp+ ofVec3f(0, 0, 20));
                ofDrawBitmapString("dist: " + ofToString(dist), endJoint+ p.normalize()*dist/2 + ofVec3f(0, 0, -40));
                ofDrawBitmapString("pos:  " +ofToString(tcp), tcp+ ofVec3f(0, 0, 80));
                
                ofVec3f fwp = forwardPose.getGlobalPosition();
                p = fwp - endJoint;
                dist = p.length();
                ofSetColor(colorTwo, 100);
                forwardPose.draw();
                if(fwp.distance(tcp) > 20){
                    ofSetColor(colorTwo, 100);
                    ofDrawLine(endJoint, fwp);
                    ofDrawBitmapString("Forward Pose", fwp + ofVec3f(0, 0, 20));
                    ofDrawBitmapString("dist: " + ofToString(dist), endJoint+p.normalize()*dist/2 + ofVec3f(0, 0, -40));
                    ofDrawBitmapString("pos:  " + ofToString(fwp), fwp + ofVec3f(0, 0, 80));
                }
            }
            i++;
        }
    }
    ofPopStyle();
}
void URDFModel::setPose(vector<double> pose){

}
void URDFModel::setEndEffector(string filename){

}
void URDFModel::setToolMesh(ofMesh mesh){

}