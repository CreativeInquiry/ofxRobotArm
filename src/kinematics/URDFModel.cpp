
#include "URDFModel.h"
using namespace ofxRobotArm;
URDFModel::URDFModel()
{

}

URDFModel::~URDFModel()
{

}
void URDFModel::load(string filepath)
{
    ofxXmlSettings xml;
    ofLog(OF_LOG_NOTICE) << filepath << endl;
    if (xml.load(ofToDataPath(filepath)))
    {
        ofLog(OF_LOG_NOTICE) << "loaded" << endl;
        xml.pushTag("robot");
        int numLinks = xml.getNumTags("link");
        int numJoints = xml.getNumTags("joint");
        ofLog(OF_LOG_NOTICE) << numJoints << endl;
        pose.resize(numJoints);
        jointMin.resize(numJoints);
        jointMax.resize(numJoints);
        nodes.resize(numJoints);
        poseRadians.resize(numJoints);

        for (int i = 0; i < numJoints; i++)
        {
            xml.pushTag("joint", i);
            ofLog() << i << endl;
            string xyz = xml.getAttribute("origin", "xyz", "0.0 0.0 0.0", 0);
            string rot = xml.getAttribute("origin", "rpy", "0.0 0.0 0.0", 0);
            cout << xyz << endl;
            jointMin[i] = xml.getAttribute("limit", "lower", -TWO_PI, 0);
            jointMax[i] = xml.getAttribute("limit", "upper", TWO_PI, 0);
            string axis = xml.getAttribute("axis", "xyz", "0.0 0.0 0.0", 0);
            ofLog() << axis << endl;
            xml.popTag();
            Pose p;
            ofNode node = ofNode();
            vector<string> pos = ofSplitString(xyz, " ");
            p.position = ofVec3f(ofToFloat(pos[0]), ofToFloat(pos[1]), ofToFloat(pos[2])) * 1000;
            ofLog() << p.position << endl;
            node.setPosition(p.position);
            vector<string> ax = ofSplitString(axis, " ");
            p.axis = ofVec3f(ofToFloat(ax[0]), ofToFloat(ax[1]), ofToFloat(ax[2]));
            vector<string> ro = ofSplitString(rot, " ");
             ofLog() <<"ro "<<rot<< endl;
            p.rotation = 0;
            p.orientation.makeRotate(ofRadToDeg(ofToFloat(ro[0])), ofVec3f(1, 0, 0),
                                     ofRadToDeg(ofToFloat(ro[1])), ofVec3f(0, 1, 0),
                                     ofRadToDeg(ofToFloat(ro[2])), ofVec3f(0, 0, 1));
            
            if (i > 0)
            {
                p.offset = p.position - pose[i - 1].position;
                ofLog() <<"OFFSET "<< p.offset << endl;
                node.setParent(nodes[i - 1]);
            }
            pose[i] = p;
            nodes[i] = node;
        }

        for (int i = 0; i < numLinks; i++)
        {
            if(xml.pushTag("link", i))
            {
                ofxAssimpModelLoader loader;
                if(xml.pushTag("visual"))
                {
                    if(xml.pushTag("geometry"));
                    {
                        string path = xml.getAttribute("mesh", "filename", "", 0);
                        if (path != "")
                        {
                            ofLog(OF_LOG_NOTICE) << path << endl;
                            if(loader.loadModel(ofToDataPath(path))){
                                ofLog(OF_LOG_NOTICE) << "LOADED" << endl;
                                ofMesh m;
                                for(int i = 0 ; i < loader.getMeshCount(); i++){
                                    m.append(loader.getMesh(i));
                                     ofLog(OF_LOG_NOTICE) << "MESH "<< endl;
                                }
                                 ofLog(OF_LOG_NOTICE) <<m.getNumVertices()<<endl;
                                meshes.push_back(m); 
                            }else{
                                ofLog(OF_LOG_NOTICE) << "NOT LOADED!"<< endl;
                            }
                        }
                        xml.popTag();
                    }
                    xml.popTag(); 
                }
                xml.popTag();
            }
        }
    }
}

void URDFModel::setPose(vector<double> pose){
    poseRadians = pose;
    for(int i = 0; i < pose.size() && i < this->pose.size(); i++){
        this->pose[i].rotation = ofRadToDeg(pose[i]);
        this->pose[i].orientation.makeRotate(this->pose[i].rotation,this->pose[i].axis);
        nodes[i].setOrientation(this->pose[i].orientation);
    }    
}

void URDFModel::draw(ofColor color)
{

    drawMesh(color);
    drawSkeleton();
}


void URDFModel::drawMesh(ofColor color){
    ofPushStyle();
    {
        ofEnableDepthTest();
        int i = 0; 
        for (auto joint : meshes)
        {
            ofSetColor(255, 255, 255);
            ofPushMatrix();
            ofMultMatrix(nodes[i].getGlobalTransformMatrix());
            joint.draw();
            ofPopMatrix();
            i++;
        }
        ofDisableDepthTest();
    }
    ofPopStyle();
}


void URDFModel::drawSkeleton()
{
    ofPushStyle();
    {
        int i = 0;
        float dist = 0;
        for (auto joint : nodes)
        {

            ofVec3f p = joint.getGlobalPosition();

            // draw each link
            ofPushStyle();
            float t = i / float(nodes.size());
            ofColor colorOne = ofColor(ofColor::aqua);
            ofColor colorTwo = ofColor(ofColor::magenta);
            ofSetColor(colorOne.getLerped(colorTwo, t));
            if (i != 0)
            {
                // draw each joint
                joint.draw();
            }
            ofSetLineWidth(5);

            if (i != 0)
            {
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
            ofSetColor(255, 200);
            ofDrawBitmapString(ofToString(i), p.x + 5, p.y, p.z + 5);

            // show angle at joint
            ofDrawBitmapString("angle: " + ofToString(pose[i].rotation), p + ofVec3f(0, 0, 20));
            if (ofGetKeyPressed(OF_KEY_CONTROL) && i == 5)
            {
                ofSetColor(colorOne);
                ofDrawBitmapString("pos: " + ofToString(p), p + ofVec3f(0, 0, 40));
            }

            if (i == 5)
            {
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
                ofDrawBitmapString("TCP Desired Pose", tcp + ofVec3f(0, 0, 20));
                ofDrawBitmapString("dist: " + ofToString(dist), endJoint + p.normalize() * dist / 2 + ofVec3f(0, 0, -40));
                ofDrawBitmapString("pos:  " + ofToString(tcp), tcp + ofVec3f(0, 0, 80));

                ofVec3f fwp = forwardPose.getGlobalPosition();
                p = fwp - endJoint;
                dist = p.length();
                ofSetColor(colorTwo, 100);
                forwardPose.draw();
                if (fwp.distance(tcp) > 20)
                {
                    ofSetColor(colorTwo, 100);
                    ofDrawLine(endJoint, fwp);
                    ofDrawBitmapString("Forward Pose", fwp + ofVec3f(0, 0, 20));
                    ofDrawBitmapString("dist: " + ofToString(dist), endJoint + p.normalize() * dist / 2 + ofVec3f(0, 0, -40));
                    ofDrawBitmapString("pos:  " + ofToString(fwp), fwp + ofVec3f(0, 0, 80));
                }
            }
            i++;
        }
    }
    ofPopStyle();
}

void URDFModel::setEndEffector(string filename)
{
}
void URDFModel::setToolMesh(ofMesh mesh)
{
}
