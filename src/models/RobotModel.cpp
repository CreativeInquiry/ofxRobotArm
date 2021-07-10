//
//  KinectModel.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "RobotModel.h"
using namespace ofxRobotArm;
RobotModel::RobotModel()
{
    pose = vector<ofxRobotArm::Pose>();
}
RobotModel::~RobotModel()
{
}
void RobotModel::setForwardPose(ofNode pose)
{
    forwardPose.setGlobalPosition(pose.getGlobalPosition() * 1000);
    forwardPose.setGlobalOrientation(pose.getGlobalOrientation());
}

void RobotModel::setup(string path, RobotType type)
{
    loadURDF(path, type);
}

ofNode RobotModel::getForwardPose(){
    return forwardPose;
}

void RobotModel::loadURDF(string path, RobotType type)
{
    this->type = type;
    ofxXmlSettings xml;
    if (xml.load(ofToDataPath(path)))
    {
        ofLog(OF_LOG_NOTICE) << "Loading URDF:" << ofToDataPath(path) << endl;
        if (!xml.pushTag("robot"))
        {
            ofLogFatalError() << "CANNOT FIND ROBOT TAG" << endl;
        }
        
        int numLinks = xml.getNumTags("link");
        if (numLinks <= 0)
        {
            ofLogFatalError() << "CANNOT FIND ROBOT LINKS" << endl;
        }
        ofLog(OF_LOG_NOTICE) << "URDF::NUM LINKS " << numLinks << endl;
        
        int numJoints = xml.getNumTags("joint");
        if (numJoints <= 0)
        {
            ofLogFatalError() << "CANNOT FIND ROBOT JOINTS" << endl;
        }
        ofLog(OF_LOG_NOTICE) << "URDF::NUM JOINTS " << numJoints << endl;
        
        pose.resize(numJoints);
        jointMin.resize(numJoints);
        jointMax.resize(numJoints);
        nodes.resize(numJoints);
        poseRadians.resize(numJoints);

        
        
        for (int i = 0; i < numJoints; i++)
        {
            string type  = xml.getAttribute("joint", "type", "revolute");
            string name  = xml.getAttribute("joint", "name", "");
            
            if(xml.pushTag("joint", i)){
                string xyz = xml.getAttribute("origin", "xyz", "0.0 0.0 0.0", 0);
                string rot = xml.getAttribute("origin", "rpy", "0.0 0.0 0.0", 0);
                jointMin[i] = xml.getAttribute("limit", "lower", -TWO_PI, 0);
                jointMax[i] = xml.getAttribute("limit", "upper", TWO_PI, 0);
                string axis = xml.getAttribute("axis", "xyz", "0.0 0.0 0.0", 0);
                
                Pose p;
                p.name = name;
                p.type = type;
                vector<string> pos = ofSplitString(xyz, " ");
                p.position = ofVec3f(ofToFloat(pos[0]), ofToFloat(pos[1]), ofToFloat(pos[2]));
                vector<string> ax = ofSplitString(axis, " ");
                p.axis = ofVec3f(ofToFloat(ax[0]), ofToFloat(ax[1]), ofToFloat(ax[2]));
                vector<string> ro = ofSplitString(rot, " ");
                p.rotOffset = ofVec3f(ofToFloat(ro[0]), ofToFloat(ro[1]), ofToFloat(ro[2]));
                p.rotation = 0;
                p.orientation.makeRotate(ofRadToDeg(ofToFloat(ro[0])), ofVec3f(1, 0, 0),
                                         ofRadToDeg(ofToFloat(ro[1])), ofVec3f(0, 1, 0),
                                         ofRadToDeg(ofToFloat(ro[2])), ofVec3f(0, 0, 1));
                pose[i] = p;
                xml.popTag();
                
                if(i > 0){
                    pose[i].offset = pose[i].position - pose[i-1].position;
                    nodes[i].setParent(nodes[i-1]);
                }
                nodes[i].setPosition(pose[i].position * 1000);
                nodes[i].setOrientation(pose[i].orientation);
            }
        }
        toolNode.setParent(nodes[5]);        
        originNode.setPosition(ofVec3f(0, 0, 0));

        nodes[0].setParent(originNode);


        for (int i = 0; i < numLinks; i++)
        {
            if (xml.pushTag("link", i))
            {
                
                if (xml.pushTag("visual"))
                {
                    if (xml.pushTag("geometry"))
                        ;
                    {
                        string path = ofToDataPath(xml.getAttribute("mesh", "filename", "", 0), true);
                        if (path != "")
                        {
                            ofLog(OF_LOG_NOTICE) << "LOADING " << path << endl;
                            ofxAssimpModelLoader loader;
                            if (loader.loadModel(path, true))
                            {
                                ofLog(OF_LOG_NOTICE) << "LOADED 3D FILE" << endl;
                                ofMesh m;
                                for (int i = 0; i < loader.getMeshCount(); i++)
                                {
                                    m.append(loader.getMesh(i));
                                }
                                ofLog(OF_LOG_NOTICE) << m.getNumVertices() << endl;
                                meshes.push_back(m);
                            }
                            else
                            {
                                ofLog(OF_LOG_NOTICE) << "NOT LOADED!" << endl;
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


void RobotModel::setOrigin(ofVec3f pos, ofQuaternion orientation)
{
    originNode.setGlobalPosition(pos);
    originNode.setGlobalOrientation(orientation);
}

// // D-H Parameters for UR Robot Arms:
// // https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics/
// void RobotModel::setup(RobotType type)
// {
//     this->type = type;
    
//     if (this->type == RobotType::UR3 || this->type == RobotType::UR5 || this->type == RobotType::UR10)
//     {
//         pose.resize(6);
//         nodes.resize(6);
//         if (this->type == RobotType::UR3)
//         {
//             ofLog(OF_LOG_NOTICE) << "LOADING ROBOTYPE UR3" << endl;
//             if (loader.loadModel(ofToDataPath("models/ur3.dae")))
//             {
//                 for (int i = 0; i < loader.getNumMeshes(); i++)
//                 {
//                     meshes.push_back(loader.getMesh(i));
//                 }
//             }
//             else
//             {
//                 ofLogFatalError() << "PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/ur3.dae" << endl;
//             }
            
//             pose[0].position.set(0, 0, 0);
//             pose[1].position.set(0, 0, 0);
//             pose[2].position.set(0, 0, 0);
//             pose[3].position.set(0, 0, 0);
//             pose[4].position.set(0, 0, 0);
//             pose[5].position.set(0, 0, 0);
//         }
//         else if (this->type == RobotType::UR5)
//         {
//             ofLog(OF_LOG_NOTICE) << "LOADING ROBOTYPE UR5" << endl;
//             if (loader.loadModel(ofToDataPath("models/ur5.dae")))
//             {
//                 for (int i = 0; i < loader.getNumMeshes(); i++)
//                 {
//                     meshes.push_back(loader.getMesh(i));
//                 }
//             }
//             else
//             {
//                 ofLogFatalError() << "PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/ur5.dae" << endl;
//             }
            
//             pose[0].position.set(0, 0, 0);
//             pose[1].position.set(0, -0.072238, 0.083204);
//             pose[2].position.set(0, -0.077537, 0.51141);
//             pose[3].position.set(0, -0.070608, 0.903192);
//             pose[4].position.set(0, -0.117242, 0.950973);
//             pose[5].position.set(0, -0.164751, 0.996802);
//         }
//         else if (this->type == RobotType::UR10)
//         {
//             ofLog(OF_LOG_NOTICE) << "LOADING ROBOTYPE UR10" << endl;
//             // should load model from addon data path, not app
//             if (loader.loadModel(ofToDataPath("models/ur10.dae")))
//             {
//                 for (int i = 0; i < loader.getNumMeshes(); i++)
//                 {
//                     meshes.push_back(loader.getMesh(i));
//                 }
//             }
//             else
//             {
//                 ofLogFatalError() << "PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/ur10.dae" << endl;
//             }
            
//             pose[0].position.set(0, 0, 0);
            
//             // These are correct joint positions for the non-home D-H position.
//             // Reference: https://asd.sutd.edu.sg/dfab/a-geometric-inverse-kinematics-solution-for-the-universal-robot/
//             // These are correct joint positions for the HOME position (0 rotations on each joint).
//             pose[1].position.set(0, -.086, .1273);
//             pose[2].position.set(0, -.1163, .7393);
//             pose[3].position.set(0, -.1094, 1.3116);
//             pose[4].position.set(0, -0.16395, 1.3733);
//             pose[5].position.set(0, -0.22535, 1.4273);
//         }
        
//         // Setup the joint axes
//         pose[0].axis.set(0, 0, 1);
//         pose[1].axis.set(0, -1, 0);
//         pose[2].axis.set(0, -1, 0);
//         pose[3].axis.set(0, -1, 0);
//         pose[4].axis.set(0, 0, 1);
//         pose[5].axis.set(0, 1, 0);
        
//         pose[0].orientation.makeRotate(0, pose[0].axis);
//         pose[1].orientation.makeRotate(0, pose[1].axis);
//         pose[2].orientation.makeRotate(0, pose[2].axis);
//         pose[3].orientation.makeRotate(0, pose[3].axis);
//         pose[4].orientation.makeRotate(0, pose[4].axis);
//         pose[5].orientation.makeRotate(0, pose[5].axis);
        
//         pose[0].rotation = 0;
//         pose[1].rotation = 0;
//         pose[2].rotation = 0;
//         pose[3].rotation = 0;
//         pose[4].rotation = 0;
//         pose[5].rotation = 0;
        
//         tool.position.set(pose[5].position + ofVec3f(0.0, -0.0308, 0.0)); // flange position
        
//         for (int i = 1; i < pose.size(); i++)
//         {
//             pose[i].offset = pose[i].position - pose[i - 1].position;
//             ofLog(OF_LOG_NOTICE) << "JOINT: " << i << " OFFSETS XYZ " << pose[i].offset << endl;
//         }
//         tool.offset = pose[5].offset;
        
//         tool.position.set(pose[5].position);
//         tool.orientation = pose[5].orientation;
//         tool.rotation = pose[5].rotation;
//         tool.axis.set(pose[5].axis);
//         // Rig Joints
//         nodes[0].setPosition(pose[0].position);
//         nodes[0].setOrientation(pose[0].orientation);
//         for (int i = 1; i < nodes.size(); i++)
//         {
//             nodes[i].setParent(nodes[i - 1]);
//             nodes[i].setPosition(pose[i].offset * 1000);
//             nodes[i].setOrientation(pose[i].orientation);
//         }
//         // Set Tool Rotations
//         toolNode.setParent(nodes[5]);
//         toolNode.setPosition(tool.position);
//         toolNode.setOrientation(nodes[5].getGlobalOrientation());
//     }
//     else if (this->type == RobotType::IRB120)
//     {
//         pose.resize(6);
//         nodes.resize(6);
//         ofLog(OF_LOG_NOTICE) << "LOADING ROBOTYPE IRB120" << endl;
//         if (loader.loadModel(ofToDataPath("models/irb120.dae")))
//         {
//             for (int i = 0; i < loader.getNumMeshes(); i++)
//             {
//                 meshes.push_back(loader.getMesh(i));
//             }
//         }
//         else
//         {
//             ofLogFatalError() << "PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/irb120.dae" << endl;
//         }
//         // Reference: https://library.e.abb.com/public/7139d7f4f2cb4d0da9b7fac6541e91d1/3HAC035960%20PS%20IRB%20120-en.pdf
//         // ROS https://github.com/ros-industrial/abb_experimental/blob/kinetic-devel/abb_irb120_support/urdf/irb120_3_58_macro.xacro
        
//         pose[0].position.set(0, 0, 0);
//         pose[1].position.set(0, 0, 0.270);
//         pose[2].position.set(0, 0, 0.560);
//         pose[3].position.set(0.134, 0, 0.630);
//         pose[4].position.set(0.302, 0, 0.630);
//         pose[5].position.set(0.374, 0, 0.630);
        
//         pose[0].axis.set(0, 0, 1);
//         pose[1].axis.set(0, 1, 0);
//         pose[2].axis.set(0, 1, 0);
//         pose[3].axis.set(1, 0, 0);
//         pose[4].axis.set(0, 1, 0);
//         pose[5].axis.set(1, 0, 0);
        
//         pose[0].orientation.makeRotate(0, pose[0].axis);
//         pose[1].orientation.makeRotate(0, pose[1].axis);
//         pose[2].orientation.makeRotate(0, pose[2].axis);
//         pose[3].orientation.makeRotate(0, pose[3].axis);
//         pose[4].orientation.makeRotate(0, pose[4].axis);
//         pose[5].orientation.makeRotate(0, pose[5].axis);
        
//         pose[0].rotation = 0;
//         pose[1].rotation = 0;
//         pose[2].rotation = 0;
//         pose[3].rotation = 0;
//         pose[4].rotation = 0;
//         pose[5].rotation = 0;
        
//         tool.position.set(pose[5].position + ofVec3f(0.0, -0.0308, 0.0)); // flange position
        
//         for (int i = 1; i < pose.size(); i++)
//         {
//             pose[i].offset = pose[i].position - pose[i - 1].position;
//             ofLog(OF_LOG_NOTICE) << "JOINT: " << i << " OFFSETS XYZ " << pose[i].offset << endl;
//         }
//         tool.offset = pose[5].offset;
        
//         tool.position.set(pose[5].position);
//         tool.orientation = pose[5].orientation;
//         tool.rotation = pose[5].rotation;
//         tool.axis.set(pose[5].axis);
//         // Rig Joints
//         nodes[0].setPosition(pose[0].position);
//         nodes[0].setOrientation(pose[0].orientation);
//         for (int i = 1; i < nodes.size(); i++)
//         {
//             nodes[i].setParent(nodes[i - 1]);
//             nodes[i].setPosition(pose[i].offset * 1000);
//             nodes[i].setOrientation(pose[i].orientation);
//         }
//         // Set Tool Rotations
//         toolNode.setParent(nodes[5]);
//         toolNode.setPosition(tool.position);
//         toolNode.setOrientation(nodes[5].getGlobalOrientation());
//     }
// }

ofQuaternion RobotModel::getToolPointQuaternion()
{
    return toolNode.getGlobalOrientation();
}

ofNode RobotModel::getTool()
{
    return toolNode;
}

void RobotModel::setToolOffset(ofVec3f localOffset)
{
    this->localOffset = localOffset;
    toolNode.setPosition(this->localOffset);
}

void RobotModel::setTCPPose(Pose pose)
{
    tool = pose;
    ofMatrix4x4 mat;
    mat.makeRotationMatrix(toolNode.getGlobalOrientation());
    ofVec3f pos = toolNode.getPosition() / 1000.0;
    tool.position = tool.position - pos * mat;
    tcpNode.setPosition(tool.position * 1000);
    tcpNode.setOrientation(pose.orientation);
}

Pose RobotModel::getModifiedTCPPose()
{
    return tool;
}

void RobotModel::setPose(vector<double> pose)
{
    
    int i = 0;
    for (auto pdouble : pose)
    {
        this->pose[i].rotation = ofRadToDeg(pdouble+this->pose[i].rotOffset.length());
        poseRadians[i] = pdouble+this->pose[i].rotOffset.length();
        this->pose[i].orientation.makeRotate(this->pose[i].rotation, this->pose[i].axis);
        nodes[i].setOrientation(this->pose[i].orientation);
        i++;
    }
}

void RobotModel::setEndEffector(string filename)
{
    string path = ofToDataPath("models/" + filename);
    
    loader.clear();
    if (loader.loadModel("models/" + filename))
    {
        toolMesh = loader.getMesh(0);
    }
    else
    {
        ofLogFatalError() << "PLEASE PLACE THE 3D FILES OF THE END EFFECTOR IN data/models/" << filename << endl;
    }
}

void RobotModel::clearEndEffector()
{
    toolMesh.clear();
}

void RobotModel::setToolMesh(ofMesh mesh)
{
    toolMesh = mesh;
}

// ----------------------------------------------------------
void RobotModel::drawSkeleton()
{
    ofPushStyle();
    {
        int i = 0;
        float dist = 0;
        for (auto joint : nodes)
        {
            
            ofVec3f p = joint.getGlobalPosition();
            ofColor colorOne = ofColor(ofColor::aqua);
            ofColor colorTwo = ofColor(ofColor::magenta);
            // draw each link
            ofPushStyle();
            {
                float t = i / float(nodes.size());
                
                if (pose[i].axis != ofVec3f(0, 1, 0))
                {
                    ofPushMatrix();
                    {
                        ofMultMatrix(joint.getGlobalTransformMatrix());
                        ofMatrix4x4 mat;
                        mat.makeRotationMatrix(-pose[i].rotation, pose[i].axis);
                        ofMultMatrix(mat);
                        ofScale(70, 70, 70);
                        ofSetLineWidth(3);
                        ofVec3f forward = ofVec3f(0, 1, 0).getPerpendicular(pose[i].axis);
                        ofSetColor(colorOne.getLerped(colorTwo, t));
                        drawArc(jointMin[i] * RAD_TO_DEG, jointMax[i] * RAD_TO_DEG, forward, pose[i].axis);
                        forward.rotate(pose[i].rotation, pose[i].axis);
                        ofDrawLine(ofVec3f(), forward);
                        ofSetColor(colorOne.getLerped(colorTwo, t), 100);
                        drawArc(jointMin[i] * RAD_TO_DEG, pose[i].rotation, ofVec3f(0, 1, 0).getPerpendicular(pose[i].axis), pose[i].axis, true);
                    }
                    ofPopMatrix();
                }
                else if (pose[i].axis != ofVec3f(0, 0, 1))
                {
                    
                    ofPushMatrix();
                    {
                        ofMultMatrix(joint.getGlobalTransformMatrix());
                        ofMatrix4x4 mat;
                        mat.makeRotationMatrix(-pose[i].rotation, pose[i].axis);
                        ofMultMatrix(mat);
                        ofScale(70, 70, 70);
                        ofSetLineWidth(3);
                        ofSetColor(colorOne.getLerped(colorTwo, t));
                        ofVec3f forward = ofVec3f(0, 0, 1).getPerpendicular(pose[i].axis);
                        drawArc(jointMin[i] * RAD_TO_DEG, jointMax[i] * RAD_TO_DEG, forward, pose[i].axis);
                        forward.rotate(pose[i].rotation, pose[i].axis);
                        ofDrawLine(ofVec3f(), forward);
                        ofSetColor(colorOne.getLerped(colorTwo, t), 100);
                        drawArc(jointMin[i] * RAD_TO_DEG, pose[i].rotation, ofVec3f(0, 0, 1).getPerpendicular(pose[i].axis), pose[i].axis, true);
                    }
                    ofPopMatrix();
                }
                
                if (i > 0)
                {
                    ofSetColor(colorOne.getLerped(colorTwo, t));
                    // draw each joint
                    joint.draw();
                    
                    ofSetLineWidth(5);
                    ofDrawLine(nodes[i - 1].getGlobalPosition(), p);
                    dist = p.distance(nodes[i - 1].getGlobalPosition());
                }
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
                ofSetColor(255, 0, 255, 100);
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

void RobotModel::drawMesh(ofColor color, bool bDrawDebug)
{
    ofEnableDepthTest();
    {
        ofPushStyle();
        {
            ofColor face = ofColor(color);
            ofColor wireframe = ofColor(ofColor::black);
            
            int i = 0;
            for (auto mesh : meshes)
            {
                if (i == 0)
                {
                    ofPushMatrix();
                    {
                        ofTranslate(nodes[i].getPosition());
                        ofScale(1000, 1000, 1000);
                        ofSetColor(face);
                        mesh.drawFaces();
                        ofSetColor(wireframe, 100);
                        mesh.drawWireframe();
                    }
                    ofPopMatrix();
                }
                else
                {
                    ofPushMatrix();
                    {
                        ofMultMatrix(nodes[i - 1].getGlobalTransformMatrix());
                        ofScale(1000, 1000, 1000);
                        ofSetColor(face);
                        mesh.drawFaces();
                        ofSetColor(wireframe, 100);
                        mesh.drawWireframe();
                    }
                    ofPopMatrix();
                }
                
                i++;
            }
        }
        ofPopStyle();
    }
    ofDisableDepthTest();
}

void RobotModel::draw(ofColor color, bool bDrawDebug)
{
    ofPushMatrix();
    {
        ofPushStyle();
        {
            
            ofSetColor(255, 255, 255);
            if (bDrawDebug)
            {
                ofPushStyle();
                {
                    ofDrawAxis(1000);
                    ofSetColor(255, 255, 0);
                    ofDrawSphere(tool.position * ofVec3f(1000, 1000, 1000), 40);
                    ofSetColor(225, 225, 225);
                }
                ofPopStyle();
            }
        }
        ofPopStyle();
    }
    ofPopMatrix();
}

void RobotModel::drawArc(float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis, bool fill)
{
    float startDegrees = aStartAngleDegrees;
    float endDegrees = aEndAngleDegrees;
    float tangleDiff = endDegrees - startDegrees;
    int iterations = fabs(tangleDiff) / 4;
    if (iterations < 2)
        iterations = 2;
    
    float currDegree = startDegrees;
    float tstep = tangleDiff / (float)iterations;
    
    ofVec3f cvec = aForwardAxis;
    
    ofMesh tmesh;
    if (fill)
    {
        tmesh.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
    }
    else
    {
        tmesh.setMode(OF_PRIMITIVE_LINE_STRIP);
    }
    
    tmesh.addVertex(ofVec3f());
    for (int i = 0; i <= iterations; i++)
    {
        cvec = aForwardAxis;
        cvec.rotate(currDegree, aSideAxis);
        tmesh.addVertex(cvec);
        currDegree += tstep;
        currDegree = ofWrapDegrees(currDegree);
    }
    tmesh.addVertex(ofVec3f());
    
    tmesh.draw();
}
