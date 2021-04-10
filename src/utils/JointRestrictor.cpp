// //
// //  JointRestrictor.cpp
// //  RobotRecordKinect
// //
// //  Created by Nick Hardeman on 11/21/16.
// //

// #include "JointRestrictor.h"
// using namespace ofxRobotArm;
// JointRestrictor::JointRestrictor()
// {
// }
// JointRestrictor::~JointRestrictor()
// {
// }
// //--------------------------------------------------------------
// ofParameterGroup &JointRestrictor::setup(string jointName)
// {
//     params.setName(jointName + "_Restrictions");

//     m_bApplyLimit.set("ApplyLimit_" + jointName, false);
//     params.add(m_bApplyLimit);

//     m_angleMinLimit.set("MinAngle_" + jointName, -45.f, -270.f, 0.0f);
//     params.add(m_angleMinLimit);

//     m_angleMaxLimit.set("MaxAngle_" + jointName, 45.f, -50.f, 270.0f);
//     params.add(m_angleMaxLimit);

//     return params;
// }

// //--------------------------------------------------------------
// float JointRestrictor::getMinJointAngle()
// {
//     return m_angleMinLimit.get();
// }

// //--------------------------------------------------------------
// float JointRestrictor::getMaxJointAngle()
// {
//     return m_angleMaxLimit.get();
// }

// //--------------------------------------------------------------
// void JointRestrictor::drawLimits(ofNode *joint)
// {
//     ofPushMatrix();
//     {
//         ofMultiMatrix(joint->getGlobalTransformMatrix());
//         ofScale(100, 100, 100);
//         drawArc(getMinJointAngle(i), getMaxJointAngle(i), taxes[0], taxes[1]);
//     }
//     ofPopMatrix();
// }
// //--------------------------------------------------------------
// void JointRestrictor::drawAngles(RobotModel *amodel, vector<double> aCurrentAngles)
// {
//     if (amodel == NULL)
//         return;
//     // loop through each node and show the limits //
//     for (int i = 0; i < amodel->pose.size(); i++)
//     {
//         if (i >= aCurrentAngles.size())
//             return;
//         if (!m_bApplyLimits[i])
//             continue;

//         vector<ofVec3f> taxes = getAxes(amodel, i);

//         ofPushMatrix();
//         {
//             ofTranslate(taxes[2]);
//             ofScale(100, 100, 100);
//             //            ofSetColor( 220, 240, 20 );
//             ofVec3f cvec = taxes[0];
//             cvec.rotate(aCurrentAngles[i] * RAD_TO_DEG, taxes[1]);
//             ofDrawLine(ofVec3f(), cvec);
//         }
//         ofPopMatrix();
//     }
// }

// //--------------------------------------------------------------
// vector<ofVec3f> JointRestrictor::getAxes(RobotModel *amodel)
// {
//     vector<ofVec3f> taxes;
//     taxes.push_back(ofVec3f(-1, 0, 0));
//     taxes.push_back(ofVec3f(0, -1, 0));
//     taxes.push_back(ofVec3f()); // position //
//     if (amodel == NULL)
//         return taxes;
//     if (aIndex >= m_bApplyLimits.size())
//         return taxes;
//     if (aIndex >= amodel->nodes.size())
//         return taxes;
//     if (aIndex >= amodel->pose.size())
//         return taxes;

//     ofNode &cnode = amodel->nodes[aIndex];

//     ofVec3f tpos = cnode.getGlobalPosition();

//     ofVec3f tfwd = ofVec3f(0, 1, 0); //cnode.getLookAtDir();
//     ofVec3f tup = ofVec3f(0, 0, 1);  //cnode.getUpDir();
//     //        ofVec3f tside   = cnode.getSideDir();

//     ofVec3f taxis = amodel->pose[aIndex].axis;

//     if (cnode.getParent() != nullptr)
//     {
//         ofNode *pnode = amodel->nodes[aIndex].getParent();
//         ofQuaternion tGParentQ = pnode->getGlobalOrientation();
//         if (aIndex == 0)
//         {
//         }
//         else if (aIndex == 1)
//         {
//             tfwd = tGParentQ * ofVec3f(-1, 0, 0);
//             tup = tGParentQ * ofVec3f(0, -1, 0);
//         }
//         else if (aIndex == 2)
//         {
//             tfwd = tGParentQ * ofVec3f(0, 0, 1);
//             tup = tGParentQ * ofVec3f(0, -1, 0);
//         }
//         else if (aIndex == 3)
//         {
//             tfwd = tGParentQ * ofVec3f(-1, 0, 0);
//             tup = tGParentQ * ofVec3f(0, -1, 0);
//         }
//         else if (aIndex == 4)
//         {
//             tfwd = tGParentQ * ofVec3f(0, -1, 0);
//             tup = tGParentQ * ofVec3f(0, 0, 1);
//         }
//         else if (aIndex == 5)
//         {
//             tfwd = tGParentQ * ofVec3f(1, 0, 0);
//             tup = tGParentQ * ofVec3f(0, 1, 0);
//         }
//     }

//     taxes[0] = tfwd;
//     taxes[1] = tup;
//     taxes[2] = tpos;

//     return taxes;
// }

// //--------------------------------------------------------------
// void JointRestrictor::drawArc(float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis)
// {
//     float startDegrees = aStartAngleDegrees;
//     float endDegrees = aEndAngleDegrees;
//     float tangleDiff = endDegrees - startDegrees;
//     int iterations = fabs(tangleDiff) / 4;
//     if (iterations < 2)
//         iterations = 2;

//     float currDegree = startDegrees;
//     float tstep = tangleDiff / (float)iterations;

//     ofVec3f cvec = aForwardAxis;

//     ofMesh tmesh;
//     tmesh.setMode(OF_PRIMITIVE_LINE_STRIP);

//     tmesh.addVertex(ofVec3f());
//     for (int i = 0; i <= iterations; i++)
//     {
//         cvec = aForwardAxis;
//         cvec.rotate(currDegree, aSideAxis);
//         tmesh.addVertex(cvec);
//         currDegree += tstep;
//         currDegree = ofWrapDegrees(currDegree);
//     }
//     tmesh.addVertex(ofVec3f());

//     tmesh.draw();
// }

// //--------------------------------------------------------------
// float JointRestrictor::getRestricted(float aInAngleRadians)
// {
//     float tinDegree = aInAngleRadians * RAD_TO_DEG;
//     float tminDegree = getMinJointAngle();
//     float tmaxDegree = getMaxJointAngle();

//     return m_bApplyLimit?ofClamp(tinDegree, tminDegree, tmaxDegree) * DEG_TO_RAD:aInAngleRadians;
// }

// //--------------------------------------------------------------
// bool JointRestrictor::canReachTarget(float aCurrentAngleInRadians, float aTargetInRadians, float aEpsilonRadians, bool bUseClosest)
// {
//     // now let's make a step and see if we can make it //
//     float tdiffRadians = ofAngleDifferenceRadians(aCurrentAngleInRadians, aTargetInRadians);
//     // take the long way home
//     if (!bUseClosest)
//     {
//         tdiffRadians = TWO_PI - tdiffRadians;
//     }
//     int tNumIterations = fabs(tdiffRadians) / (aEpsilonRadians / 4.f);
//     if (tNumIterations < 1)
//         tNumIterations = 1;
//     float tstep = tdiffRadians / (float)tNumIterations;

//     if (!bUseClosest)
//     {
//         tstep = -tstep;
//     }

//     float tepsilon = aEpsilonRadians;
//     for (int i = 0; i < tNumIterations; i++)
//     {
//         float cradian = (aCurrentAngleInRadians + (float)i * tstep);
//         float trestricted = getRestricted(cradian);
//         float tdiff = fabs(ofAngleDifferenceRadians(cradian, aTargetInRadians));
//         if (tdiff <= tepsilon * 0.5f)
//         {
//             return true;
//         }
//         if (fabs(cradian - trestricted) > (0.01 * DEG_TO_RAD))
//         {
//             return false;
//         }
//     }
//     return false;
// }

// //--------------------------------------------------------------
// float JointRestrictor::getAngleToTargetDifference(float aCurrentAngleInRadians, float aTargetInRadians, bool bUseClosest)
// {
//     float tangleRadians = aCurrentAngleInRadians;
//     if (!bUseClosest)
//     {
//         tangleRadians = TWO_PI - aCurrentAngleInRadians;
//     }
//     tangleRadians = getRestricted(tangleRadians);
//     float tdiffRadians = ofAngleDifferenceRadians(tangleRadians, aTargetInRadians);
//     return tdiffRadians;
// }

// //--------------------------------------------------------------
// float JointRestrictor::getCloserLimit(float aTargetInRadians)
// {
//     float tMinRadians = getMinJointAngle() * DEG_TO_RAD;
//     float tMaxRadians = getMaxJointAngle() * DEG_TO_RAD;

//     float tMinDiff = fabs(ofAngleDifferenceRadians(aTargetInRadians, tMinRadians));
//     float tMaxDiff = fabs(ofAngleDifferenceRadians(aTargetInRadians, tMaxRadians));
//     if (tMinDiff <= tMaxDiff)
//     {
//         return tMinRadians;
//     }
//     return tMaxRadians;
// }
