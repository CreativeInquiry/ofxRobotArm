// //
// //  JointRestrictor.h
// //  RobotRecordKinect
// //
// //  Created by Nick Hardeman on 11/21/16.
// //

// #pragma once
// #include "ofMain.h"
// #include "RobotModel.h"

// namespace ofxRobotArm {
//     class JointRestrictor {
//     public:
//         JointRestrictor();
//         ~JointRestrictor();
//         ofParameterGroup & setup(string name);
        
//         void update( float aDeltaTimef );
//         void drawAngles( );
//         void drawLimits(ofNode *joint)
//         void drawArc( float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis );
    
//         float getMinJointAngle();
//         float getMaxJointAngle();
//         float getRestricted(float aInAngleRadians );
//         bool canReachTarget(float aCurrentAngleInRadians, float aTargetInRadians, float aEpsilonRadians, bool bUseClosest );
//         float getAngleToTargetDifference(float aCurrentAngleInRadians, float aTargetInRadians, bool bUseClosest );
//         float getCloserLimit(float aTargetInRadians );
        
//         ofParameterGroup params;

//         ofParameter< float >  m_angleMinLimit;
//         ofParameter< float >  m_angleMaxLimit;
//         ofParameter< bool >   m_bApplyLimit;

//     protected:
//         float shoulderAngle = 0;
//     };
// }
