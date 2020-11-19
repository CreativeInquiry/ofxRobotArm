//
//  JointRestrictor.h
//  RobotRecordKinect
//
//  Created by Nick Hardeman on 11/21/16.
//

#pragma once
//#include "UR5KinematicModel.h"
//#include "UR10KinematicModel.h"
#include "RobotKinematicModel.h"

namespace ofxRobotArm {
    class JointRestrictor {
    public:
        ofParameterGroup & setup();
        void update( float aDeltaTimef );
//        void drawLimits( UR10KinematicModel* amodel );
//        void drawAngles( UR10KinematicModel* amodel, vector< double > aCurrentAngles );
//        void drawLimits( UR5KinematicModel* amodel );
//        void drawAngles( UR5KinematicModel* amodel, vector< double > aCurrentAngles );
       
        void drawAngles( RobotKinematicModel* amodel, vector< double > aCurrentAngles );
        void drawLimits( RobotKinematicModel* amodel );
        
        void setShoulderAngle(float angle);
        float getMinJointAngle(int aIndex);
        float getMaxJointAngle(int aIndex);
//        vector< ofVec3f > getAxes( UR10KinematicModel* amodel, int aIndex );
//        vector< ofVec3f > getAxes( UR5KinematicModel* amodel, int aIndex );
        vector< ofVec3f > getAxes( RobotKinematicModel* amodel, int aIndex );
        void drawArc( float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis );
        
        float getRestricted( int aIndex, float aInAngleRadians );
        bool canReachTarget( int aIndex, float aCurrentAngleInRadians, float aTargetInRadians, float aEpsilonRadians, bool bUseClosest );
        float getAngleToTargetDifference( int aIndex, float aCurrentAngleInRadians, float aTargetInRadians, bool bUseClosest );
        float getCloserLimit( int aIndex, float aTargetInRadians );
        

        ofParameterGroup params;
        ofParameter< float > mMinGlobalAngle, mMaxGlobalAngle;
        vector< ofParameter< float > > m_angleMinLimits;
        vector< ofParameter< float > > m_angleMaxLimits;
        vector< ofParameter< bool > > m_bApplyLimits;
        vector< bool > m_bInverseAngleDiffs;
        ofParameter< bool > m_bLimitElbow;
    protected:
        float shoulderAngle = 0;
    };
}
