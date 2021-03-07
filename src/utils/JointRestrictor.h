//
//  JointRestrictor.h
//  RobotRecordKinect
//
//  Created by Nick Hardeman on 11/21/16.
//

#pragma once
#include "ofMain.h"
#include "RobotModel.h"

namespace ofxRobotArm {
    class JointRestrictor {
    public:
        JointRestrictor();
        ~JointRestrictor();
        ofParameterGroup & setup();
        void setShoulderAngle(float angle);
        
        void update( float aDeltaTimef );
        void drawAngles( RobotModel * amodel, vector< double > aCurrentAngles );
        void drawLimits( RobotModel * amodel );
        void drawArc( float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis );
    
        vector< ofVec3f > getAxes( RobotModel * amodel, int aIndex );
    
        float getMinJointAngle(int aIndex);
        float getMaxJointAngle(int aIndex);
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
