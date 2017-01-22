//
//  JointRestrictor.h
//  RobotRecordKinect
//
//  Created by Nick Hardeman on 11/21/16.
//

#pragma once
#include "ofMain.h"
#include "UR5KinematicModel.h"

namespace ofxRobotArm {
    class JointRestrictor {
    public:
        void setup();
        void update( float aDeltaTimef );
        void drawLimits( UR5KinematicModel* amodel );
        void drawAngles( UR5KinematicModel* amodel, vector< double > aCurrentAngles );
        
        
        vector< ofVec3f > getAxes( UR5KinematicModel* amodel, int aIndex );
        void drawArc( float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis );
        
        float getRestricted( int aIndex, float aInAngleRadians );
        bool canReachTarget( int aIndex, float aCurrentAngleInRadians, float aTargetInRadians, float aEpsilonRadians, bool bUseClosest );
        float getAngleToTargetDifference( int aIndex, float aCurrentAngleInRadians, float aTargetInRadians, bool bUseClosest );
        float getCloserLimit( int aIndex, float aTargetInRadians );
        
    protected:
        ofParameterGroup params;
        ofParameter< float > mMinGlobalAngle, mMaxGlobalAngle;
        vector< ofParameter< float > > m_angleMinLimits;
        vector< ofParameter< float > > m_angleMaxLimits;
        vector< ofParameter< bool > > m_bApplyLimits;
        vector< bool > m_bInverseAngleDiffs;
    };
}
