//
//  RobotArmSafety.h
//  Mimic-Nick
//
//  Created by Nick Hardeman on 12/15/16.
//

#pragma once
#include "JointRestrictor.h"
#include "CylinderRestrictor.h"
#include "RobotArmCollision.h"
#include "RobotController.h"

namespace ofxRobotArm {
    class RobotArmSafety {
    public:
        void setup();
        void setDesiredAngles( vector< double > aangles );
        void setCurrentRobotArmAnlges( vector< double > aRobotArmAngles );
        void update(RobotController& aRobotController );
        void update( float aDeltaTimef );
        void draw();
        void draw( UR5KinematicModel* amodel, ofCamera& acam );
        
        vector< double > getTargetRobotAngles();
        vector< double > getDesiredAngles();
        bool isArmPoseCloseToTargetPose();
        
        void setLerpMult( float aMult );
        
    protected:
        float getLerpAmnt( float aDiffInRadians, float aDeltaTimef );
        ofParameterGroup params;
        ofParameter< float > m_angleLerp;
        ofParameter< float > m_angleEpsilon;
        ofParameter< float > m_maxLerpAngle;
        ofParameter< float > m_minDegreesPerSecToSpeedLerp;
        ofParameter< float > m_maxDegreesPerSecToSpeedLerp;
        
        shared_ptr< ofxRobotArm::JointRestrictor > m_jointRestrictor;
        shared_ptr< ofxRobotArm::CylinderRestrictor > mCylinderRestrictor;
        shared_ptr< ofxRobotArm::RobotArmCollision > mCollision;
        
        bool m_bWithinCylinder = false;
        
        vector< double > mDesiredAngles;
        // angles that have NOT been restricted
        vector< double > mTargetAngles;
        // angles that have been restricted //
        // the angles that should be the target for the robot //
        vector< double > mTargetRobotAngles;
        // actual angles of the robot //
        vector< double > mCurrentRobotArmAngles;
        
        vector< double > accumulatedRadialVelocity;
        vector< double > speedLimitLerpPerJoint;
        
        float speedLimitLerp = 0.0;
        float mDeltaTime = 1.0/60.0;
        float mLerpMult = 1.0f;
        float maxDeg = 0.0;
    };
}
