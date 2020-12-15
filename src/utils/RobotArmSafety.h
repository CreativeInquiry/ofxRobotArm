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
//#include "UR5KinematicModel.h"
//#include "UR10KinematicModel.h"
#include "RobotKinematicModel.h"
#include "URDriver.h"
//#include "RobotController.h"

namespace ofxRobotArm {
    class RobotArmSafety {
    public:
        ofParameterGroup & setup();
        ofParameterGroup & setup(RobotType type);
        void setDesiredAngles( vector< double > aangles );
        void setCurrentRobotArmAnlges( vector< double > aRobotArmAngles );
//        void update(RobotController& aRobotController );
        void update( float aDeltaTimef );
//        void update( UR10KinematicModel& previewArm );
//        void update( UR5KinematicModel& previewArm );
        void update( RobotKinematicModel& previewArm );
        void draw();
//        void draw( UR5KinematicModel* amodel, ofCamera& acam );
//        void draw( UR10KinematicModel* amodel, ofCamera& acam );
        void draw( RobotKinematicModel* amodel, ofCamera& acam );
        
        void checkCollision(vector<double> actual, vector<double> target);
        
        vector< double > getTargetRobotAngles();
        vector< double > getDesiredAngles();
        bool isArmPoseCloseToTargetPose();
        
        void setLerpMult( float aMult );
        
        shared_ptr< JointRestrictor > m_jointRestrictor;
        shared_ptr< CylinderRestrictor > mCylinderRestrictor;
        shared_ptr< RobotArmCollision > mCollision;
  
        float getLerpAmnt( float aDiffInRadians, float aDeltaTimef );
        ofParameterGroup params;
        ofParameter< float > m_angleLerp;
        ofParameter< float > m_angleEpsilon;
        ofParameter< float > m_maxLerpAngle;
        ofParameter< float > m_minDegreesPerSecToSpeedLerp;
        ofParameter< float > m_maxDegreesPerSecToSpeedLerp;
        
    protected:
        
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
