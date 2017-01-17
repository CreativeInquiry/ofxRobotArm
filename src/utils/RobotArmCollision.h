//
//  RobotArmCollision.h
//  Mimic-Nick
//
//  Created by Nick Hardeman on 12/13/16.
//

#pragma once
#include "UR5KinematicModel.h"

namespace ofxRobotArm {
    class RobotArmCollision {
    public:
        
        class CollisionSphere {
        public:
            float radius = 40.0f;
            ofVec3f localPos, globalPos;
            ofVec3f vel;
            float lengthAlongAppendage = 0.0;
            float pctAlongAppendage = 0.0;
            bool bColliding=false;
            bool bCollidingWarning=false;
            int appendageIndex = 0;
            float radiusWarning=50.0f;
            float closestCollisionDistSq = -1;
            bool bLastSphere=false;
        };
        
        class Appendage {
        public:
            float length = 0.0;
            vector< CollisionSphere > spheres, prevSpheres;
            bool bColliding=false;
            bool bCollidingWarning = false;
            // in radians //
            float angleVel      = 0.0;
            float currentAngle  = 2000.;
            float prevAngle     = 0.0;
            float origAngle     = 0.0f;
            float angleDir      = 1;
            float actualRobotAngle = 0;
        };
        
        void setup();
        void setRobotAngles( vector< double > aAcutalRobotAngles );
        void setDesiredAngles( vector< double > aDesiredRobotAngles );
        void update( float aDeltaTimef );
        void draw();
        
        bool shouldApply() { return bApply; }
        
        vector< double > getDesiredAngles();
        void updateAppendages( shared_ptr< UR5KinematicModel > amodel, vector< Appendage >& aAppendages );
        void updateModel( shared_ptr< UR5KinematicModel > amodel, vector< Appendage >& aAppendages );
        float getClosestCollisionDistanceSq();
        int getNumWarningCollisions();
        
        bool isColliding();
        bool isWarning();
        
        bool isGoingToCollide();
        bool isGoingToWarn();
        
        bool hasMainCollisionWarnings();
        bool isNeckHittingForearm();
        
    protected:
        void solveMainCollisions();
        void solveHeadToForearmCollision();
        
        ofParameterGroup params;
        ofParameter< bool > bApply;
        ofParameter< bool > bDrawWarnings, bDrawStops;
        vector< ofParameter< float > > mPaddings;
        ofParameter< float > mCorrectStepAngle;
        ofParameter< float > mMaxCorrectiveAngle;
        shared_ptr< UR5KinematicModel > mModel, mPredictiveModel;
        vector< Appendage > mAppendages, mPredictiveAppendages;
        vector< double > mDesiredAngles;
        float mResetPct = 0.0f;
        
        ofMesh sphereMesh;
    };
}
