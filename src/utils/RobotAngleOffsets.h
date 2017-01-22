//
//  RobotAngleOffsets.h
//  RobotRecordKinect
//
//  Created by Nick Hardeman on 11/2/16.
//

#pragma once
#include "ofMain.h"
namespace ofxRobotArm{
    class RobotAngleOffsets {
    public:
        
        void setup(bool abAddInvAngles, bool abAddScales );
        void update( float aDeltaTimef );
        vector< double > getOffsetPose( vector< double > aInPose );
        
    protected:
        ofParameterGroup params;
        ofParameter< bool > bReset;
        vector< ofParameter< float > > invAddAngles;
        vector< ofParameter< float > > addAngles;
        vector< ofParameter< float > > angleScales;
        
        vector< float > mSmoothInvAddAngles;
        vector< float > mSmoothAddAngles;
        vector< float > mSmoothAngleScales;
    };
}
