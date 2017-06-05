//
//  CylinderRestrictor.h
//  RobotRecordKinect-Nick
//
//  Created by Nick Hardeman on 11/29/16.
//

#pragma once
#include "ofMain.h"
#include "UR5KinematicModel.h"

namespace ofxRobotArm {
    class CylinderRestrictor {
    public:
        ofParameterGroup &  setup();
        void update( float aDeltaTimef );
        void draw();
        
        bool isEnabled();
        bool isWithinCylinder( UR5KinematicModel* amodel );
        
    protected:
        ofParameterGroup params;
        ofParameter< bool > bEnableCylinderLimit;
        ofParameter< bool > bDrawCylinderLimit;
        ofParameter< float > m_cylinderRadius;
        ofParameter< float > m_cylinderHeight;
        ofParameter< float > m_cylinderZ;
        
        shared_ptr< ofCylinderPrimitive > m_cylinder;
        
    };
}
