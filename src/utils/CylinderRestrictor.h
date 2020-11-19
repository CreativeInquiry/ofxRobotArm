//
//  CylinderRestrictor.h
//  RobotRecordKinect-Nick
//
//  Created by Nick Hardeman on 11/29/16.
//

#pragma once
#include "ofMain.h"
//#include "UR5KinematicModel.h"
//#include "UR10KinematicModel.h"
#include "RobotKinematicModel.h"

namespace ofxRobotArm {
    class CylinderRestrictor {
    public:
        ofParameterGroup &  setup();
        void update( float aDeltaTimef );
        void draw();
        
        bool isEnabled();
        bool isWithinCylinder( RobotKinematicModel* amodel );
//        bool isWithinCylinder( UR5KinematicModel* amodel );
//        bool isWithinCylinder( UR10KinematicModel* amodel );

        ofParameterGroup params;
        ofParameter< bool > bEnableCylinderLimit;
        ofParameter< bool > bDrawCylinderLimit;
        ofParameter< float > m_cylinderRadius;
        ofParameter< float > m_cylinderHeight;
        ofParameter< float > m_cylinderZ;
    protected:
        shared_ptr< ofCylinderPrimitive > m_cylinder;
        
    };
}
