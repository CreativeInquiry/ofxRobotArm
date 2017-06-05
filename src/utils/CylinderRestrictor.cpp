//
//  CylinderRestrictor.cpp
//  RobotRecordKinect-Nick
//
//  Created by Nick Hardeman on 11/29/16.
//

#include "CylinderRestrictor.h"

using namespace ofxRobotArm;

//--------------------------------------------------------------
 ofParameterGroup &  CylinderRestrictor::setup() {
    params.setName("CylinderRestrictor");
    
    params.add( bEnableCylinderLimit.set( "EnableCylinder", false ));
    params.add( bDrawCylinderLimit.set( "DrawCylinder", false ));
    params.add( m_cylinderRadius.set( "CylinderRadius", 500, 50, 1500 ));
    params.add( m_cylinderHeight.set( "CylinderHeight", 900, 50, 1500 ));
    //    params.add( m_cylinderY.set( "CylinderY", 0, -1000, 1000 ));
    params.add( m_cylinderZ.set( "CylinderZ", 450, -1000, 1000 ));
    return params;
}

//--------------------------------------------------------------
void CylinderRestrictor::update( float aDeltaTimef ) {
    if( !m_cylinder ) {
        m_cylinder = shared_ptr< ofCylinderPrimitive >( new ofCylinderPrimitive() );
        m_cylinder->setResolution( 24, 1, 1 );
        m_cylinder->rotate( 90, ofVec3f(1,0,0) );
        m_cylinder->getMesh().clearColors();
    }
    m_cylinder->setPosition( ofVec3f( 0, 0, m_cylinderZ ));
    if( m_cylinder->getRadius() != m_cylinderRadius ) {
        m_cylinder->setRadius( m_cylinderRadius );
    }
    if( m_cylinder->getHeight() != m_cylinderHeight ) {
        m_cylinder->setHeight( m_cylinderHeight );
    }
}

//--------------------------------------------------------------
void CylinderRestrictor::draw() {
    if( m_cylinder && bDrawCylinderLimit ) {
        m_cylinder->drawWireframe();
        ofDrawSphere( m_cylinder->getPosition(), 30 );
    }
}

//--------------------------------------------------------------
bool CylinderRestrictor::isEnabled() {
    return (bEnableCylinderLimit && m_cylinder);
}

//--------------------------------------------------------------
bool CylinderRestrictor::isWithinCylinder( UR5KinematicModel* amodel ) {
    if( !m_cylinder ) return true;
    ofVec3f cpos = m_cylinder->getPosition();
    // loop through the nodes //
    float cheight = m_cylinder->getHeight();
    float cradius = m_cylinder->getRadius();
    for( int i = 1; i < (int)amodel->nodes.size(); i++ ) {
        // first check for the height in z space //
        ofVec3f npos = amodel->nodes[i].getGlobalPosition();
        if( npos.z > cpos.z + cheight/2 ) return false;
        if( npos.z < cpos.z - cheight/2 ) return false;
        
        ofVec2f npos2d( npos.x, npos.y );
        if( npos2d.length() > cradius ) return false;
    }
    return true;
}





