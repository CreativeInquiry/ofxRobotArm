//
//  RobotAngleOffsets.cpp
//  RobotRecordKinect
//
//  Created by Nick Hardeman on 11/2/16.
//

#include "RobotAngleOffsets.h"

//--------------------------------------------------------------
ofParameterGroup &  RobotAngleOffsets::setup(bool abAddInvAngles, bool abAddScales ) {
    params.setName( "RobotArmOffsets" );
    
    params.add( bReset.set("Reset", false ));
    for( int i = 0; i < 6; i++ ) {
        ofParameter< float > tinv;
        tinv.set( "InvAddAngle_"+ofToString(i,0), 0.0f, -180.0f, 180.0f );
        if(abAddInvAngles) {
            invAddAngles.push_back( tinv );
            params.add( invAddAngles.back() );
        }
        
        ofParameter< float > tang;
        tang.set("AddAngle_"+ofToString(i,0), 0.0f, -180.0f, 180.0f );
        addAngles.push_back( tang );
        if( i <= 0 ) params.add( addAngles.back() );
        
        ofParameter< float > tscale;
        tscale.set("AngleScale_"+ofToString(i,0), 1.0f, -10.0f, 10.0f );
        if( abAddScales ) {
            angleScales.push_back( tscale );
            params.add( angleScales.back() );
        }
    }
    
    return params;
}

//--------------------------------------------------------------
void RobotAngleOffsets::update( float aDeltaTimef ) {
    if( mSmoothInvAddAngles.size() != invAddAngles.size() ) {
        mSmoothInvAddAngles.assign( invAddAngles.size(), 0.0 );
    }
    
    if( mSmoothAddAngles.size() != addAngles.size() ) {
        mSmoothAddAngles.assign( addAngles.size(), 0.0 );
    }
    
    if( mSmoothAngleScales.size() != angleScales.size() ) {
        mSmoothAngleScales.assign( angleScales.size(), 1.0 );
    }
    
    if( bReset ) {
        for( int i = 0; i < invAddAngles.size(); i++ ) {
            invAddAngles[i] = 0;
        }
        for( int i = 0; i < addAngles.size(); i++ ) {
            addAngles[i] = 0;
        }
        bReset = false;
    }
    
    float maxLerp   = 0.5 * aDeltaTimef * 60.0;
    float tLerpAmnt = 0.05 * aDeltaTimef * 60.0f;
    
    for( int i = 0; i < mSmoothInvAddAngles.size(); i++ ) {
        float tlerp = (invAddAngles[i]-mSmoothInvAddAngles[i]) * tLerpAmnt;//ofLerp( mSmoothInvAddAngles[i], invAddAngles[i], 0.05 * aDeltaTimef * 60.0f );
        tlerp = ofClamp( tlerp, -maxLerp, maxLerp );
        mSmoothInvAddAngles[i] = mSmoothInvAddAngles[i]+tlerp;//ofLerp( mSmoothInvAddAngles[i], invAddAngles[i], 0.05 * aDeltaTimef * 60.0f );
    }
    for( int i = 0; i < mSmoothAddAngles.size(); i++ ) {
        
        //ofLog(OF_LOG_VERBOSE) << "mSmoothAddAngles["<<i<<"]: " << mSmoothAddAngles[i] << " | " << ofGetFrameNum() << endl;
//        float tlerp = ofLerp( mSmoothAddAngles[i], addAngles[i], 0.05 * aDeltaTimef * 60.0f );
        float tlerp = (addAngles[i]-mSmoothAddAngles[i]) * tLerpAmnt;//ofLerp( mSmoothInvAddAngles[i], invAddAngles[i], 0.05 * aDeltaTimef * 60.0f );
        tlerp = ofClamp( tlerp, -maxLerp, maxLerp );
        mSmoothAddAngles[i] = mSmoothAddAngles[i]+tlerp;//ofLerp( mSmoothAddAngles[i], addAngles[i], 0.05 * aDeltaTimef * 60.0f );
    }
    for( int i = 0; i < mSmoothAngleScales.size(); i++ ) {
//        float tlerp = ofLerp( mSmoothAngleScales[i], angleScales[i], 0.05 * aDeltaTimef * 60.0f );
        float tlerp = (angleScales[i]-mSmoothAngleScales[i]) * tLerpAmnt;
        tlerp = ofClamp( tlerp, -maxLerp, maxLerp );
        mSmoothAngleScales[i] = mSmoothAngleScales[i] + tlerp;//ofLerp( mSmoothAngleScales[i], angleScales[i], 0.05 * aDeltaTimef * 60.0f );
    }
    
}

//--------------------------------------------------------------
vector< double > RobotAngleOffsets::getOffsetPose( vector< double > aInPose ) {
    vector< double > tOutPose;
    tOutPose.resize( aInPose.size(), 0.0f );
    
    for( int i = 0; i < aInPose.size(); i++ ) {
        tOutPose[i] = aInPose[i];
        
        if( i < mSmoothInvAddAngles.size() ) {
            if( mSmoothInvAddAngles[i] != 0.0f ) {
//                tOutPose[i] = ofWrapRadians( ( DEG_TO_RAD * mSmoothInvAddAngles[i]) - tOutPose[i] );
                tOutPose[i] = ( ( DEG_TO_RAD * mSmoothInvAddAngles[i]) - tOutPose[i] );
            }
        }
        if( i < mSmoothAddAngles.size() ) {
            if( mSmoothAddAngles[i] != 0. ) {
//                tOutPose[i] = ofWrapRadians( tOutPose[i] + (mSmoothAddAngles[i] * DEG_TO_RAD) );
                tOutPose[i] = ( tOutPose[i] + (mSmoothAddAngles[i] * DEG_TO_RAD) );
            }
        }
        
        if( i < mSmoothAngleScales.size() ) {
//            tOutPose[i] = ofWrapRadians( tOutPose[i] * mSmoothAngleScales[i] );
            tOutPose[i] = ( tOutPose[i] * mSmoothAngleScales[i] );
        }
    }
    return tOutPose;
}








