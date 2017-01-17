//
//  RobotArmSafety.cpp
//  Mimic-Nick
//
//  Created by Nick Hardeman on 12/15/16.
//

#include "RobotArmSafety.h"
using namespace ofxRobotArm;

//--------------------------------------------------------------
void RobotArmSafety::setup() {
    mTargetRobotAngles.assign( 6, 0.0 );
    
    params.setName( "RobotSafety" );
    params.add( m_angleLerp.set("AngleLerp", 0.76, 0.0001, 1.0f ));
    params.add( m_maxLerpAngle.set("MaxAngleLerp", 18.3, 0.1, 30.f ));
    params.add( m_angleEpsilon.set("AngleEpsilon", 1.f, 0.0, 3.0 ));
    params.add( m_minDegreesPerSecToSpeedLerp.set("MinDegPerSecToLerpDown", 70, 10, 300));
    params.add( m_maxDegreesPerSecToSpeedLerp.set("MaxDegPerSecToLerpDown", 150, 90, 1000));
    
//    m_minDegreesPerSecToSpeedLerp = 450;
//    m_maxDegreesPerSecToSpeedLerp = 1000;
    

    m_jointRestrictor = shared_ptr< JointRestrictor >( new JointRestrictor() );
    m_jointRestrictor->setup();
    
    mCylinderRestrictor = shared_ptr< CylinderRestrictor >( new CylinderRestrictor() );
    mCylinderRestrictor->setup();
    
    mCollision = shared_ptr< RobotArmCollision >( new RobotArmCollision() );
    mCollision->setup();
}

//--------------------------------------------------------------
void RobotArmSafety::setDesiredAngles( vector< double > aangles ) {
    mDesiredAngles = aangles;
}

//--------------------------------------------------------------
void RobotArmSafety::setCurrentRobotArmAnlges( vector< double > aRobotArmAngles ) {
    mCurrentRobotArmAngles = aRobotArmAngles;
    if( mCollision ) mCollision->setRobotAngles( aRobotArmAngles );
}

//--------------------------------------------------------------
void RobotArmSafety::update( RobotController& aRobotController ) {
    m_bWithinCylinder = false;
    if( mCylinderRestrictor ) {
        if( mCylinderRestrictor->isEnabled() ) {
            m_bWithinCylinder = mCylinderRestrictor->isWithinCylinder( &aRobotController.previewArm );
        } else {
            m_bWithinCylinder = true;
        }
    }
}

//--------------------------------------------------------------
void RobotArmSafety::update( float aDeltaTimef ) {
    mDeltaTime = aDeltaTimef;
    // now set the desired angles from the robot man for the collider
    // this will check against the actual robot position and alter the desired angles if it
    // detects a potential collision //
    // this will use the desired angles inside the robot man //
    if( mCollision->shouldApply() ) {
        mCollision->setDesiredAngles( mDesiredAngles );
        mCollision->update( aDeltaTimef );
        mDesiredAngles = mCollision->getDesiredAngles();
    }
    
    m_jointRestrictor->update( aDeltaTimef );
    mCylinderRestrictor->update( aDeltaTimef );
    
    float tEpsilon  = (m_angleEpsilon * DEG_TO_RAD) * aDeltaTimef * 60.f;
    bool bWeAllGood = true;
    // we desire nothing, do not move forward //
    if( mDesiredAngles.size() == 0 ) bWeAllGood = false;
    if( mCurrentRobotArmAngles.size() == 0 ) bWeAllGood = false;
    
    bool bOKeyPressed = ofGetKeyPressed('o');
    
    if( m_bWithinCylinder && bWeAllGood ) {
        
        if( mTargetAngles.size() != mDesiredAngles.size() ) mTargetAngles.assign( mDesiredAngles.size(), 0.0f );
        if( mTargetRobotAngles.size() != mDesiredAngles.size() ) mTargetRobotAngles.assign( mDesiredAngles.size(), 0.0 );
        if( accumulatedRadialVelocity.size() != mDesiredAngles.size() ) accumulatedRadialVelocity.assign( mDesiredAngles.size(), 0.0 );
        if( speedLimitLerpPerJoint.size() != mDesiredAngles.size() ) speedLimitLerpPerJoint.assign( mDesiredAngles.size(), 0.0 );
        
        for( int i = 0; i < mDesiredAngles.size(); i++ ) {
//            cout << "RobotArmSafety " << i << " updating angle: " << mDesiredAngles[i] << " | " << ofGetFrameNum() << endl;
            // now we need to determine the target angle including the mix pct //
            float targetAngle = mDesiredAngles[i];
//            float deltaAngle = targetAngle - mCurrentRobotArmAngles[i];//ofWrapRadians( targetAngle - mCurrentRobotArmAngles[i], -TWO_PI, TWO_PI );//ofAngleDifferenceRadians( mCurrentRobotArmAngles[i], targetAngle );
            float deltaAngle = ofAngleDifferenceRadians( mCurrentRobotArmAngles[i], targetAngle );
            
            float lerpAngle = getLerpAmnt( deltaAngle, aDeltaTimef );
            
            //            m_currentPoseAngles[i] += lerpAngle;
            float tdesiredAngle = mCurrentRobotArmAngles[i] + lerpAngle;
            // this would be the restricted angle //
            //            m_currentPoseAngles[i] = m_jointRestrictor->getRestricted( i, m_currentPoseAngles[i] );
            float trestrictedAngle = m_jointRestrictor->getRestricted( i, tdesiredAngle );
            
            // can the joint reach the target using the closest angle? //
            bool bItsPossibleToReachDesiredAngle = m_jointRestrictor->canReachTarget( i, mCurrentRobotArmAngles[i], targetAngle, tEpsilon, true );
            //            tdesiredAngle = trestrictedAngle;
            
            if( i == 2 ) {
//                cout << "poss to reach desired angle: " << bItsPossibleToReachDesiredAngle << " | " << ofGetFrameNum() << endl;
            }
            
            if( !bItsPossibleToReachDesiredAngle ) {
                //                tdesiredAngle = trestrictedAngle;
                // figure out if we can go the other way to achieve the target angle //
                bItsPossibleToReachDesiredAngle = m_jointRestrictor->canReachTarget( i, mCurrentRobotArmAngles[i], targetAngle, tEpsilon, false );
                
                //                cout << i << " - poss to reach further angle: " << bItsPossibleToReachDesiredAngle << " isMixing: " << isMixing() << " | " << ofGetFrameNum() << endl;
                
                float tFurtherDiff = m_jointRestrictor->getAngleToTargetDifference(i, mCurrentRobotArmAngles[i], targetAngle, false );
                float tFurtherLerp = getLerpAmnt( tFurtherDiff, aDeltaTimef );
                
                if( bItsPossibleToReachDesiredAngle ) {
                    tdesiredAngle = mCurrentRobotArmAngles[i] - lerpAngle;
                    //cout << i << " - possible to reach the further angle, trying to lerp there m_currentPoseAngles: " << mCurrentRobotArmAngles[i] << " target: " << targetAngle << " lerp: " << tFurtherLerp << " tFurtherDiff: " << tFurtherDiff << " | " << ofGetFrameNum() << endl;
                    //                    tdesiredAngle = m_currentPoseAngles[i] + tFurtherLerp;
                    tdesiredAngle = m_jointRestrictor->getRestricted( i, tdesiredAngle );
                    //                    tdesiredAngle = trestrictedAngle;
                } else {
                    // we can't reach the target angle the long way around either, so leave it to the restricted angle //
                    // so let's figure out which way is closest //
                    float tTargetWithLimit = m_jointRestrictor->getCloserLimit( i, mDesiredAngles[i] );
                    //                    float tClosest  = fabs(m_jointRestrictor->getAngleToTargetDifference( i, targetPose[i], tTargetWithLimit, true ));
                    //                    float tOther    = fabs(m_jointRestrictor->getAngleToTargetDifference( i, targetPose[i], tTargetWithLimit, false ));
                    //
                    //                    cout << i << " - tTargetWithLimit: " << (tTargetWithLimit*RAD_TO_DEG) <<" - (tOther < tClosest): " << (tOther < tClosest) << " tClosest: " << tClosest << " tOther: " << tOther << " | " << ofGetFrameNum() << endl;
                    
                    targetAngle     = tTargetWithLimit;
                    // now we need to figure out if we can go the closest route //
                    bool bCanWeTakeTheCloseRoute = m_jointRestrictor->canReachTarget( i, mCurrentRobotArmAngles[i], targetAngle, tEpsilon, true );
                    
                    tFurtherDiff        = m_jointRestrictor->getAngleToTargetDifference( i, mCurrentRobotArmAngles[i], targetAngle, true );
                    tFurtherLerp        = getLerpAmnt( tFurtherDiff, aDeltaTimef );
                    trestrictedAngle    = m_jointRestrictor->getRestricted( i, mCurrentRobotArmAngles[i] + tFurtherLerp );
                    
                    if( !bCanWeTakeTheCloseRoute ) {
                        tFurtherDiff    = m_jointRestrictor->getAngleToTargetDifference( i, mCurrentRobotArmAngles[i], targetAngle, true );
                        tFurtherLerp    = getLerpAmnt( tFurtherDiff, aDeltaTimef );
                        trestrictedAngle = m_jointRestrictor->getRestricted( i, mCurrentRobotArmAngles[i] - tFurtherLerp );
                    }
                    
                    //cout << "Figure out the closest route: " << bCanWeTakeTheCloseRoute << " tFurtherDiff: " << (tFurtherDiff*RAD_TO_DEG) << " target: " << (targetAngle*RAD_TO_DEG) << " lerp: " << ( tFurtherLerp * RAD_TO_DEG ) << " | " << ofGetFrameNum() << endl;
                    
                    tdesiredAngle = trestrictedAngle;
                    //                    bItsPossibleToReachDesiredAngle = false;
                    
                }
            }
            
            tdesiredAngle = m_jointRestrictor->getRestricted(i,tdesiredAngle);
            
            // START SPEED BASED ACCEL LIMIT
            
            
            float deltaFromCurrentToDesired = ofAngleDifferenceRadians(mCurrentRobotArmAngles[i], tdesiredAngle);
            float curFrameDesiredSpeed = deltaFromCurrentToDesired/aDeltaTimef;
            
            //if our current average radial velocity is basically none - ( stopped ) - lerp down a lot!
            if( fabs(accumulatedRadialVelocity[i]) < 0.01 ){
                if( fabs(speedLimitLerpPerJoint[i]) < 0.001 ){
                    speedLimitLerpPerJoint[i] = 0.0;
                }else{
                    speedLimitLerpPerJoint[i] = ofLerp(speedLimitLerpPerJoint[i], 0.0, 0.3 * aDeltaTimef * 60.0);
                }
            }else{
                
                //otherwise use the difference between our current velocity and what our velocity would be this frame
                //to lerp down really high velocity changes - this should help with sudden stops and sudden starts and sudden changes in direction
                
                float deltaSpeed = ofAngleDifferenceRadians( accumulatedRadialVelocity[i], curFrameDesiredSpeed );
                float deltaSpeedDegrees = deltaSpeed * RAD_TO_DEG;
                
                float slowDownPct = ofMap( fabs(deltaSpeedDegrees), m_minDegreesPerSecToSpeedLerp, m_maxDegreesPerSecToSpeedLerp, 1.0, 0.0, true );
                speedLimitLerpPerJoint[i] = ofLerp(speedLimitLerpPerJoint[i], slowDownPct, 0.7 * aDeltaTimef * 60.0);
                
                if( fabs(deltaSpeedDegrees) > m_minDegreesPerSecToSpeedLerp){
                    cout << "LIMIT: joint " << i  << " - needs slowing down by " << slowDownPct * 100.0 << "%. deltaSpeed is (deg/sec) " << deltaSpeedDegrees << endl;
                }
                
//                if( i == 2 ){
//                    maxDeg = MAX(maxDeg, deltaSpeedDegrees);
//                    cout << "["<< i << "] deltaSpeed is (deg/sec) " << deltaSpeedDegrees << " max is " << maxDeg << endl;
//                }
            }
            
//            if( i == 2 ){
//                cout << i << " speedLimitLerpPerJoint is " << speedLimitLerpPerJoint[i] << endl;
//            }
//            if( i == 2 ){
//                cout << " speedLimitLerpPerJoint is " << speedLimitLerpPerJoint[i] << endl;
//            }
            
            float mapLerp = ofMap(speedLimitLerpPerJoint[i], 0, 0.9, 0.2, 1.0, true);
            
            float prevDesiredAngle = tdesiredAngle;
            tdesiredAngle = m_jointRestrictor->getRestricted(i, ofLerp(mCurrentRobotArmAngles[i], tdesiredAngle, mapLerp * aDeltaTimef * 60.0) );
            if( mapLerp < 0.99 && bOKeyPressed ) {
                cout << i << " - RobotArmSafety :: update : applying slow down : prevDesired: " << ofRadToDeg( prevDesiredAngle ) << " new: " << ofRadToDeg(tdesiredAngle) << " diff: " << ofRadToDeg( ofWrapRadians( tdesiredAngle - prevDesiredAngle )) << " lerp: " << mapLerp << " | " << ofGetFrameNum() << endl;
            }
            
            
            // END SPEED BASED ACCEL LIMIT
            
            mTargetAngles[i]        = targetAngle;
            mTargetRobotAngles[i]   = tdesiredAngle;
            
            float curFrameVel = ofAngleDifferenceRadians(mCurrentRobotArmAngles[i], tdesiredAngle) / aDeltaTimef;
            accumulatedRadialVelocity[i] = ofLerp(accumulatedRadialVelocity[i], curFrameVel, 0.5 * aDeltaTimef * 60.0);
            
            // figure out if we are close enough //
//            float tdiff = ofAngleDifferenceRadians( mCurrentRobotArmAngles[i], targetAngle );
//
//            if( !bItsPossibleToReachDesiredAngle ) {
//                cout << i << " - not possible to reach angle m_currentPoseAngles[i]: " << (mCurrentRobotArmAngles[i]*RAD_TO_DEG) << " target angle: " << (targetAngle*RAD_TO_DEG) << " tdiff: " << fabs( tdiff * RAD_TO_DEG ) << " | " << ofGetFrameNum() << endl;
//            }
//            
////            cout << i << " targetAngle: " << targetPose[i] << " cAngle: " << m_currentPoseAngles[i] << " lerpAngle: " << lerpAngle << " diff: " << tdiff << " epsilon: " << tEpsilon << " | " << ofGetFrameNum() << endl;
//            
//            if( fabs( tdiff ) > tEpsilon ) {
//                bWeAllGood = false;
//            }
            
        }
        
        // now set the desired angles from the robot man for the collider
        // this will check against the actual robot position and alter the desired angles if it
        // detects a potential collision //
        // changing this to use the target angles, since the joint restrictor figures out the proper approach to
        // illegal angles
        
//        mCollision->setDesiredAngles( mTargetRobotAngles );
//        if( mCollision->shouldApply() ) {
//            mCollision->update( aDeltaTimef );
//            mDesiredAngles      = mCollision->getDesiredAngles();
//            mTargetRobotAngles  = mCollision->getDesiredAngles();
//        }
    }
    
    // what's going in, what's coming out?
    if( mTargetRobotAngles.size() > 4 && mDesiredAngles.size() > 4 && bOKeyPressed ) {
//        cout << "targetRobotAngles: " << ofRadToDeg(mTargetRobotAngles[3]) << " desired: " << ofRadToDeg( mDesiredAngles[3] ) << " diff: " << fabs(ofRadToDeg( ofAngleDifferenceRadians( mDesiredAngles[3], mTargetRobotAngles[3] ))) << " mLerpMult: " << mLerpMult << " | " << ofGetFrameNum() << endl;
    }

}

//--------------------------------------------------------------
void RobotArmSafety::draw() {
    if( mCollision && mCollision->shouldApply() ) {
        mCollision->draw();
    }
}

//--------------------------------------------------------------
void RobotArmSafety::draw( UR5KinematicModel* amodel, ofCamera& acam ) {
    if( m_jointRestrictor ) {
        
        //        cout << "mCurrentRobotArmAngles.size(): " << mCurrentRobotArmAngles.size() << " mTargetAngles.size(): " << mTargetAngles.size() << " mTargetRobotAngles.size(): " << mTargetRobotAngles.size() << " | " << ofGetFrameNum() << endl;
        
        if( mCurrentRobotArmAngles.size() && mTargetAngles.size() && mTargetRobotAngles.size() ) {
            ofSetColor( 200 );
            m_jointRestrictor->drawLimits( amodel );
            
            ofSetColor( 130, 240, 60 );
//            m_jointRestrictor->drawAngles( amodel, mTargetRobotAngles );
            m_jointRestrictor->drawAngles( amodel, mDesiredAngles );
            ofSetColor( 200, 240, 20 );
            m_jointRestrictor->drawAngles( amodel, mCurrentRobotArmAngles );
            ofSetColor( 220, 40, 60 );
            m_jointRestrictor->drawAngles( amodel, mTargetAngles );
        }
        
        //            cout << "force stop: " << m_bForceStop << " | " << ofGetFrameNum() << endl;
        ofSetColor( 100, 240, 60 );
        if( !m_bWithinCylinder ) ofSetColor( 220, 40, 60 );
        mCylinderRestrictor->draw();
    }
}

//--------------------------------------------------------------
float RobotArmSafety::getLerpAmnt( float aDiffInRadians, float aDeltaTimef ) {
    float tMaxAngle = (m_maxLerpAngle * DEG_TO_RAD) * aDeltaTimef * 60.0f;
    float lerpAmnt  = m_angleLerp * mLerpMult * aDeltaTimef * 60.0f;
//    if( isMixing() ) {
//        //        lerpAmnt = lerpAmnt * 0.1f;
//        lerpAmnt *= m_mixPct;
//    }
//    float lerpAngle = ofAngleDifferenceRadians( acurrAngle, aTargetAngle ) * lerpAmnt;
    float lerpAngle = aDiffInRadians * lerpAmnt;
    
    if( lerpAngle < -tMaxAngle ) lerpAngle = -tMaxAngle;
    if( lerpAngle > tMaxAngle ) lerpAngle = tMaxAngle;
    return lerpAngle;
}

//--------------------------------------------------------------
vector< double > RobotArmSafety::getTargetRobotAngles() {
    return mTargetRobotAngles;
}

//--------------------------------------------------------------
vector< double > RobotArmSafety::getDesiredAngles() {
    return mDesiredAngles;
}

//--------------------------------------------------------------
bool RobotArmSafety::isArmPoseCloseToTargetPose() {
    if( mCurrentRobotArmAngles.size() == 0 ) return false;
    if( mTargetRobotAngles.size() == 0 ) return false;
    
    float tepsilon = m_angleEpsilon * DEG_TO_RAD * mDeltaTime * 60.0;
    vector< double > tTargetAngles  = mTargetRobotAngles;
    vector< double > tRobotAngles   = mCurrentRobotArmAngles;
    // let's see if we should wait for the robot //
    bool bCloseEnough = true;
    // run through and check to see that the robot is within the epsilon //
    int tNum = MIN( tRobotAngles.size(), tTargetAngles.size() );
    for( int i = 0; i < tNum; i++ ) {
        float tangle = ofAngleDifferenceRadians( tTargetAngles[i], tRobotAngles[i] );
        if( tangle > tepsilon ) {
            bCloseEnough = false;
            break;
        }
    }
    return bCloseEnough;
}

//--------------------------------------------------------------
void RobotArmSafety::setLerpMult( float aMult ) {
    mLerpMult = aMult;
}





