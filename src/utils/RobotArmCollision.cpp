//
//  RobotArmCollision.cpp
//  Mimic-Nick
//
//  Created by Nick Hardeman on 12/13/16.
//

#include "RobotArmCollision.h"

using namespace ofxRobotArm;
//--------------------------------------------------------------
ofParameterGroup &  RobotArmCollision::setup() {
    mModel = shared_ptr< UR5KinematicModel >( new UR5KinematicModel() );
    mModel->setup();
    
    mPredictiveModel = shared_ptr< UR5KinematicModel >( new UR5KinematicModel() );
    mPredictiveModel->setup();
    
    params.setName( "Robot Collisions" );
    params.add( bApply.set("ApplyCollisionDetection", true ));
    params.add( bDrawStops.set("DrawStops", false ));
    params.add( mStopSphereScale.set("StopSphereScale", 1.4, 1.0, 2. ) );
    params.add( bDrawWarnings.set("DrawWarnings", true ));
    params.add( mCorrectStepAngle.set("CorrectStep", 0.05f, 0.005, 1.f ));
    params.add( mMaxCorrectiveAngle.set("MaxCorrectiveAngle", 45, 5, 180 ));
    params.add( mSpherePadding.set("SpherePadding", 1.25, 0.5, 3.f ));
    
    
    bool bAddSlidersToPanel = true;
    // diff paddings for each joint //
    for( int i = 0; i < 6; i++ ) {
        ofParameter< float > tpad;
        float tdefault = 10.0f;
        if( i == 1 ) {
            tdefault = 60.0f;
        } else if( i == 3 ) {
            tdefault = 25;
        } else if( i == 4 ) {
            tdefault = 20;
        }
        tpad.set("Padding_"+ofToString(i,0), tdefault, 1.f, 100.0f );
        if(bAddSlidersToPanel) params.add( tpad );
        mPaddings.push_back( tpad );
    }
    
    
    sphereMesh = ofMesh::icosphere( 1 );
    
    return params;
}

//--------------------------------------------------------------
void RobotArmCollision::setRobotAngles( vector< double > aAcutalRobotAngles ) {
    mModel->setAngles( aAcutalRobotAngles );
    
    if( mAppendages.size() ) {
        for( int i = 0; i < mAppendages.size(); i++ ) {
            if( i < aAcutalRobotAngles.size() ) {
                mAppendages[i].prevAngle    = mAppendages[i].actualRobotAngle;
                if( mAppendages[i].currentAngle == 2000 ) {
                    //                    // this is the first time it's being set, so make it 0 //
                    mAppendages[i].prevAngle = aAcutalRobotAngles[i];
                }
                mAppendages[i].currentAngle     = aAcutalRobotAngles[i];
                mAppendages[i].actualRobotAngle = aAcutalRobotAngles[i];
            }
        }
    }
}

//--------------------------------------------------------------
void RobotArmCollision::setDesiredAngles( vector< double > aDesiredRobotAngles ) {
    mDesiredAngles = aDesiredRobotAngles;
}

//--------------------------------------------------------------
void RobotArmCollision::update( float aDeltaTimef ) {
    if( !mModel ) return;
    
    if( mPrevSpherePadding != mSpherePadding || mPrevSphereStopScale != mStopSphereScale ) {
        mAppendages.clear();
        mPrevSpherePadding      = mSpherePadding;
        mPrevSphereStopScale    = mStopSphereScale;
        return; // need to skip this frame so it doesn't jump
    }
    mPrevSpherePadding      = mSpherePadding;
    mPrevSphereStopScale    = mStopSphereScale;
    

    
    if( !mAppendages.size() ) {
        //        mAppendages.resize( mModel->nodes.size()-1 );
        //        ofLog(OF_LOG_VERBOSE) << "resizing the mAppendages: " << mAppendages.size() << " | " << ofGetFrameNum() << endl;
        // figure out the lengths //
        //        float tspacing  = 50.0f;
        
        for( int i = 0; i < (int)mModel->nodes.size()-1; i++ ) {
            ofNode cnode = mModel->nodes[i];
            ofNode nnode = mModel->nodes[i+1];
            Appendage app;
            
            float tradius   = 40.f;
            if( i == 0 ) {
                tradius = 65.0f;
                //                tradius = 5.0f; // for now, make little so it doesn't conflict with others //
            } else if( i == 1 ) {
                tradius = 50.0f;
            }
            
            float tpadding  = mPaddings[i];
            float tspacing  = tradius * mSpherePadding;
            
            app.length = (nnode.getGlobalPosition() - cnode.getGlobalPosition()).length();
            int tnumSpheres = ceil(app.length / tspacing);
            if( tnumSpheres < 2 ) tnumSpheres = 2;
            
            //            ofLog(OF_LOG_VERBOSE) << i << " - appendage num spheres: " << tnumSpheres << endl;
            for( int k = 0; k < tnumSpheres; k++ ) {
                CollisionSphere tsphere;
                tsphere.pctAlongAppendage = ofMap( k, 0, tnumSpheres-1, 0.0, 1.0, true );
                if( tnumSpheres == 1 ) tsphere.pctAlongAppendage = 1.0f;
                tsphere.lengthAlongAppendage = app.length * tsphere.pctAlongAppendage;
                tsphere.radius          = tradius * mStopSphereScale;
                tsphere.radiusWarning   = tradius + tpadding;
                tsphere.appendageIndex  = i;
                if( k == tnumSpheres-1 ) {
                    tsphere.bLastSphere = true;
                }
                app.spheres.push_back( tsphere );
                //                app.prevSpheres.push_back( tsphere );
                
                //                ofLog(OF_LOG_VERBOSE) << "adding a sphere " << k << " " << endl;
            }
            mAppendages.push_back( app );
            //            ofLog(OF_LOG_VERBOSE) << i << " - where the spheres go? " << mAppendages.back().spheres.size() << endl;
        }
        
        ofLog(OF_LOG_VERBOSE) << "mAppendages.size(): " << mAppendages.size() << " | " << ofGetFrameNum() << endl;
    }
    
    // update the angle velocities //
    for( int i = 0; i < mAppendages.size(); i++ ) {
        //        mAppendages[i].angleVel = aDeltaTimef * 60.f * ofAngleDifferenceRadians( mAppendages[i].prevAngle, mAppendages[i].currentAngle );
        mAppendages[i].angleVel = aDeltaTimef * 60.f * ofWrapRadians( mAppendages[i].currentAngle - mAppendages[i].prevAngle, ofDegToRad(-360), ofDegToRad(360) );
    }
    
    updateAppendages( mModel, mAppendages );
    
    // make sure we have predictive appendages //
    if( mPredictiveAppendages.size() != mAppendages.size() ) {
        mPredictiveAppendages.clear();
        for( int i = 0; i < mAppendages.size(); i++ ) {
            mPredictiveAppendages.push_back( mAppendages[i] );
        }
    }
    
    // now lets calculate the predictive model //
    for( int i = 0; i < mPredictiveAppendages.size(); i++ ) {
        mPredictiveAppendages[i].currentAngle   = mAppendages[i].currentAngle;
        mPredictiveAppendages[i].origAngle      = mPredictiveAppendages[i].currentAngle;
        mPredictiveAppendages[i].angleVel       = mAppendages[i].angleVel;
        //        mPredictiveAppendages[i].currentAngle   = ofLerpRadians( mPredictiveAppendages[i].currentAngle, mDesiredAngles[i], 0.25 * aDeltaTimef * 60.0f );
        //        mPredictiveAppendages[i].currentAngle   += mPredictiveAppendages[i].angleVel;
    }
    if( mPredictiveModel ) {
        updateModel( mPredictiveModel, mPredictiveAppendages );
        //        updateAppendages( mPredictiveModel, mPredictiveAppendages );
    }
    
    //    if( mDesiredAngles.size() > 3 && mPredictiveAppendages.size() > 3 ) {
    //        ofLog(OF_LOG_VERBOSE) << "--------------------------" << endl;
    //        ofLog(OF_LOG_VERBOSE) << "before correction warning: " << isGoingToWarn() << " desired angle: " << mDesiredAngles[2] << " current angle: " << mPredictiveAppendages[2].currentAngle << " | " << ofGetFrameNum() << endl;
    //    }
    
    // get the max angle difference and then create some steps to interpolate in between //
    float tmaxAngleDiff = FLT_MIN;
    for( int i = 0; i < mPredictiveAppendages.size() && i < mDesiredAngles.size(); i++ ) {
        //        float tangleDiff = ofAngleDifferenceRadians(mPredictiveAppendages[i].currentAngle, mDesiredAngles[i] );
        float tangleDiff = ofWrapRadians(mDesiredAngles[i] - mPredictiveAppendages[i].currentAngle, ofDegToRad(-360), ofDegToRad(360) );
        if( fabs(tangleDiff) > tmaxAngleDiff ) {
            tmaxAngleDiff = tangleDiff;
        }
    }
    
    // now we know the greatest angle difference. //
    // create the number of iterations //
    float titerAngle = ofDegToRad( 0.5 );
    int tnumLerpIters = tmaxAngleDiff / titerAngle;
    if( tnumLerpIters < 3 ) tnumLerpIters = 3;
    
    vector< Appendage > origAppendages = mPredictiveAppendages;
    
    int tnumPredictiveApps = mPredictiveAppendages.size();
    for( int tk = 0; tk < tnumLerpIters && tk < mPredictiveAppendages.size() && tk < mDesiredAngles.size(); tk++ ) {
        if( !bApply ) break;
        
        float tpct = ofMap( tk, 0, tnumLerpIters-1, 0.0, 1.0, true );
        // update the mPredictiveAppendages with the lerped values //
        for( int tla = 0; tla < tnumPredictiveApps; tla++ ) {
            //            mPredictiveAppendages[tla].angleVel = ofAngleDifferenceRadians( mPredictiveAppendages[tla].origAngle, mDesiredAngles[tla] );
            mPredictiveAppendages[tla].angleVel     = ofWrapRadians( mDesiredAngles[tla] - mPredictiveAppendages[tla].origAngle, ofDegToRad(-360), ofDegToRad(360) );
            mPredictiveAppendages[tla].currentAngle = ofWrapRadians(ofLerpRadians( mPredictiveAppendages[tla].origAngle, mDesiredAngles[tla], tpct ), -TWO_PI, TWO_PI);
        }
        
        updateModel( mPredictiveModel, mPredictiveAppendages );
        
        if( isGoingToWarn() ) {
            
            ofLog(OF_LOG_NOTICE) << "Going to warn :: hasMainCollisions: " << hasMainCollisionWarnings() << " isNeckHittingForearm: " << isNeckHittingForearm() << " | " << ofGetFrameNum() << endl;
            // try to solve for main collisions first //
            if( hasMainCollisionWarnings() ) {
                
                vector< Appendage > tempApps = mPredictiveAppendages;
                solveMainCollisions();
                //                // now set the predictiveAppendages to the solved ones //
                if( !hasMainCollisionWarnings() ) {
                    for( int npa = 0; npa < mPredictiveAppendages.size(); npa++ ) {
                        mPredictiveAppendages[npa].currentAngle = mDesiredAngles[ npa ];
                        mPredictiveAppendages[npa].angleVel     = ofWrapRadians( mDesiredAngles[npa] - mPredictiveAppendages[npa].origAngle, ofDegToRad(-360), ofDegToRad(360) );
                    }
                } else {
                    mPredictiveAppendages = tempApps;
                    updateModel( mPredictiveModel, mPredictiveAppendages );
                }
                ofLog((OF_LOG_NOTICE)) << "after main solver :: hasMainCollisions : " << hasMainCollisionWarnings() << " isNeckHittingForearm: " << isNeckHittingForearm() << " | " << ofGetFrameNum() << endl;
            }
            
            // do we still have collision warnings //
            //            if( !hasMainCollisionWarnings() && isNeckHittingForearm() ) {
            if( isNeckHittingForearm() ) {
                vector< Appendage > tempApps = mPredictiveAppendages;
                //                solveHeadToForearmCollision( 0 );
                //                if( isGoingToWarn() ) {
                //                    mPredictiveAppendages = tempApps;
                //                    updateModel( mPredictiveModel, mPredictiveAppendages );
                //                }
                //                ofLog((OF_LOG_NOTICE)) << "after forearm solver 1 :: hasMainCollisions : " << hasMainCollisionWarnings() << " isNeckHittingForearm: " << isNeckHittingForearm() << " | " << ofGetFrameNum() << endl;
                if( isNeckHittingForearm() ) {
                    ofLog((OF_LOG_NOTICE)) << "after forearm solver 2 :: hasMainCollisions : " << hasMainCollisionWarnings() << " isNeckHittingForearm: " << isNeckHittingForearm() << " | " << ofGetFrameNum() << endl;
                    tempApps = mPredictiveAppendages;
                    solveHeadToForearmCollision( 1 );
                    if( isGoingToWarn() ) {
                        mPredictiveAppendages = tempApps;
                        updateModel( mPredictiveModel, mPredictiveAppendages );
                    }
                }
                if( isNeckHittingForearm() ) {
                    tempApps = mPredictiveAppendages;
                    solveHeadToForearmCollision( -1 );
                    ofLog((OF_LOG_NOTICE)) << "after forearm solver 3 :: hasMainCollisions : " << hasMainCollisionWarnings() << " isNeckHittingForearm: " << isNeckHittingForearm() << " | " << ofGetFrameNum() << endl;
                    if( isGoingToWarn() ) {
                        mPredictiveAppendages = tempApps;
                        updateModel( mPredictiveModel, mPredictiveAppendages );
                    }
                }
            }
            
            break;
        }
    }
    
    
    if( mDesiredAngles.size() > 3 && mPredictiveAppendages.size() > 3 ) {
        //        ofLog(OF_LOG_VERBOSE) << "after correction warning: " << isGoingToWarn() << " desired angle: " << mDesiredAngles[2] << " current angle: " << mPredictiveAppendages[2].currentAngle << " tdir: " << mPredictiveAppendages[2].angleDir << " | " << ofGetFrameNum() << endl;
    }
    
    // another check using the desired angles //
    for( int i = 0; i < mPredictiveAppendages.size() && i < mDesiredAngles.size(); i++ ) {
        mPredictiveAppendages[i].currentAngle = mDesiredAngles[i];
    }
    updateModel( mPredictiveModel, mPredictiveAppendages );
    //    for( int i = 0; i < mPredictiveAppendages.size(); i++ ) {
    //        if( mPredictiveAppendages[i].bCollidingWarning ) {
    //            mDesiredAngles[i] = mAppendages[i].currentAngle;
    //        }
    //    }
    
    bool bStillGoingToCollide = isGoingToCollide();// || isGoingToWarn();
    for( int i = 0; i < mAppendages.size() && i < mDesiredAngles.size(); i++ ) {
        if( bStillGoingToCollide ) {
            mDesiredAngles[i] = mAppendages[i].actualRobotAngle;
        }
    }
    
    if( mDesiredAngles.size() > 4 ) {
        //        ofLog(OF_LOG_VERBOSE) << "RobotArmCollision :: angle[2]: " << ofRadToDeg( mDesiredAngles[2] ) << " | " << ofGetFrameNum() << endl;
    }
    
}

//--------------------------------------------------------------
void RobotArmCollision::draw() {
    if( !bDrawStops && !bDrawWarnings ) return;
    for( int i = 0; i < mAppendages.size(); i++ ) {
        Appendage& app = mAppendages[i];
        for( auto& cs : app.spheres ) {
            ofSetColor(220, 100, 30 );
            if( i == 1 ) {
                ofSetColor(0, 220, 40 );
            } else if( i == 2 ) {
                ofSetColor( 0, 201, 230 );
            } else if( i == 3 ) {
                ofSetColor(210, 100, 40 );
            } else if( i == 4 ) {
                ofSetColor(190, 20, 180 );
            }
            if( cs.bCollidingWarning ) {
                ofSetColor(200, 230, 30 );
            }
            if( cs.bColliding ) {
                ofSetColor(240, 20, 30 );
            }
            //            if( bDrawStops ) ofDrawSphere( cs.globalPos, cs.radius );
            if( bDrawStops ) {
                ofPushMatrix(); {
                    ofTranslate( cs.globalPos );
                    ofScale(cs.radius, cs.radius, cs.radius );
                    sphereMesh.draw();
                } ofPopMatrix();
            }
            //            ofNoFill();
            //            if( bDrawWarnings ) ofDrawSphere( cs.globalPos, cs.radiusWarning );
            if( bDrawWarnings ) {
                ofPushMatrix(); {
                    ofTranslate( cs.globalPos );
                    ofScale(cs.radiusWarning, cs.radiusWarning, cs.radiusWarning );
                    sphereMesh.drawWireframe();
                } ofPopMatrix();
            }
            //            ofFill();
            ofSetColor(230, 230, 230 );
            //            ofDrawLine( cs.globalPos, cs.globalPos + cs.vel * 50.0f );
        }
        if( i >= 4 ) {
            break;
        }
    }
}

//--------------------------------------------------------------
vector< double > RobotArmCollision::getDesiredAngles() {
    return mDesiredAngles;
}

//--------------------------------------------------------------
void RobotArmCollision::updateAppendages( shared_ptr< UR5KinematicModel > amodel, vector< Appendage >& aAppendages ) {
    // calculate the positions of the sphere //
    // there is one less appendage than nodes //
    //    ofLog(OF_LOG_VERBOSE) << "mModel num nodes: " << mModel->nodes.size() << " | " << ofGetFrameNum() << endl;
    for( int i = 0; i < aAppendages.size(); i++ ) {
        if( i >= amodel->nodes.size() ) continue;
        Appendage& app  = aAppendages[i];
        //        ofNode& anode   = mModel->nodes[i];
        //        // now we need the direction of the joint //
        ofVec3f ndir    = amodel->nodes[i].getLookAtDir();
        ofVec3f nup     = amodel->nodes[i].getUpDir();
        ofQuaternion globalRot = amodel->nodes[i].getGlobalOrientation();
        ofVec3f npos    = amodel->nodes[i].getGlobalPosition();
        if( i == 0 ) {
            ndir *= -1.f;
        } else if( i == 1 ) {
            ndir = globalRot * ofVec3f(0,0,1);
            npos += globalRot * ofVec3f(0,1,0) * -65.f;
        } else if( i == 2 ) {
            ndir = globalRot * ofVec3f(0,0,1);
            npos += globalRot * ofVec3f(0,1,0) * 55.f;
        } else if( i == 3 ) {
            ndir = globalRot * ofVec3f(0,0,1);
            npos += globalRot * ofVec3f(0,1,0) * -45.f;
            npos += ndir * -25.f;
        } else if( i == 4 ) {
            ndir = globalRot * -ofVec3f(0,1,0);
            npos += globalRot * ofVec3f(0,0,1) * 45.f;
            npos += ndir * -15.0;
        }
        
        ndir.normalize();
        //        ofLog(OF_LOG_VERBOSE) << i << " - mModel node: " << " num spheres: " << app.spheres.size() << " | " << ofGetFrameNum() << endl;
        // so we can calculate velocities //
        app.prevSpheres = app.spheres;
        
        float tpadding  = mPaddings[i];
        
        for( int k = 0; k < app.spheres.size(); k++ ) {
            CollisionSphere& cs = app.spheres[k];
            CollisionSphere& ps = app.prevSpheres[k];
            
            cs.radiusWarning = cs.radius + tpadding;
            cs.localPos     = ndir * cs.lengthAlongAppendage;
            cs.globalPos    = cs.localPos + npos;
            cs.vel          = cs.globalPos - ps.globalPos;
            // we will check later in this function if they are colliding //
            cs.bColliding   = false;
            cs.bCollidingWarning = false;
            cs.closestCollisionDistSq = FLT_MAX;
        }
    }
    
    // now lets check the collisions //
    vector< CollisionSphere* > tallSpheres;
    for( int i = 0; i < aAppendages.size(); i++ ) {
        for( int j = 0; j < aAppendages[i].spheres.size(); j++ ) {
            tallSpheres.push_back( &aAppendages[i].spheres[j] );
        }
    }
    
    int tnumSpheres = tallSpheres.size();
    for( int i = 0; i < tnumSpheres-1; i++ ) {
        for( int j = i+1; j < tnumSpheres; j++ ) {
            CollisionSphere* sphereA = tallSpheres[i];
            CollisionSphere* sphereB = tallSpheres[j];
            if( sphereA->appendageIndex == sphereB->appendageIndex ) continue;
            // we don't want to check 3 and 4, since they can't intersect //
            if( sphereA->appendageIndex == sphereB->appendageIndex - 1 ) continue;
            
            // the head shouldn't collide with the last sphere on the elbow //
            if( sphereA->appendageIndex == 2 && sphereB->appendageIndex == 4 ) {
                if( sphereA->bLastSphere ) continue;
            }
            
            // do a dist check //
            float tCollisionDist    = sphereA->radius + sphereB->radius;
            float tCollisionDistSq  = tCollisionDist * tCollisionDist;
            
            float tWarningDistSq = sphereA->radiusWarning + sphereB->radiusWarning;
            tWarningDistSq = tWarningDistSq * tWarningDistSq;
            
            ofVec3f tdiff = sphereA->globalPos - sphereB->globalPos;
            float tdiffLenSq = tdiff.lengthSquared();
            
            if( tdiffLenSq < sphereA->closestCollisionDistSq ) {
                sphereA->closestCollisionDistSq = tdiffLenSq;
            }
            if( tdiffLenSq < sphereB->closestCollisionDistSq ) {
                sphereB->closestCollisionDistSq = tdiffLenSq;
            }
            
            if( tdiffLenSq <= tWarningDistSq ) {
                sphereA->bCollidingWarning = true;
                sphereB->bCollidingWarning = true;
            }
            
            if( tdiffLenSq <= tCollisionDistSq ) {
                sphereA->bColliding = true;
                sphereB->bColliding = true;
            }
        }
    }
    
    for( int i = 0; i < aAppendages.size(); i++ ) {
        aAppendages[i].bColliding       = false;
        aAppendages[i].bCollidingWarning = false;
        for( int j = 0; j < aAppendages[i].spheres.size(); j++ ) {
            if( aAppendages[i].spheres[j].bCollidingWarning ) {
                aAppendages[i].bCollidingWarning = true;
            }
            
            if( aAppendages[i].spheres[j].bColliding ) {
                aAppendages[i].bColliding = true;
            }
        }
    }
}

//--------------------------------------------------------------
void RobotArmCollision::updateModel( shared_ptr< UR5KinematicModel > amodel, vector< Appendage >& aAppendages ) {
    vector< double > tempPredictiveAngles;
    for( int i = 0; i < aAppendages.size(); i++ ) {
        tempPredictiveAngles.push_back( aAppendages[i].currentAngle );
    }
    if( amodel ) {
        amodel->setAngles( tempPredictiveAngles );
        updateAppendages( amodel, aAppendages );
    }
}

//--------------------------------------------------------------
float RobotArmCollision::getClosestCollisionDistanceSq() {
    float tclosestDistSq = FLT_MAX;
    for( int i = 0; i < mAppendages.size(); i++ ) {
        for( int j = 0; j < mAppendages[i].spheres.size(); j++ ) {
            if( mAppendages[i].spheres[j].closestCollisionDistSq < tclosestDistSq ) {
                tclosestDistSq = mAppendages[i].spheres[j].closestCollisionDistSq;
            }
        }
    }
    return tclosestDistSq;
}

//--------------------------------------------------------------
int RobotArmCollision::getNumWarningCollisions() {
    int tnumCols = 0;
    for( auto& app : mAppendages ) {
        for( auto& ts : app.spheres ) {
            if( ts.bCollidingWarning ) {
                tnumCols++;
            }
        }
    }
    return tnumCols;
}

//--------------------------------------------------------------
bool RobotArmCollision::isColliding() {
    bool bC = false;
    for( auto& app : mAppendages ) {
        if( app.bColliding ) {
            bC = true;
            break;
        }
    }
    return bC;
}

//--------------------------------------------------------------
bool RobotArmCollision::isWarning() {
    bool bC = false;
    for( auto& app : mAppendages ) {
        if( app.bCollidingWarning ) {
            bC = true;
            break;
        }
    }
    return bC;
}

//--------------------------------------------------------------
bool RobotArmCollision::isGoingToCollide() {
    bool bC = false;
    for( auto& app : mPredictiveAppendages ) {
        if( app.bColliding ) {
            bC = true;
            break;
        }
    }
    return bC;
}

//--------------------------------------------------------------
bool RobotArmCollision::isGoingToWarn() {
    bool bC = false;
    for( auto& app : mPredictiveAppendages ) {
        if( app.bCollidingWarning ) {
            bC = true;
            break;
        }
    }
    return bC;
}

//--------------------------------------------------------------
bool RobotArmCollision::hasMainCollisionWarnings() {
    if( mPredictiveAppendages.size() < 5 ) return false;
    // base with neck //
    bool baseWithNeck   = mPredictiveAppendages[0].bCollidingWarning && mPredictiveAppendages[3].bCollidingWarning;
    bool baseWithHead   = mPredictiveAppendages[0].bCollidingWarning && mPredictiveAppendages[4].bCollidingWarning;
    bool armWithNeck    = mPredictiveAppendages[1].bCollidingWarning && mPredictiveAppendages[3].bCollidingWarning;
    bool armWithHead    = mPredictiveAppendages[1].bCollidingWarning && mPredictiveAppendages[4].bCollidingWarning;
    return ( baseWithNeck || baseWithHead || armWithNeck || armWithHead );
}

//--------------------------------------------------------------
bool RobotArmCollision::isNeckHittingForearm() {
    if( mPredictiveAppendages.size() < 5 ) return false;
    return( mPredictiveAppendages[2].bCollidingWarning && mPredictiveAppendages[4].bCollidingWarning );
}

//--------------------------------------------------------------
void RobotArmCollision::solveMainCollisions() {
    float tangleStep    = ofDegToRad( mCorrectStepAngle );
    int tMaxIterations  = ofDegToRad( mMaxCorrectiveAngle )/tangleStep;
    bool bSolved        = false;
    
    int tnumPredictiveApps = mPredictiveAppendages.size();
    if( tnumPredictiveApps < 5 ) return; // just in case //
    vector< Appendage > tempStoredPredictive = mPredictiveAppendages;
    
    //            for( int i = tnumPredictiveApps-1; i > 1; i-- ) {
    //            for( int i = 3; i > 0; i-- ) {
    for( int i = 2; i < 4; i++ ) { // works most of the time //
        
        //                mPredictiveAppendages[i] = tempStoredPredictive[i];
        mPredictiveAppendages = tempStoredPredictive;
        
        {
            //                float tcurrentAngle = mPredictiveAppendages[i].currentAngle;
            //                float testAngleA    = ofWrapRadians( mPredictiveAppendages[i].currentAngle + ofDegToRad(1.5f) );
            //                float testAngleB    = ofWrapRadians( mPredictiveAppendages[i].currentAngle - ofDegToRad(1.5f) );
            //                float testDistSqA   = FLT_MAX;
            //                float testDistSqB   = FLT_MAX;
            //
            //                // first determine the direction //
            //                mPredictiveAppendages[i].angleDir = -1;
            //
            //                mPredictiveAppendages[i].currentAngle = testAngleA;
            //                updateModel( mPredictiveModel, mPredictiveAppendages );
            //                testDistSqA = getClosestCollisionDistanceSq();
            //                mPredictiveAppendages[i].currentAngle = testAngleB;
            //                updateModel( mPredictiveModel, mPredictiveAppendages );
            //                testDistSqB = getClosestCollisionDistanceSq();
            //
            //                if( testDistSqA > testDistSqB ) {
            //                    mPredictiveAppendages[i].angleDir = 1.0;
            //                } else {
            //                    mPredictiveAppendages[i].angleDir = -1.0;
            //                }
            //                mPredictiveAppendages[i].currentAngle = tcurrentAngle;
        }
        
        //                for( int j = i; j > 1; j-- ) {
        for( int j = i; j < 4; j++ ) {
            //                    for( int j = tnumPredictiveApps-1; j >= 2; j-- ) {
            Appendage& tapp = mPredictiveAppendages[j];
            
            // get the closest warning //
            //                        float tcurrentAngle = tapp.currentAngle;
            //                        float testAngleA    = ( tapp.currentAngle + ofDegToRad(1.5f) );
            //                        float testAngleB    = ( tapp.currentAngle - ofDegToRad(1.5f) );
            //                        float testDistSqA   = FLT_MAX;
            //                        float testDistSqB   = FLT_MAX;
            
            // first determine the direction //
            float tdir = -1.f;
            
            //                    tapp.currentAngle = testAngleA;
            //                    updateModel( mPredictiveModel, mPredictiveAppendages );
            //                    testDistSqA = getClosestCollisionDistanceSq();
            //                    tapp.currentAngle = testAngleB;
            //                    updateModel( mPredictiveModel, mPredictiveAppendages );
            //                    testDistSqB = getClosestCollisionDistanceSq();
            //
            //                    if( testDistSqA > testDistSqB ) {
            //                        tdir = 1.0;
            //                    } else {
            //                        tdir = -1.0;
            //                    }
            //
            //                    if( j == 2 ) {
            //                        ofLog(OF_LOG_VERBOSE) << "angle dir: " << tapp.angleDir << " testDistA: " << testDistSqA << " testDistB: " << testDistSqB << " | " << ofGetFrameNum() << endl;
            //                    }
            
            //                        tapp.currentAngle = tcurrentAngle;
            //
            tdir = 1;
            if( tapp.angleVel > 0 ) {
                tdir = -1;
            }
            tapp.angleDir = tdir;
            
            
            
            // I think we might need to calculate the direction better,
            // if the joint gets 'fixed' by this algorithm, then it will adjust the ang velocity //
            for( int k = 0; k < tMaxIterations; k++ ) {
                tapp.currentAngle += tangleStep * tapp.angleDir;
                tapp.currentAngle = ( tapp.currentAngle );
                // test to see if there is a warning //
                updateModel( mPredictiveModel, mPredictiveAppendages );
                if( !hasMainCollisionWarnings() ) {
                    // now set the desired angles //
                    mDesiredAngles[j] = mPredictiveAppendages[j].currentAngle;
                    //                    for( int m = j; m < tnumPredictiveApps; m++ ) {
                    //                        if( m < mDesiredAngles.size() ) {
                    //                            if( m == 2 ) {
                    ////                                ofLog(OF_LOG_VERBOSE) << j << " - changing from: " << mDesiredAngles[m] << " to " << mPredictiveAppendages[m].currentAngle << " | " << ofGetFrameNum() << endl;
                    //                            }
                    //                            mDesiredAngles[m] = mPredictiveAppendages[m].currentAngle;
                    //                        }
                    //                    }
                    bSolved = true;
                    break;
                }
            }
            if( bSolved ) {
                break;
            }
        }
        if( bSolved ) {
            break;
        }
    }
}

//--------------------------------------------------------------
void RobotArmCollision::solveHeadToForearmCollision( int aForcedDirection ) {
    float tangleStep    = ofDegToRad( mCorrectStepAngle );
    int tMaxIterations  = ofDegToRad( mMaxCorrectiveAngle )/tangleStep;
    bool bSolved        = false;
    
    int tnumPredictiveApps = mPredictiveAppendages.size();
    if( tnumPredictiveApps < 5 ) return; // just in case //
    vector< Appendage > tempStoredPredictive = mPredictiveAppendages;
    
    for( int i = 4; i > 2; i-- ) {
        mPredictiveAppendages = tempStoredPredictive;
        //        ofLog(OF_LOG_VERBOSE) << "Reseting the mPredictiveAppendages :: " << i << " | " << ofGetFrameNum() << endl;
        for( int j = i; j < 4; j++ ) {
            //                    for( int j = i; j >= 2; j-- ) {
            Appendage& tapp = mPredictiveAppendages[j];
            //            ofLog(OF_LOG_VERBOSE) << "Testing appendage: " << j << " | " << ofGetFrameNum() << endl;
            
            // first determine the direction //
            // get the closest warning //
            float tcurrentAngle = tapp.currentAngle;
            //            float testAngleA    = ( tapp.currentAngle + ofDegToRad(1.5f) );
            //            float testAngleB    = ( tapp.currentAngle - ofDegToRad(1.5f) );
            //            float testDistSqA   = FLT_MAX;
            //            float testDistSqB   = FLT_MAX;
            //            int tnumCollidingA  = 0;
            //            int tnumCollidingB  = 0;
            
            // first determine the direction //
            float tdir          = -1.f;
            
            //            tapp.currentAngle = testAngleA;
            //            updateModel( mPredictiveModel, mPredictiveAppendages );
            //            testDistSqA     = getClosestCollisionDistanceSq();
            //            tnumCollidingA  = getNumWarningCollisions();
            //            tapp.currentAngle = testAngleB;
            //            updateModel( mPredictiveModel, mPredictiveAppendages );
            //            testDistSqB     = getClosestCollisionDistanceSq();
            //            tnumCollidingB  = getNumWarningCollisions();
            //
            //            if( tnumCollidingA == tnumCollidingB ) {
            //                if( testDistSqA > testDistSqB ) {
            //                    tdir = -1.0;
            //                } else {
            //                    tdir = 1.0;
            //                }
            //            } else {
            //                if( tnumCollidingA > tnumCollidingB ) {
            //                    tdir = -1;
            //                } else {
            //                    tdir = 1;
            //                }
            //            }
            
            tdir = 1;
            if( tapp.angleVel > 0 ) {
                tdir = -1;
            }
            
            if( aForcedDirection != 0 ) {
                tdir = (float)aForcedDirection;
            }
            
            tapp.angleDir = tdir;
            
            //                        float tdir = -1.f;
            //                        if( tapp.angleVel < 0 ) {
            //                            tdir = 1;
            //                        }
            //            tapp.angleDir       = tdir;
            //            tapp.currentAngle   = tcurrentAngle;
            
            // I think we might need to calculate the direction better,
            // if the joint gets 'fixed' by this algorithm, then it will adjust the ang velocity //
            for( int k = 0; k < tMaxIterations; k++ ) {
                tapp.currentAngle += tangleStep * tapp.angleDir;
                tapp.currentAngle = ( tapp.currentAngle );
                // test to see if there is a warning //
                updateModel( mPredictiveModel, mPredictiveAppendages );
                if( !isGoingToWarn() ) {
                    // now set the desired angles //
                    //                    ofLog(OF_LOG_VERBOSE) << "Set the new desired angle for " << j << " mpredictive " << ofRadToDeg(mPredictiveAppendages[j].currentAngle) << " | " << ofGetFrameNum() << endl;
                    for( int m = j; m < 4; m++ ) {
                        if( m < mDesiredAngles.size() ) {
                            mDesiredAngles[m] = mPredictiveAppendages[m].currentAngle;
                        }
                    }
                    //                    mDesiredAngles[j] = mPredictiveAppendages[j].currentAngle;
                    bSolved = true;
                    break;
                }
            }
            if( bSolved ) {
                break;
            }
        }
        if( bSolved ) {
            break;
        }
    }
}



