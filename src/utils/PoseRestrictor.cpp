//
//  PoseRestrictor.cpp
//  RobotRecordKinect
//
//  Created by Nick Hardeman on 11/21/16.
//

#include "PoseRestrictor.h"
using namespace ofxRobotArm;
PoseRestrictor::PoseRestrictor(){
    
}
PoseRestrictor::~PoseRestrictor(){
    
}
//--------------------------------------------------------------
ofParameterGroup & PoseRestrictor::setup() {
    params.setName( "Joint_Restrictions" );
    
    params.add( mMinGlobalAngle.set("MinGlobalAngle", -360, -360, 360 ));
    params.add( mMaxGlobalAngle.set("MaxGlobalAngle", 360, -360, 360 ));
    params.add( m_bLimitElbow.set("LimitElbow", false) );
    
    for( int i = 0; i < 6; i++ ) {
        ofParameter< bool > bApplyLimit;
        bApplyLimit.set("ApplyLimit_"+ofToString(i,0), false );
        m_bApplyLimits.push_back( bApplyLimit );
        params.add( bApplyLimit );
        
        ofParameter< float > tminAngle;
        tminAngle.set("MinAngle_"+ofToString(i,0), -45.f, -270.f, 0.0f );
        m_angleMinLimits.push_back( tminAngle );
        params.add( m_angleMinLimits.back() );
        
        ofParameter< float > tmaxAngle;
        tmaxAngle.set("MaxAngle_"+ofToString(i,0), 45.f, -50.f, 270.0f );
        m_angleMaxLimits.push_back( tmaxAngle );
        params.add( m_angleMaxLimits.back() );
        
    }
    
    setShoulderAngle(m_angleMaxLimits[1]/2 + m_angleMinLimits[1]/2);
    
    return params;
}

//--------------------------------------------------------------
void PoseRestrictor::update( float aDeltaTimef ) {
    
}

//--------------------------------------------------------------
void PoseRestrictor::setShoulderAngle(float angle){
    shoulderAngle = angle * RAD_TO_DEG;
}

//--------------------------------------------------------------
float PoseRestrictor::getMinJointAngle(int aIndex){
    float angle = 0;
    if(aIndex >= 0 && aIndex < m_angleMinLimits.size() ){
        angle = m_angleMinLimits[aIndex];
        
        //this limits the elbow angle adaptively based on the shoulder angle
        if( m_bLimitElbow && aIndex == 2){
            float limit = ofMap(shoulderAngle, m_angleMinLimits[1]+10, m_angleMinLimits[1]+40, 0.0, 1.0, true);
            angle *= limit;
        }
        
    }
    
    return angle;
}

//--------------------------------------------------------------
float PoseRestrictor::getMaxJointAngle(int aIndex){
    float angle = 0;
    if(aIndex >= 0 && aIndex < m_angleMaxLimits.size() ){
        angle = m_angleMaxLimits[aIndex];
        
        //this limits the elbow angle adaptively based on the shoulder angle
        if( m_bLimitElbow && aIndex == 2){
            float limit = ofMap(shoulderAngle, m_angleMaxLimits[1]-10, m_angleMaxLimits[1]-40, 0.0, 1.0, true);
            angle *= limit;
        }
    }
    return angle;
}

//--------------------------------------------------------------
void PoseRestrictor::drawLimits( RobotModel* amodel ) {
    if( amodel == NULL ) return;
    // loop through each node and show the limits //
    for( int i = 0; i < amodel->pose.size(); i++ ) {
        if( !m_bApplyLimits[i] ) continue;
        
        vector< ofVec3f > taxes = getAxes( amodel, i );
        
        if( i < m_angleMinLimits.size() && i < m_angleMaxLimits.size() ) {
            ofPushMatrix(); {
                ofTranslate( taxes[2] );
                ofScale( 100, 100, 100 );
                //                ofSetColor( 200 );
                drawArc(getMinJointAngle(i), getMaxJointAngle(i), taxes[0], taxes[1] );
            } ofPopMatrix();
        }
    }
}

//--------------------------------------------------------------
void PoseRestrictor::drawAngles( RobotModel* amodel, vector< double > aCurrentAngles ) {
    if( amodel == NULL ) return;
    // loop through each node and show the limits //
    for( int i = 0; i < amodel->pose.size(); i++ ) {
        if( i >= aCurrentAngles.size() ) return;
        if( !m_bApplyLimits[i] ) continue;
        
        vector< ofVec3f > taxes = getAxes( amodel, i );
        
        ofPushMatrix(); {
            ofTranslate( taxes[2] );
            ofScale( 100, 100, 100 );
            //            ofSetColor( 220, 240, 20 );
            ofVec3f cvec = taxes[0];
            cvec.rotate( aCurrentAngles[i] * RAD_TO_DEG, taxes[1] );
            ofDrawLine( ofVec3f(), cvec );
            
        } ofPopMatrix();
    }
}

//--------------------------------------------------------------
vector< ofVec3f > PoseRestrictor::getAxes( RobotModel* amodel, int aIndex ) {
    vector< ofVec3f > taxes;
    taxes.push_back( ofVec3f(-1,0,0) );
    taxes.push_back( ofVec3f(0,-1,0 ) );
    taxes.push_back( ofVec3f( ) ); // position //
    if( amodel == NULL ) return taxes;
    if( aIndex >= m_bApplyLimits.size() ) return taxes;
    if( aIndex >= amodel->nodes.size() ) return taxes;
    if( aIndex >= amodel->pose.size() ) return taxes;
    
    ofNode& cnode   = amodel->nodes[ aIndex ];
    
    ofVec3f tpos    = cnode.getGlobalPosition();
    
    ofVec3f tfwd    = ofVec3f(0,1,0);//cnode.getLookAtDir();
    ofVec3f tup     = ofVec3f(0,0,1);//cnode.getUpDir();
    //        ofVec3f tside   = cnode.getSideDir();
    
    ofVec3f taxis   = amodel->pose[ aIndex ].axis;
    
    if( cnode.getParent() != nullptr ) {
        ofNode* pnode = amodel->nodes[ aIndex ].getParent();
        ofQuaternion tGParentQ = pnode->getGlobalOrientation();
        if( aIndex == 0 ) {
            
        } else if( aIndex == 1 ) {
            tfwd    = tGParentQ * ofVec3f( -1, 0, 0 );
            tup     = tGParentQ * ofVec3f( 0, -1, 0 );
        } else if( aIndex == 2 ) {
            tfwd    = tGParentQ * ofVec3f( 0, 0, 1 );
            tup     = tGParentQ * ofVec3f( 0, -1, 0 );
        } else if( aIndex == 3  ) {
            tfwd    = tGParentQ * ofVec3f( -1, 0, 0 );
            tup     = tGParentQ * ofVec3f( 0, -1, 0 );
        } else if( aIndex == 4 ) {
            tfwd    = tGParentQ * ofVec3f( 0, -1, 0 );
            tup     = tGParentQ * ofVec3f( 0, 0, 1 );
        } else if( aIndex == 5 ) {
            tfwd    = tGParentQ * ofVec3f( 1, 0, 0 );
            tup     = tGParentQ * ofVec3f( 0, 1, 0 );
        }
    }
    
    taxes[0] = tfwd;
    taxes[1] = tup;
    taxes[2] = tpos;
    
    return taxes;
}

//--------------------------------------------------------------
void PoseRestrictor::drawArc( float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis ) {
    float startDegrees = aStartAngleDegrees;//aStartAngleRadians * RAD_TO_DEG;
    float endDegrees = aEndAngleDegrees;//aEndAngleRadians * RAD_TO_DEG;
    float tangleDiff = endDegrees - startDegrees;//ofAngleDifferenceDegrees( startDegrees, endDegrees );
    int iterations = fabs(tangleDiff) / 4;
    if( iterations < 2 ) iterations = 2;
    
    float currDegree = startDegrees;
    float tstep = tangleDiff / (float)iterations;
    
    ofVec3f cvec = aForwardAxis;
    //    cvec.rotate( startDegrees, aSideAxis );
    
    //    ofLog(OF_LOG_VERBOSE) << "currDegree " << currDegree << " tstep: " << tstep << " angle diff: " << tangleDiff << " iterations: " << iterations << " | " << ofGetFrameNum() << endl;
    
    ofMesh tmesh;
    tmesh.setMode( OF_PRIMITIVE_LINE_STRIP );
    
    tmesh.addVertex( ofVec3f() );
    for( int i = 0; i <= iterations; i++ ) {
        cvec = aForwardAxis;
        cvec.rotate( currDegree, aSideAxis );
        tmesh.addVertex( cvec );
        currDegree += tstep;
        currDegree = ofWrapDegrees( currDegree );
    }
    tmesh.addVertex( ofVec3f() );
    
    tmesh.draw();
}

//--------------------------------------------------------------
float PoseRestrictor::getRestricted( int aIndex, float aInAngleRadians ) {
    if( aIndex >= m_bApplyLimits.size() ) {
        //        ofLog(OF_LOG_VERBOSE) << "PoseRestrictor :: getRestricted : aindex " << aIndex << " is too high!! " << endl;
        return aInAngleRadians;
    }
    
    //    if( !m_bApplyLimits[aIndex] ) return aInAngleRadians;
    
    //    float tinDegree = ofWrapRadians(aInAngleRadians) * RAD_TO_DEG;
    float tinDegree     = aInAngleRadians * RAD_TO_DEG;
    float tminDegree    = getMinJointAngle(aIndex);
    float tmaxDegree    = getMaxJointAngle(aIndex);
    
    if( !m_bApplyLimits[ aIndex ] ) {
        tminDegree = mMinGlobalAngle;
        tmaxDegree = mMaxGlobalAngle;
        //        return aInAngleRadians;
    }
    
    return ofClamp( tinDegree, tminDegree, tmaxDegree ) * DEG_TO_RAD;
}

//--------------------------------------------------------------
bool PoseRestrictor::canReachTarget( int aIndex, float aCurrentAngleInRadians, float aTargetInRadians, float aEpsilonRadians, bool bUseClosest ) {
    if( aIndex >= m_bApplyLimits.size() ) {
        //        ofLog(OF_LOG_VERBOSE) << "PoseRestrictor :: canReachTarget : aindex " << aIndex << " is too high!! " << endl;
        return true;
    }
    
    //    if( !m_bApplyLimits[aIndex] ) return true;
    
    //    float tMinRadians = m_angleMinLimits[ aIndex ] * DEG_TO_RAD;
    //    float tMaxRadians = m_angleMaxLimits[ aIndex ] * DEG_TO_RAD;
    
    //    float lerpAngle = ofAngleDifferenceRadians( m_currentPoseAngles[i], targetAngle ) * lerpAmnt;
    // now let's make a step and see if we can make it //
    float tdiffRadians = ofAngleDifferenceRadians( aCurrentAngleInRadians, aTargetInRadians );
    // take the long way home
    if( !bUseClosest ) {
        tdiffRadians = TWO_PI - tdiffRadians;
    }
    int tNumIterations = fabs(tdiffRadians) / (aEpsilonRadians/4.f);
    if( tNumIterations < 1 ) tNumIterations = 1;
    float tstep = tdiffRadians / (float)tNumIterations;
    
    if( !bUseClosest ) {
        tstep = -tstep;
    }
    
//    ofLog(OF_LOG_VERBOSE) << "Lets run through this " << ( aCurrentAngleInRadians * RAD_TO_DEG ) << " target: " << (aTargetInRadians*RAD_TO_DEG) << " iterations: " << tNumIterations << " tstep: " << (tstep * RAD_TO_DEG ) << " bUseClosest: " << bUseClosest << " tdiffRadians: " << tdiffRadians << endl;
    
    float tepsilon      = aEpsilonRadians;// * DEG_TO_RAD;
    for( int i = 0; i < tNumIterations; i++ ) {
        //        float cradian       = ofWrapRadians( aCurrentAngleInRadians + (float)i * tstep );
        float cradian       = ( aCurrentAngleInRadians + (float)i * tstep );
        float trestricted   = getRestricted( aIndex, cradian );
        float tdiff         = fabs( ofAngleDifferenceRadians(cradian, aTargetInRadians) );
        if( tdiff <= tepsilon*0.5f ) {
            return true;
        }
//        ofLog(OF_LOG_VERBOSE) << i << " - cradian: " << (cradian * RAD_TO_DEG ) << " trestricted: " << (trestricted * RAD_TO_DEG ) << " tdiff: " << (tdiff*RAD_TO_DEG) << " | " << ofGetFrameNum() << endl;
        //        if( cradian != trestricted ) {
        if( fabs(cradian-trestricted) > (0.01 * DEG_TO_RAD) ) {
//            ofLog(OF_LOG_VERBOSE) << " xxxxx cradian != trestricted: " << cradian << " trestricted: " << trestricted << endl;
            return false;
        }
    }
    return false;
}

//--------------------------------------------------------------
float PoseRestrictor::getAngleToTargetDifference( int aIndex, float aCurrentAngleInRadians, float aTargetInRadians, bool bUseClosest ) {
    float tangleRadians = aCurrentAngleInRadians;
    if( !bUseClosest ) {
        tangleRadians = TWO_PI - aCurrentAngleInRadians;
    }
    tangleRadians = getRestricted( aIndex, tangleRadians );
    float tdiffRadians  = ofAngleDifferenceRadians( tangleRadians, aTargetInRadians );
    return tdiffRadians;
}

//--------------------------------------------------------------
float PoseRestrictor::getCloserLimit( int aIndex, float aTargetInRadians ) {
    float tMinRadians = getMinJointAngle( aIndex ) * DEG_TO_RAD;
    float tMaxRadians = getMaxJointAngle( aIndex ) * DEG_TO_RAD;
    
    float tMinDiff = fabs(ofAngleDifferenceRadians( aTargetInRadians, tMinRadians ));
    float tMaxDiff = fabs(ofAngleDifferenceRadians( aTargetInRadians, tMaxRadians ));
    if( tMinDiff <= tMaxDiff ) {
        return tMinRadians;
    }
    return tMaxRadians;
}






