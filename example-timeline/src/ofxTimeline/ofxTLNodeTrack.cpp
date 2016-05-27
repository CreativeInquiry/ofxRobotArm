/**
 * ofxTimeline
 * openFrameworks graphical timeline addon
 *
 * Copyright (c) 2011-2012 James George
 * Development Supported by YCAM InterLab http://interlab.ycam.jp/en/
 * http://jamesgeorge.org + http://flightphase.com
 * http://github.com/obviousjim + http://github.com/flightphase
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "ofxTLNodeTrack.h"
#include "ofxTimeline.h"


ofVec3f ofHermiteInterpolate(ofVec3f y0, ofVec3f y1, ofVec3f y2, ofVec3f y3, float pct, float tension, float bias){
	ofVec3f m0,m1;
	float pct2,pct3;
	float a0,a1,a2,a3;
	pct2 = pct * pct;
	pct3 = pct2 * pct;
	m0  = (y1-y0)*(1+bias)*(1-tension)/2;
	m0 += (y2-y1)*(1-bias)*(1-tension)/2;
	m1  = (y2-y1)*(1+bias)*(1-tension)/2;
	m1 += (y3-y2)*(1-bias)*(1-tension)/2;
	a0 =  2*pct3 - 3*pct2 + 1;
	a1 =  pct3 - 2*pct2 + pct;
	a2 =  pct3 - pct2;
	a3 =  -2*pct3 + 3*pct2;
	return(a0*y1 + a1*m0+a2*m1+a3*y2);
}

ofxTLNodeTrack::ofxTLNodeTrack(){
	node = NULL;
	lockNodeToTrack = false;
	dampening = .1;
}

ofxTLNodeTrack::~ofxTLNodeTrack(){
	
}

void ofxTLNodeTrack::enable(){
	ofxTLKeyframes::enable();
	if(enabled){
		ofAddListener(ofEvents().update, this, &ofxTLNodeTrack::update);
	}
}

void ofxTLNodeTrack::disable(){
	ofxTLKeyframes::disable();
	if(!enabled){
		ofRemoveListener(ofEvents().update, this, &ofxTLNodeTrack::update);
	}
}

void ofxTLNodeTrack::setNode(ofNode& _node)
{
	node = &_node;
}


void ofxTLNodeTrack::draw(){
	//draw your keyframes into bounds
	ofPushStyle();

	if(lockNodeToTrack){
		ofSetColor(timeline->getColors().keyColor, 40*(sin(ofGetElapsedTimef()*5)*.5+.5)+25);
		ofFill();
		ofRect(bounds);
	}
	ofSetColor(timeline->getColors().keyColor);
	ofNoFill();


	//	for(int i = 0; i < track.getSamples().size(); i++){
	for(int i = 0; i < keyframes.size(); i++){
		if(!isKeyframeIsInBounds(keyframes[i])){
			continue;
		}
		
		ofxTLNodeFrame* sample =(ofxTLNodeFrame*)keyframes[i];
		float screenX = millisToScreenX(keyframes[i]->time);
		float screenY = bounds.y;
		ofPoint screenPoint = ofPoint(screenX,screenY);
		
		//        if(keyframes[i] == selectedKeyframe){
		if(isKeyframeSelected(sample)){
			if(sample->easeInSelected){
				ofSetColor(timeline->getColors().highlightColor);
				draweEase(sample->easeIn, screenPoint, true);
				ofSetColor(timeline->getColors().keyColor);
				draweEase(sample->easeOut, screenPoint, false);
			}
			else {
				ofSetColor(timeline->getColors().keyColor);
				draweEase(sample->easeIn, screenPoint, true);
				ofSetColor(timeline->getColors().highlightColor);
				draweEase(sample->easeOut,screenPoint, false);
			}
		}
		else{
			ofSetColor(timeline->getColors().keyColor);
			draweEase(sample->easeIn,  screenPoint, true);
			draweEase(sample->easeOut, screenPoint, false);
		}
	}

	ofFill();
	ofSetColor(timeline->getColors().highlightColor);
	for(int i = 0; i < selectedKeyframes.size(); i++){
		float screenX = millisToScreenX( selectedKeyframes[i]->time );
		float screenY = bounds.y+bounds.height/2;
		ofCircle(screenX, screenY, 4);
	}

	ofPopStyle();

}

void ofxTLNodeTrack::draw3d(){
	
	if(lockNodeToTrack) return;
	
	ofxTLNodeFrame interFrame;
	ofNode n;
	ofPushStyle();

	
	for(int i = 0; i  < keyframes.size(); i++){
		ofxTLNodeFrame* frame = (ofxTLNodeFrame*)keyframes[i];
		n.setPosition(frame->position);
		n.setOrientation(frame->orientation);
		n.draw();
		ofPushStyle();
		ofPopStyle();
	}
	
	unsigned long long startMillis = screenXToMillis(bounds.x);
	unsigned long long endMillis = screenXToMillis(bounds.getMaxX());
	unsigned long long step = (endMillis - startMillis)/100;
	for(unsigned long long millis = startMillis; millis < endMillis; millis += step ){
		setFrameToTime(&interFrame, millis);
		n.setPosition(interFrame.position);
		n.setOrientation(interFrame.orientation);
		ofSetColor(0,0,255);
		ofLine(n.getPosition(), n.getPosition() + n.getLookAtDir()*10);
		ofSetColor(0,255,0);
		ofLine(n.getPosition(), n.getPosition() + n.getUpDir()*10);
		ofSetColor(255,0,0);
		ofLine(n.getPosition(), n.getPosition() + n.getSideDir()*10);
	}

	setFrameToTime(&interFrame, currentTrackTime());
	n.setPosition(interFrame.position);
	n.setOrientation(interFrame.orientation);
	
	ofSetLineWidth(3);
	ofSetColor(0,0,255);
	ofLine(n.getPosition(), n.getPosition() + n.getLookAtDir()*25);
	ofSetColor(0,255,0);
	ofLine(n.getPosition(), n.getPosition() + n.getUpDir()*25);
	ofSetColor(255,0,0);
	ofLine(n.getPosition(), n.getPosition() + n.getSideDir()*25);
	
	ofNoFill();
	ofSetColor(255);
	ofBox(n.getPosition(), 4);
	
	ofPopStyle();
}

void ofxTLNodeTrack::draweEase(NodeTrackEase ease, ofPoint screenPoint, bool easeIn){
    switch (ease) {
        case OFXTL_NODE_EASE_LINEAR:
            if(easeIn){
                ofTriangle(screenPoint.x-bounds.height, screenPoint.y,
                           screenPoint.x, screenPoint.y,
                           screenPoint.x, screenPoint.y+bounds.height);
            }
            else{
                ofTriangle(screenPoint.x, screenPoint.y,
                           screenPoint.x, screenPoint.y+bounds.height,
                           screenPoint.x+bounds.height, screenPoint.y+bounds.height);
            }
            break;
        case OFXTL_NODE_EASE_SMOOTH:
            if(easeIn){
                ofBezier(screenPoint.x-bounds.height, screenPoint.y,
                         screenPoint.x-bounds.height/2, screenPoint.y,
                         screenPoint.x, screenPoint.y+bounds.height/2,
                         screenPoint.x, screenPoint.y+bounds.height);
                ofLine(screenPoint.x-bounds.height, screenPoint.y,
                       screenPoint.x, screenPoint.y);
                ofLine(screenPoint.x, screenPoint.y,
                       screenPoint.x, screenPoint.y+bounds.height);
            }
            else {
                ofBezier(screenPoint.x, screenPoint.y,
                         screenPoint.x, screenPoint.y+bounds.height/2,
                         screenPoint.x+bounds.height/2, screenPoint.y+bounds.height,
                         screenPoint.x+bounds.height, screenPoint.y+bounds.height);
                ofLine(screenPoint.x, screenPoint.y,
                       screenPoint.x, screenPoint.y+bounds.height);
                ofLine(screenPoint.x, screenPoint.y+bounds.height,
                       screenPoint.x+bounds.height, screenPoint.y+bounds.height);
            }
            break;
        case OFXTL_NODE_EASE_CUT:
            if(easeIn){
	            ofRect(screenPoint.x-bounds.height/2, screenPoint.y,
                       bounds.height/2, bounds.height/2);
            }
            else{
                ofRect(screenPoint.x, screenPoint.y+bounds.height/2,
                       bounds.height/2, bounds.height/2);
            }
            break;
        default:
            ofLogError("ofxTLNodeTrack::draweEase -- invalid ease");
            break;
    }
}

void ofxTLNodeTrack::update(ofEventArgs& args){
	if(lockNodeToTrack){
//		cout << "moving camera " << timeline << " + " << timeline->getName() << endl;
		moveToTime(timeline->getCurrentTimeMillis());
	}
}

void ofxTLNodeTrack::setTimelineInOutToTrack(){
	//TODO: timebased camera tracking
	if(keyframes.size() > 0){
		unsigned long long inTime  = keyframes[0]->time;
		unsigned long long outTime = keyframes[keyframes.size()-1]->time;
//		cout << " IN AND OUT SETTING " << inTime << " " << outTime << endl;
		timeline->setInOutRangeMillis(inTime, outTime);
//		timeline->setInPointAtMillis(keyframes[0]->time);
//		timeline->setOutPointAtMillis(keyframes[keyframes.size()-1]->time);
	}
	else{
		timeline->setInOutRange(ofRange(0,1.0));
	}
}

bool ofxTLNodeTrack::mousePressed(ofMouseEventArgs& args, long millis){
	//for the general behavior call the super class
	//or you can do your own thing. Return true if the click caused an item to
	//become selectd
	bool ret = ofxTLKeyframes::mousePressed(args, millis);
	createNewOnMouseup = false;
	if(selectedKeyframe != NULL){
		ofxTLNodeFrame* sample = (ofxTLNodeFrame*)selectedKeyframe;
		sample->easeInSelected = args.x < millisToScreenX(sample->time);
	}
	return ret;

}

void ofxTLNodeTrack::mouseMoved(ofMouseEventArgs& args, long millis){
	ofxTLKeyframes::mouseMoved(args, millis);
}

void ofxTLNodeTrack::mouseDragged(ofMouseEventArgs& args, long millis){
	ofxTLKeyframes::mouseDragged(args, millis);
}

void ofxTLNodeTrack::mouseReleased(ofMouseEventArgs& args, long millis){
	ofxTLKeyframes::mouseReleased(args, millis);
}

//keys pressed events, and nuding from arrow keys with normalized nudge amount 0 - 1.0
void ofxTLNodeTrack::keyPressed(ofKeyEventArgs& args){
	ofxTLKeyframes::keyPressed(args);
	
	bool modified = false;
	for(int i = 0; i < selectedKeyframes.size(); i++){
		ofxTLNodeFrame* key = (ofxTLNodeFrame*)selectedKeyframes[i];
		if(args.key == OF_KEY_UP){
			if(key->easeInSelected){
				key->easeIn = getPreviousEase(key->easeIn);
			}
			else {
				key->easeOut = getPreviousEase(key->easeOut);
			}
			modified = true;
		}
		else if(args.key == OF_KEY_DOWN){
			if(key->easeInSelected){
				key->easeIn = getNextEase(key->easeIn);
			}
			else {
				key->easeOut = getNextEase(key->easeOut);
			}
			modified = true;
		}
	}
	
	if(modified){
		timeline->flagTrackModified(this);
	}
	
}

void ofxTLNodeTrack::regionSelected(ofLongRange timeRange, ofRange valueRange){
    for(int i = 0; i < keyframes.size(); i++){
    	if(timeRange.contains( keyframes[i]->time )){
            selectKeyframe(keyframes[i]);
        }
	}
}

string ofxTLNodeTrack::getTrackType(){
	return "CameraTrack";
}

ofxTLKeyframe* ofxTLNodeTrack::newKeyframe(){
	//return our type of keyframe, stored in the parent class
	ofxTLNodeFrame* newKey = new ofxTLNodeFrame();
	if(node != NULL){
		newKey->position = node->getPosition();
		newKey->orientation = node->getOrientationQuat();
	}
	newKey->easeIn = newKey->easeOut = OFXTL_NODE_EASE_LINEAR;

	return newKey;
}

void ofxTLNodeTrack::restoreKeyframe(ofxTLKeyframe* key, ofxXmlSettings& xmlStore){
	ofxTLNodeFrame* cameraFrame = (ofxTLNodeFrame*)key;
	cameraFrame->position = ofVec3f(xmlStore.getValue("px", 0.),
						 			xmlStore.getValue("py", 0.),
									xmlStore.getValue("pz", 0.));
	cameraFrame->orientation.set(xmlStore.getValue("ox", 0.),
								  xmlStore.getValue("oy", 0.),
								  xmlStore.getValue("oz", 0.),
								  xmlStore.getValue("ow", 1.));
	cameraFrame->easeIn  = (NodeTrackEase)xmlStore.getValue("easein", (int)OFXTL_NODE_EASE_LINEAR);
	cameraFrame->easeOut = (NodeTrackEase)xmlStore.getValue("easeout", (int)OFXTL_NODE_EASE_LINEAR);
}

void ofxTLNodeTrack::storeKeyframe(ofxTLKeyframe* key, ofxXmlSettings& xmlStore){
	ofxTLNodeFrame* cameraFrame = (ofxTLNodeFrame*)key;
	xmlStore.addValue("px", cameraFrame->position.x);
	xmlStore.addValue("py", cameraFrame->position.y);
	xmlStore.addValue("pz", cameraFrame->position.z);
	
	xmlStore.addValue("ox", cameraFrame->orientation._v.x);
	xmlStore.addValue("oy", cameraFrame->orientation._v.y);
	xmlStore.addValue("oz", cameraFrame->orientation._v.z);
	xmlStore.addValue("ow", cameraFrame->orientation._v.w);
	
	xmlStore.addValue("easein",  (int)cameraFrame->easeIn);
	xmlStore.addValue("easeout", (int)cameraFrame->easeOut);
}

ofxTLKeyframe* ofxTLNodeTrack::keyframeAtScreenpoint(ofVec2f p){
    if(bounds.inside(p.x, p.y)){
        for(int i = 0; i < keyframes.size(); i++){
            float offset = p.x - timeline->millisToScreenX(keyframes[i]->time);
            if (abs(offset) < bounds.height/2) {
                return keyframes[i];
            }
        }
    }
	return NULL;
}

void ofxTLNodeTrack::moveToTime(unsigned long long millis){
	if(node ==NULL){
		ofLogError("ofxTLNodeTrack -- can't modify a null camera!");
		return;
	}
	
	if(keyframes.size() == 0){
		return;
	}
	
	if(keyframes.size() == 1 || millis <= keyframes[0]->time){
		ofxTLNodeFrame* firstFrame = (ofxTLNodeFrame*)keyframes[0];
//		node->setPosition(firstFrame->position);
//		node->setOrientation(firstFrame->orientation);
		moveToPosition(firstFrame);

		return;
	}
	
	if (millis >= keyframes[keyframes.size()-1]->time) {
		ofxTLNodeFrame* lastFrame = (ofxTLNodeFrame*)keyframes[keyframes.size()-1];
//		node->setPosition(lastFrame->position);
//		node->setOrientation(lastFrame->orientation);
		moveToPosition(lastFrame);
		return;
	}
	
	//cout << "Sampling at frame " << frame << " with frames ranging between " << samples[0].frame << " - " << samples[samples.size()-1].frame << endl;
	
	ofxTLNodeFrame interp;
	setFrameToTime(&interp, millis);
	moveToPosition(&interp);
//	node->setPosition(interp.position);
//	node->setOrientation(interp.orientation);
	//	cout << "set position to " << node->getPosition() << endl;
}

void ofxTLNodeTrack::setFrameToTime(ofxTLNodeFrame* target, unsigned long long millis){
	for(int i = 1; i < keyframes.size(); i++){
		if(keyframes[i]->time > millis){
			ofxTLNodeFrame* prev = (ofxTLNodeFrame*)(i > 2 ? keyframes[i-2] : keyframes[i-1]);
			ofxTLNodeFrame* next = (ofxTLNodeFrame*)(i < keyframes.size()-1 ? keyframes[i+1] : keyframes[i]);
			interpolateBetween(target, prev, (ofxTLNodeFrame*)keyframes[i-1], (ofxTLNodeFrame*)keyframes[i], next, millis);
			break;
		}
	}
}

void ofxTLNodeTrack::moveToPosition(ofxTLNodeFrame* target){
	node->setPosition(node->getPosition().getInterpolated(target->position, dampening) );
	ofQuaternion q;
	q.slerp(dampening, node->getOrientationQuat(), target->orientation);
	node->setOrientation(q);
}

void ofxTLNodeTrack::setDampening(float damp){
	dampening = damp;
}

float ofxTLNodeTrack::getDampening(){
	return dampening;
}

void ofxTLNodeTrack::jumpToTarget(){
    float curDamp = dampening;
    dampening = 1.0;
    moveToTime(timeline->getCurrentTimeMillis());
    
    dampening = curDamp;
}

void ofxTLNodeTrack::interpolateBetween(ofxTLNodeFrame* target,
										  ofxTLNodeFrame* prev,
										  ofxTLNodeFrame* sample1,
										  ofxTLNodeFrame* sample2,
										  ofxTLNodeFrame* next, unsigned long long millis)
{

    float alpha = 0;
    //CUT
    if(sample1->easeOut == OFXTL_NODE_EASE_CUT || sample2->easeIn == OFXTL_NODE_EASE_CUT ){
        alpha = 0;
    }
    //LINEAR
    else if(sample1->easeOut == OFXTL_NODE_EASE_LINEAR && sample2->easeIn == OFXTL_NODE_EASE_LINEAR){
        alpha = ofMap(millis, sample1->time, sample2->time, 0, 1.0, false);
    }
    //EASE IN
    else if(sample1->easeOut == OFXTL_NODE_EASE_SMOOTH && sample2->easeIn == OFXTL_NODE_EASE_LINEAR){
        ofxEasingQuad ease;
        alpha = ofxTween::map(millis, sample1->time, sample2->time, 0, 1.0, false, ease, ofxTween::easeIn);
    }
    //EASE OUT
    else if(sample1->easeOut == OFXTL_NODE_EASE_LINEAR && sample2->easeIn == OFXTL_NODE_EASE_SMOOTH){
        ofxEasingQuad ease;
        alpha = ofxTween::map(millis, sample1->time, sample2->time, 0, 1.0, false, ease, ofxTween::easeOut);
    }
    //EASE IN OUT
    else if(sample1->easeOut == OFXTL_NODE_EASE_SMOOTH && sample2->easeIn == OFXTL_NODE_EASE_SMOOTH){
        ofxEasingQuad ease;
        alpha = ofxTween::map(millis, sample1->time, sample2->time, 0, 1.0, false, ease, ofxTween::easeInOut);
    }
    
	//float alpha = ofMap(millis, sample1->time, sample2->time, 0, 1.0, false);
    
	target->time = millis;
	target->position = ofHermiteInterpolate(prev->position,sample1->position,sample2->position,next->position,alpha,0,0);
	//	interp.position = sample1.position.getInterpolated(sample2.position, alpha);
	target->orientation.slerp(alpha, sample1->orientation, sample2->orientation);
	//cout << "interpolating between " << sample1.position << " and " << sample2.position << " with alpha " << alpha << endl;
}

void ofxTLNodeTrack::selectedKeySecondaryClick(ofMouseEventArgs& args){
	//you can make a popup window start here
//	timeline->presentedModalContent(this);
	//and then when you want to get rid of it call somewhere else
//	timeline->dismissedModalContent();
	//this will lock all other timeline interaction and feed all things into your track
	//so feel free to draw out of bounds, but don't go off the screen or out of the timeline
}


void ofxTLNodeTrack::willDeleteKeyframe(ofxTLKeyframe* keyframe){
	//do any cleanup before this keyframe gets hosed
}

NodeTrackEase ofxTLNodeTrack::getNextEase(NodeTrackEase ease){
    switch(ease){
        case OFXTL_NODE_EASE_CUT:
            return OFXTL_NODE_EASE_LINEAR;
        case OFXTL_NODE_EASE_LINEAR:
            return OFXTL_NODE_EASE_SMOOTH;
        case OFXTL_NODE_EASE_SMOOTH:
            return OFXTL_NODE_EASE_CUT;
    }
}

NodeTrackEase ofxTLNodeTrack::getPreviousEase(NodeTrackEase ease){
    switch(ease){
        case OFXTL_NODE_EASE_CUT:
            return OFXTL_NODE_EASE_SMOOTH;
        case OFXTL_NODE_EASE_LINEAR:
            return OFXTL_NODE_EASE_CUT;
        case OFXTL_NODE_EASE_SMOOTH:
            return OFXTL_NODE_EASE_LINEAR;
    }
}