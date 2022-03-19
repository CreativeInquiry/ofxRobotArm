/*
 *  ParallelTransportFrames.cpp
 *
 *  Copyright (c) 2012, Neil Mendoza, http://www.neilmendoza.com
 *  All rights reserved. 
 *  
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions are met: 
 *  
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of Neil Mendoza nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission. 
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 *  POSSIBILITY OF SUCH DAMAGE. 
 *
 */
#include "ParallelTransportFrames.h"

namespace ofxRobotArm
{
    ParallelTransportFrames::ParallelTransportFrames() :
        maxPoints(4), maxFrames(numeric_limits<unsigned>::max())
    {
        
    }
    
    bool ParallelTransportFrames::addPoint(const ofVec3f& point)
    {
        points.push_back(toGlm(point));
        while (points.size() > maxPoints) points.pop_front();
        if (points.size() == 3) firstFrame();
        else if (points.size() > 3)
        {
            nextFrame();
            return true;
        }
        return false;
    }
                          
    void ParallelTransportFrames::firstFrame()
    {
        glm::vec3 t = glm::normalize(points[1] - points[0]);

        glm::vec3 n = glm::normalize(glm::cross( t,  points[2] - points[0] ));
        
        if( n.length() == 0.0f )
        {
            int i = fabs( t[0] ) < fabs( t[1] ) ? 0 : 1;
            if( fabs( t[2] ) < fabs( t[i] ) ) i = 2;

            glm::vec3 v;
            v[i] = 1.f;
            n = glm::normalize(glm::cross(t, v));
        }

        glm::vec3 b = glm::cross(t, n );

        ofMatrix4x4 m(t[0], t[1], t[2], 0.0,
                      b[0], b[1], b[2], 0.0,
                      n[0], n[1], n[2], 0.0,
                      points[0][0], points[0][1], points[0][2], 1.f); 
            
        frames.push_back(toGlm(m));
        
        prevTangent = t;
        startNormal = n;
    }
    
    void ParallelTransportFrames::nextFrame()
    {
        curTangent = points.back() - points[points.size() - 2];
        glm::vec3 a;	// Rotation axis.
        float r = 0;						// Rotation angle.
        
        if( ( prevTangent.length() != 0.0 ) && ( curTangent.length() != 0.0 ) )
        {
            curTangent = glm::normalize(curTangent);
            float dot = glm::dot(prevTangent, curTangent);
            
            if( dot > 1.f ) dot = 1.f; 
            else if( dot < -1.0 ) dot = -1.0;
            
            r = acos( dot );
            a = glm::cross(prevTangent, curTangent );
        }
        
        if( ( a.length() != 0.0 ) && ( r != 0.0 ) )
        {
            glm::mat4 R = glm::rotate(glm::mat4(), ofRadToDeg(r), a);
            glm::mat4 Tj = glm::translate(glm::mat4(), points.back());
            glm::mat4 Ti = glm::translate(glm::mat4(), -points[points.size()-2]);
            
            frames.push_back(frames.back() * Ti * R * Tj);
        }
        else
        {
            glm::mat4 Tr = glm::translate(glm::mat4(),  points.back() - points[points.size() - 2]);
            frames.push_back(frames.back() * Tr);
        }
        prevTangent = curTangent;
        while (frames.size() > maxFrames) frames.pop_front();
    }
    
    glm::mat4 ParallelTransportFrames::normalMatrix() const
    {
        ofMatrix4x4 normalMatrix = glm::inverseTranspose(frames.back());
        return toGlm(ofMatrix4x4(normalMatrix(0, 0), normalMatrix(0, 1), normalMatrix(0, 2), 0.f,
                           normalMatrix(1, 0), normalMatrix(1, 1), normalMatrix(1, 2), 0.f,
                           normalMatrix(2, 0), normalMatrix(2, 1), normalMatrix(2, 2), 0.f,
                           0.f,                0.f,                0.f,                1.f));
    }
    
    glm::vec3 ParallelTransportFrames::calcCurrentNormal() const
    {
        return glm::vec3((normalMatrix() * glm::vec4(getStartNormal(), 0)));
    }
    
    void ParallelTransportFrames::debugDraw(float axisSize)
    {
        ofSetColor(0, 255, 255);
        ofNoFill();
        for (int i = 0; i < frames.size(); ++i)
        {
            ofPushMatrix();
            ofMultMatrix(frames[i]);
            ofRotateDeg(90, 0, 1, 0);
            ofDrawCircle(0, 0, axisSize * 2.f);
            ofDrawAxis(axisSize);
            ofPopMatrix();
        }
    }
    
    void ParallelTransportFrames::clear()
    {
        points.clear();
        frames.clear();
    }
}
