/*
 *  ParallelTransportFrames.h
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
#pragma once

#include "ofMain.h"

namespace itg
{
    /*
     * PTF based on Cinder's implementation as edge cases are all handled 
     */
    class ParallelTransportFrames
    {
    public:
        // TODO: add startNormal
        
        ParallelTransportFrames();
        
        bool addPoint(const ofVec3f& point);
        void debugDraw(float axisSize = 10.f);
        
        glm::mat4x4 transformMatrix() const { return frames.back(); }
        glm::mat4x4 normalMatrix() const;
        
        unsigned framesSize() const { return frames.size(); }
        unsigned pointsSize() const { return points.size(); }
        
        deque<glm::mat4x4>& getFrames() { return frames; }
        
        glm::mat4 frameAt(unsigned idx) const { return frames[idx]; }
        
        glm::vec3 getStartNormal() const { return startNormal; }
        glm::vec3 getCurrentTangent() const { return curTangent; }
       
        glm::vec3 calcCurrentNormal() const;
        
        void clear();
        
        void setMaxFrames(unsigned maxFrames) { this->maxFrames = maxFrames; }
        
    private:
        unsigned maxPoints, maxFrames;
        
        void firstFrame();
        void nextFrame();
        
        glm::vec3 startNormal;
        glm::vec3 prevTangent, curTangent;
        
        deque<glm::vec3> points;
        deque<glm::mat4> frames;
    };
}
