//
//  URDFLoader.h
//  ofxRobotArm//
//  Created by Dan Moore on 2/20/16.
//  Copyright (c) 2016,2020 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//



#pragma once
#include "ofxYAML.h"

namespace ofxRobotArm{
     class URDFLoader{
         public:
               URDFLoader();
               ~URDFLoader();
               void load(string path);
               ofxYAML yaml;
     }

     class INFOLoader{
          public:
               INFOLoader();
               ~INFOLoader();
               void load(string path);
               ofxYAML yaml;
               URDFLoader urdfLoader;
     }
}