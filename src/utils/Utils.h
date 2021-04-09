//
//  Utils.h
//  urModernDriverTest
//
//  Created by dantheman on 3/30/16.
//
//
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
// #pragma once
// /// \brief Converts a 3D point from millimeters to meters
// /// \param v ofVec3f to convert
// /// \return returns a copy of the point in meters
// ofVec3f toMeters(ofVec3f v){
//     return ofVec3f(v/ofVec3f(1000, 1000, 1000));
// }

// /// \brief Converts a 3D point from meters to millimeters
// /// \param v ofVec3f to convert
// /// \return copy of the point in millimeters
// ofVec3f toMM(ofVec3f v){
//     return ofVec3f(v*ofVec3f(1000, 1000, 1000));
// }

#pragma once
ofQuaternion convertAxisAngle(double rx, double ry, double rz) {
    float angle = ofVec3f(rx, ry, rz).normalize().length();
    double s = sin(angle/2);
    float x = (rx) * s;
    float y = (ry) * s;
    float z = (rz) * s;
    float w = cos(angle/2);
    return ofQuaternion(x, y, z, w);
}
