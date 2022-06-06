//
//  opw.cpp
//  example-ik
//
//  Created by Dan Moore on 6/5/22.
//

#include "opw.h"
using namespace ofxRobotArm;

void OPW::setup(vector<double> offsets,
           vector<double> sign_corrections,
           vector<double> joint_limit_min,
                vector<double> joint_limit_max){
    
}

void  OPW::setParams(vector<double> params){
    
    a1 = params[0];
    a2 = params[1];
    b  = params[2];
    c1 = params[3];
    c2 = params[4];
    c3 = params[5];
    c4 = params[6];
    
}
void  OPW::computeParams(RobotModel robot){
    
    
}

ofMatrix4x4 OPW::forward(vector<double> pose){
    
    ofMatrix4x4 sol;
    double q[pose.size()];
    for(int i = 0; i < pose.size(); i++){
        q[i] = pose[i]*sign_corrections[i] - offsets[0];
    }

    double psi3 = std::atan2(a2, c3);
    double k = std::sqrt(pow(a2, 2) + pow(c3, 2));

    double cx1 = c2 * std::sin(q[1]) + k * std::sin(q[1] + q[2] + psi3) + a1;
    double cy1 = b;
    double cz1 = c2 * std::cos(q[1]) + k * std::cos(q[1] + q[2] + psi3);

    double cx0 = cx1 * std::cos(q[0]) - cy1 * std::sin(q[0]);
    double cy0 = cx1 * std::sin(q[0]) + cy1 * std::cos(q[0]);
    double cz0 = cz1 + c1;

    double s1 = std::sin(q[0]);
    double s2 = std::sin(q[1]);
    double s3 = std::sin(q[2]);
    double s4 = std::sin(q[3]);
    double s5 = std::sin(q[4]);
    double s6 = std::sin(q[5]);

    double c1_2 = std::cos(q[0]);
    double c2_2 = std::cos(q[1]);
    double c3_2 = std::cos(q[2]);
    double c4_2 = std::cos(q[3]);
    double c5_2 = std::cos(q[4]);
    double c6_2 = std::cos(q[5]);

    ofMatrix4x4 r_0c;
    r_0c.set(c1_2 * c2_2 * c3_2 - c1_2 * s2 * s3, -s1, c1_2 * c2_2 * s3 + c1_2 * s2 * c3_2, 0,
             s1 * c2_2 * c3_2 - s1 * s2 * s3, c1_2, s1 * c2_2 * s3 + s1 * s2 * c3_2, 0,
             -s2 * c3_2 - c2_2 * s3, 0, -s2 * s3 + c2_2 * c3_2, 0,
             0, 0, 0, 1);

    ofMatrix4x4 r_ce;
    r_ce.set(c4_2 * c5_2 * c6_2 - s4 * s6, -c4_2 * c5_2 * s6 - s4 * c6_2, c4_2 * s5, 0,
             s4 * c5_2 * c6_2 + c4_2 * s6, -s4 * c5_2 * s6 + c4_2 * c6_2, s4 * s5, 0,
             -s5 * c6_2, s5 * s6, c5_2, 0,
             0, 0, 0, 1);

    ofMatrix4x4 r_oe = r_0c * r_ce;

    ofVec3f u = ofVec3f(cx0, cy0, cz0) + r_oe * ofVec3f(0, 0, 1) * c4;
    ofMatrix4x4 mat;
    mat.makeTranslationMatrix(u);
    sol = r_oe * mat;

    return sol;
}


vector<vector<double> > OPW::inverse(ofMatrix4x4 pose){
    double sol[8 * 6];
    vector<vector<double> > sols;
    ofVec3f c = pose.getTranslation() - ofMatrix4x4::transform3x3(pose, ofVec3f(0, 0, 1)) * c4;
    double nx1 = std::sqrt(pow(c.x, 2) + pow(c.y, 2) - pow(b, 2)) - a1;

    // Compute theta1_i, theta1_ii
    double tmp1 = std::atan2(c.y, c.x);
    double tmp2 = std::atan2(b, nx1 + a1);
    double theta1_i = tmp1 - tmp2;
    double theta1_ii = tmp1 + tmp2 - PI;
    double tmp3 = (c.z - c1);
    double s1_2 = pow(nx1, 2) + pow(tmp3, 2);

    double tmp4 = nx1 + 2 * a1;
    double s2_2 = pow(tmp4, 2) + pow(tmp3, 2);
    double kappa_2 = pow(a2, 2) + pow(c3, 2);

    double c2_2 = c2 * c2;

    double tmp5 = s1_2 + c2_2 - kappa_2;

    double s1 = std::sqrt(s1_2);
    double s2 = std::sqrt(s2_2);
    double theta2_i = -std::acos(tmp5 / (2.0 * s1 * c2)) + std::atan2(nx1, c.z - c1);
    double theta2_ii = std::acos(tmp5 / (2.0 * s1 * c2)) + std::atan2(nx1, c.z - c1);

    double tmp6 = s2_2 + c2_2 - kappa_2;

    double theta2_iii = -std::acos(tmp6 / (2.0 * s2 * c2)) - std::atan2(nx1 + 2.0 * a1, c.z - c1);
    double theta2_iv = std::acos(tmp6 / (2.0 * s2 * c2)) - std::atan2(nx1 + 2.0 * a1, c.z - c1);

    // theta3
    double tmp7 = s1_2 - c2_2 - kappa_2;
    double tmp8 = s2_2 - c2_2 - kappa_2;
    double tmp9 = 2 * c2 * std::sqrt(kappa_2);
    double theta3_i = std::acos(tmp7 / tmp9) - std::atan2(a2, c3);

    double theta3_ii = -std::acos(tmp7 / tmp9) - std::atan2(a2, c3);

    double theta3_iii = std::acos(tmp8 / tmp9) - std::atan2(a2, c3);
    double theta3_iv = -std::acos(tmp8 / tmp9) - std::atan2(a2, c3);

    // Now for the orientation part...
    double s23[4];
    double c23[4];
    double sin1[4];
    double cos1[4];

    sin1[0] = std::sin(theta1_i);
    sin1[1] = std::sin(theta1_i);
    sin1[2] = std::sin(theta1_ii); // ???
    sin1[3] = std::sin(theta1_ii);

    cos1[0] = std::cos(theta1_i);
    cos1[1] = std::cos(theta1_i);
    cos1[2] = std::cos(theta1_ii); // ???
    cos1[3] = std::cos(theta1_ii);

    s23[0] = std::sin(theta2_i + theta3_i);
    s23[1] = std::sin(theta2_ii + theta3_ii);
    s23[2] = std::sin(theta2_iii + theta3_iii);
    s23[3] = std::sin(theta2_iv + theta3_iv);

    c23[0] = std::cos(theta2_i + theta3_i);
    c23[1] = std::cos(theta2_ii + theta3_ii);
    c23[2] = std::cos(theta2_iii + theta3_iii);
    c23[3] = std::cos(theta2_iv + theta3_iv);

    double m[4];
    m[0] = get(pose, 0, 2) * s23[0] * cos1[0] + get(pose, 1, 2) * s23[0] * sin1[0] + get(pose, 2, 2) * c23[0];
    m[1] = get(pose, 0, 2) * s23[1] * cos1[1] + get(pose, 1, 2) * s23[1] * sin1[1] + get(pose, 2, 2) * c23[1];
    m[2] = get(pose, 0, 2) * s23[2] * cos1[2] + get(pose, 1, 2) * s23[2] * sin1[2] + get(pose, 2, 2) * c23[2];
    m[3] = get(pose, 0, 2) * s23[3] * cos1[3] + get(pose, 1, 2) * s23[3] * sin1[3] + get(pose, 2, 2) * c23[3];

    double theta4_i = std::atan2(get(pose, 1, 2) * cos1[0] - get(pose, 0, 2) * sin1[0],
                                 get(pose, 0, 2) * c23[0] * cos1[0] + get(pose, 1, 2) * c23[0] * sin1[0] - get(pose, 2, 2) * s23[0]);

    double theta4_ii = std::atan2(get(pose, 1, 2) * cos1[1] - get(pose, 0, 2) * sin1[1],
                                  get(pose, 0, 2) * c23[1] * cos1[1] + get(pose, 1, 2) * c23[1] * sin1[1] - get(pose, 2, 2) * s23[1]);

    double theta4_iii = std::atan2(get(pose, 1, 2) * cos1[2] - get(pose, 0, 2) * sin1[2],
                                   get(pose, 0, 2) * c23[2] * cos1[2] + get(pose, 1, 2) * c23[2] * sin1[2] - get(pose, 2, 2) * s23[2]);

    double theta4_iv = std::atan2(get(pose, 1, 2) * cos1[3] - get(pose, 0, 2) * sin1[3],
                                  get(pose, 0, 2) * c23[3] * cos1[3] + get(pose, 1, 2) * c23[3] * sin1[3] - get(pose, 2, 2) * s23[3]);

    double theta4_v = theta4_i + PI;
    double theta4_vi = theta4_ii + PI;
    double theta4_vii = theta4_iii + PI;
    double theta4_viii = theta4_iv + PI;

    double theta5_i = std::atan2(std::sqrt(1 - m[0] * m[0]), m[0]);
    double theta5_ii = std::atan2(std::sqrt(1 - m[1] * m[1]), m[1]);
    double theta5_iii = std::atan2(std::sqrt(1 - m[2] * m[2]), m[2]);
    double theta5_iv = std::atan2(std::sqrt(1 - m[3] * m[3]), m[3]);

    double theta5_v = -theta5_i;
    double theta5_vi = -theta5_ii;
    double theta5_vii = -theta5_iii;
    double theta5_viii = -theta5_iv;
    
    double zero_threshold = 1e-6;
    double theta6_i = 0;
    if (std::abs(theta5_i) < zero_threshold)
    {
        theta4_i = 0;
        glm::vec3 xe(get(pose, 0, 0), get(pose, 1, 0), get(pose, 2, 0));
        glm::vec3 col1 = glm::vec3(-std::sin(theta1_i), std::cos(theta1_i), 0);  // yc
        glm::vec3 col2  = glm::vec3(get(pose, 0, 2), get(pose, 1, 2), get(pose, 2, 2)); // zc and ze are equal
        glm::vec3 col3 = glm::cross(col1, col2);// xc
        glm::mat3x3 Rc = glm::mat3x3(col1.x, col1.y, col1.z, col2.x, col2.y, col2.z, col3.x, col3.y, col3.z);
        glm::vec3 xec = glm::transpose(Rc) * xe;
        theta6_i = std::atan2(xec.y, xec.x);
    }else{
       double theta4_iy = get(pose, 1, 2) * cos1[0] - get(pose, 0, 2) * sin1[0];
       double theta4_ix = get(pose, 0, 2) * c23[0] * cos1[0] + get(pose, 1, 2) * c23[0] * sin1[0] - get(pose, 2, 2) * s23[0];
       theta4_i = std::atan2(theta4_iy, theta4_ix);

       double theta6_iy = get(pose, 0, 1) * s23[0] * cos1[0] + get(pose, 1, 1) * s23[0] * sin1[0] + get(pose, 2, 1) * c23[0];
       double theta6_ix = -get(pose, 0, 0) * s23[0] * cos1[0] - get(pose, 1, 0) * s23[0] * sin1[0] - get(pose, 2, 0) * c23[0];
       theta6_i = std::atan2(theta6_iy, theta6_ix);
    }
    
    double theta6_ii;
    if (std::abs(theta5_ii) < zero_threshold)
    {
        theta4_ii = 0;
        glm::vec3 xe(get(pose, 0, 0), get(pose, 1, 0), get(pose, 2, 0));
        glm::vec3 col1 = glm::vec3(-std::sin(theta1_i), std::cos(theta1_i), 0);  // yc
        glm::vec3 col2  = glm::vec3(get(pose, 0, 2), get(pose, 1, 2), get(pose, 2, 2)); // zc and ze are equal
        glm::vec3 col3 = glm::cross(col1, col2);// xc
        glm::mat3x3 Rc = glm::mat3x3(col1.x, col1.y, col1.z, col2.x, col2.y, col2.z, col3.x, col3.y, col3.z);
        glm::vec3 xec = glm::transpose(Rc) * xe;
        theta6_ii = std::atan2(xec.y, xec.x);
    }
    else
    {
        double theta4_iiy = get(pose, 1, 2) * cos1[1] - get(pose, 0, 2) * sin1[1];
        double theta4_iix = get(pose, 0, 2) * c23[1] * cos1[1] + get(pose,1, 2) * c23[1] * sin1[1] - get(pose, 2, 2) * s23[1];
        theta4_ii = std::atan2(theta4_iiy, theta4_iix);

        double theta6_iiy = get(pose, 0, 1) * s23[1] * cos1[1] + get(pose, 1, 1) * s23[1] * sin1[1] + get(pose, 2, 1) * c23[1];
        double theta6_iix = -get(pose, 0, 0) * s23[1] * cos1[1] - get(pose, 1, 0) * s23[1] * sin1[1] - get(pose, 2, 0) * c23[1];
        theta6_ii = std::atan2(theta6_iiy, theta6_iix);
    }

    double theta6_iii;
    if (std::abs(theta5_iii) < zero_threshold)
    {
        theta4_iii = 0;
        glm::vec3 xe(get(pose, 0, 0), get(pose, 1, 0), get(pose, 2, 0));
        glm::vec3 col1 = glm::vec3(-std::sin(theta1_ii), std::cos(theta1_ii), 0);  // yc
        glm::vec3 col2  = glm::vec3(get(pose, 0, 2), get(pose, 1, 2), get(pose, 2, 2)); // zc and ze are equal
        glm::vec3 col3 = glm::cross(col1, col2);// xc
        glm::mat3x3 Rc = glm::mat3x3(col1.x, col1.y, col1.z, col2.x, col2.y, col2.z, col3.x, col3.y, col3.z);
        glm::vec3 xec = glm::transpose(Rc) * xe;
        theta6_iii = std::atan2(xec.y, xec.x);
    }
    else
    {
        double theta4_iiiy = get(pose, 1, 2) * cos1[1] - get(pose, 0, 2) * sin1[1];
        double theta4_iiix = get(pose, 0, 2) * c23[1] * cos1[1] + get(pose,1, 2) * c23[1] * sin1[1] - get(pose, 2, 2) * s23[1];
        theta4_iii = std::atan2(theta4_iiiy, theta4_iiix);

        double theta6_iiiy = get(pose, 0, 1) * s23[1] * cos1[1] + get(pose, 1, 1) * s23[1] * sin1[1] + get(pose, 2, 1) * c23[1];
        double theta6_iiix = -get(pose, 0, 0) * s23[1] * cos1[1] - get(pose, 1, 0) * s23[1] * sin1[1] - get(pose, 2, 0) * c23[1];
        theta6_iii = std::atan2(theta6_iiiy, theta6_iiix);
    }

    
    double theta6_iv;
    if (std::abs(theta5_iv) < zero_threshold)
    {
        theta4_iv = 0;
        glm::vec3 xe(get(pose, 0, 0), get(pose, 1, 0), get(pose, 2, 0));
        glm::vec3 col1 = glm::vec3(-std::sin(theta1_ii), std::cos(theta1_ii), 0);  // yc
        glm::vec3 col2  = glm::vec3(get(pose, 0, 2), get(pose, 1, 2), get(pose, 2, 2)); // zc and ze are equal
        glm::vec3 col3 = glm::cross(col1, col2);// xc
        glm::mat3x3 Rc = glm::mat3x3(col1.x, col1.y, col1.z, col2.x, col2.y, col2.z, col3.x, col3.y, col3.z);
        glm::vec3 xec = glm::transpose(Rc) * xe;
        theta6_iv = std::atan2(xec.y, xec.x);
    }
    else
    {
        double theta4_ivy = get(pose, 1, 2) * cos1[1] - get(pose, 0, 2) * sin1[1];
        double theta4_ivx = get(pose, 0, 2) * c23[1] * cos1[1] + get(pose,1, 2) * c23[1] * sin1[1] - get(pose, 2, 2) * s23[1];
        theta4_iv = std::atan2(theta4_ivy, theta4_ivx);

        double theta6_ivy = get(pose, 0, 1) * s23[1] * cos1[1] + get(pose, 1, 1) * s23[1] * sin1[1] + get(pose, 2, 1) * c23[1];
        double theta6_ivx = -get(pose, 0, 0) * s23[1] * cos1[1] - get(pose, 1, 0) * s23[1] * sin1[1] - get(pose, 2, 0) * c23[1];
        theta6_iv = std::atan2(theta6_ivy, theta6_ivx);
    }
    
    
    theta4_v = theta4_i + PI;
    theta4_vi = theta4_ii + PI;
    theta4_vii = theta4_iii + PI;
    theta4_viii = theta4_iv + PI;

    double theta6_v = theta6_i - PI;
    double theta6_vi = theta6_ii - PI;
    double theta6_vii = theta6_iii - PI;
    double theta6_viii = theta6_iv - PI;

    sol[6 * 0 + 0] = (theta1_i + offsets[0]) * sign_corrections[0];
    sol[6 * 0 + 1] = (theta2_i + offsets[1]) * sign_corrections[1];
    sol[6 * 0 + 2] = (theta3_i + offsets[2]) * sign_corrections[2];
    sol[6 * 0 + 3] = (theta4_i + offsets[3]) * sign_corrections[3];
    sol[6 * 0 + 4] = (theta5_i + offsets[4]) * sign_corrections[4];
    sol[6 * 0 + 5] = (theta6_i + offsets[5]) * sign_corrections[5];

    sol[6 * 1 + 0] = (theta1_i + offsets[0]) * sign_corrections[0];
    sol[6 * 1 + 1] = (theta2_ii + offsets[1]) * sign_corrections[1];
    sol[6 * 1 + 2] = (theta3_ii + offsets[2]) * sign_corrections[2];
    sol[6 * 1 + 3] = (theta4_ii + offsets[3]) * sign_corrections[3];
    sol[6 * 1 + 4] = (theta5_ii + offsets[4]) * sign_corrections[4];
    sol[6 * 1 + 5] = (theta6_ii + offsets[5]) * sign_corrections[5];

    sol[6 * 2 + 0] = (theta1_ii + offsets[0]) * sign_corrections[0];
    sol[6 * 2 + 1] = (theta2_iii + offsets[1]) * sign_corrections[1];
    sol[6 * 2 + 2] = (theta3_iii + offsets[2]) * sign_corrections[2];
    sol[6 * 2 + 3] = (theta4_iii + offsets[3]) * sign_corrections[3];
    sol[6 * 2 + 4] = (theta5_iii + offsets[4]) * sign_corrections[4];
    sol[6 * 2 + 5] = (theta6_iii + offsets[5]) * sign_corrections[5];

    sol[6 * 3 + 0] = (theta1_ii + offsets[0]) * sign_corrections[0];
    sol[6 * 3 + 1] = (theta2_iv + offsets[1]) * sign_corrections[1];
    sol[6 * 3 + 2] = (theta3_iv + offsets[2]) * sign_corrections[2];
    sol[6 * 3 + 3] = (theta4_iv + offsets[3]) * sign_corrections[3];
    sol[6 * 3 + 4] = (theta5_iv + offsets[4]) * sign_corrections[4];
    sol[6 * 3 + 5] = (theta6_iv + offsets[5]) * sign_corrections[5];

    sol[6 * 4 + 0] = (theta1_i + offsets[0]) * sign_corrections[0];
    sol[6 * 4 + 1] = (theta2_i + offsets[1]) * sign_corrections[1];
    sol[6 * 4 + 2] = (theta3_i + offsets[2]) * sign_corrections[2];
    sol[6 * 4 + 3] = (theta4_v + offsets[3]) * sign_corrections[3];
    sol[6 * 4 + 4] = (theta5_v + offsets[4]) * sign_corrections[4];
    sol[6 * 4 + 5] = (theta6_v + offsets[5]) * sign_corrections[5];

    sol[6 * 5 + 0] = (theta1_i + offsets[0]) * sign_corrections[0];
    sol[6 * 5 + 1] = (theta2_ii + offsets[1]) * sign_corrections[1];
    sol[6 * 5 + 2] = (theta3_ii + offsets[2]) * sign_corrections[2];
    sol[6 * 5 + 3] = (theta4_vi + offsets[3]) * sign_corrections[3];
    sol[6 * 5 + 4] = (theta5_vi + offsets[4]) * sign_corrections[4];
    sol[6 * 5 + 5] = (theta6_vi + offsets[5]) * sign_corrections[5];

    sol[6 * 6 + 0] = (theta1_ii + offsets[0]) * sign_corrections[0];
    sol[6 * 6 + 1] = (theta2_iii + offsets[1]) * sign_corrections[1];
    sol[6 * 6 + 2] = (theta3_iii + offsets[2]) * sign_corrections[2];
    sol[6 * 6 + 3] = (theta4_vii + offsets[3]) * sign_corrections[3];
    sol[6 * 6 + 4] = (theta5_vii + offsets[4]) * sign_corrections[4];
    sol[6 * 6 + 5] = (theta6_vii + offsets[5]) * sign_corrections[5];

    sol[6 * 7 + 0] = (theta1_ii + offsets[0]) * sign_corrections[0];
    sol[6 * 7 + 1] = (theta2_iv + offsets[1]) * sign_corrections[1];
    sol[6 * 7 + 2] = (theta3_iv + offsets[2]) * sign_corrections[2];
    sol[6 * 7 + 3] = (theta4_viii + offsets[3]) * sign_corrections[3];
    sol[6 * 7 + 4] = (theta5_viii + offsets[4]) * sign_corrections[4];
    sol[6 * 7 + 5] = (theta6_viii + offsets[5]) * sign_corrections[5];
    

    for (int i = 0; i < 8; i++)
    {
        vector<double> fooSol;
        sol[6 * i + 0] = ofClamp(sol[6 * i + 0], ofDegToRad(joint_limit_min[0]) * sign_corrections[0] , ofDegToRad(joint_limit_max[0]) * sign_corrections[0] );
        sol[6 * i + 1] = ofClamp(sol[6 * i + 1], ofDegToRad(joint_limit_min[1]) * sign_corrections[1] , ofDegToRad(joint_limit_max[1]) * sign_corrections[1] );
        sol[6 * i + 2] = ofClamp(sol[6 * i + 2], ofDegToRad(joint_limit_min[2]) * sign_corrections[2] , ofDegToRad(joint_limit_max[2]) * sign_corrections[2] );
        sol[6 * i + 3] = ofClamp(sol[6 * i + 3], ofDegToRad(joint_limit_min[3]) * sign_corrections[3] , ofDegToRad(joint_limit_max[3]) * sign_corrections[3] );
        sol[6 * i + 4] = ofClamp(sol[6 * i + 4], ofDegToRad(joint_limit_min[4]) * sign_corrections[4] , ofDegToRad(joint_limit_max[4]) * sign_corrections[4] );
        sol[6 * i + 5] = ofClamp(sol[6 * i + 5], ofDegToRad(joint_limit_min[5]) * sign_corrections[5] , ofDegToRad(joint_limit_max[5]) * sign_corrections[5] );
        
        fooSol.push_back(sol[6 * i + 0]);
        fooSol.push_back(sol[6 * i + 1]);
        fooSol.push_back(sol[6 * i + 2]);
        fooSol.push_back(sol[6 * i + 3]);
        fooSol.push_back(sol[6 * i + 4]);
        fooSol.push_back(sol[6 * i + 5]);
        
        if (isValid(fooSol))
        {
            harmonizeTowardZero(fooSol);
            sols.push_back(fooSol);
        }
    }

    return sols;
}
