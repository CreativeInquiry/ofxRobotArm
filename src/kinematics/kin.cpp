#include "kin.h"




using namespace ofxRobotArm;
const static double ANGLE_THRESH = ofDegToRad(30);
const double ZERO_THRESH = 0.00000001;
int SIGN(double x) {
    return (x > 0) - (x < 0);
}

//#define UR10_PARAMS
//#ifdef UR10_PARAMS
//const double d1 =  0.1273;
//const double a2_2 = -0.612;
//const double a3 = -0.5723;
//const double d4 =  0.163941;
//const double d5 =  0.1157;
//const double d6 =  0.0922;
//#endif
//
////#define UR5_PARAMS
//#ifdef UR5_PARAMS
//const double d1 =  0.089159;
//const double a2_2 = -0.42500;
//const double a3 = -0.39225;
//const double d4 =  0.10915;
//const double d5 =  0.09465;
//const double d6 =  0.0823;
//#endif
//
////#define UR3_PARAMS
//#ifdef UR3_PARAMS
//const double d1 =  0.1519;
//const double a2_2 = -0.24365;
//const double a3 = -0.21325;
//const double d4 =  0.11235;
//const double d5 =  0.08535;
//const double d6 =  0.0819;
//#endif

Kinematics::Kinematics(ofxRobotArm::RobotType type){
    
    offsets.assign(6, 0);
    sign_corrections.assign(6, 1);
    joint_limit_min.assign(6, 0);
    joint_limit_max.assign(6, 0);
    this->type = type;
    if (type == UR3){
        d1 =  0.1519;
        a2 = -0.24365;
        a3 = -0.21325;
        d4 =  0.11235;
        d5 =  0.08535;
        d6 =  0.0819;
    }
    else if (type == UR5){
        d1 =  0.089159;
        a2 = -0.42500;
        a3 = -0.39225;
        d4 =  0.10915;
        d5 =  0.09465;
        d6 =  0.0823;
    }
    else if (type == UR10){
        d1 =  0.1273;
        a2 = -0.612;
        a3 = -0.5723;
        d4 =  0.163941;
        d5 =  0.1157;
        d6 =  0.0922;
    }
    else if (type == IRB120){
        d1 =  0.290;
        a2 =  0.270;
        a3 =  -0.070;
        d4 =  0.302;
        d5 =  0.0;
        d6 =  0.072;
        
        a1 = 0; //a1
        a2_2 = -0.070; //a2_2 Different a2
        b  = 0; //b
        c1 = 0.270; //c1
        c2 = 0.290; //c2
        c3 = 0.302; //c3
        c4 = 0.072; //c4

        offsets[2] = -PI/2;
        offsets[3] = -PI;
        offsets[4] = -PI/2;
        sign_corrections[3] = -1;
        
        joint_limit_min[0] = -165;
        joint_limit_min[1] = -110;
        joint_limit_min[2] = -110;
        joint_limit_min[3] = -160;
        joint_limit_min[4] = -120;
        joint_limit_min[5] = -400;
        
        joint_limit_max[0] = 165;
        joint_limit_max[1] = 110;
        joint_limit_max[2] = 70;
        joint_limit_max[3] = 160;
        joint_limit_max[4] = 120;
        joint_limit_max[5] = 400;
    }
}

Kinematics::Kinematics(){
    
}
Kinematics::~Kinematics(){
    
}

void Kinematics::forwardHK(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
    double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
    *T = c234*c1*s5 - c5*s1; T++;
    *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
    *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
    *T = d6*c234*c1*s5 - a3*c23*c1 - a2_2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
    *T = c1*c5 + c234*s1*s5; T++;
    *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
    *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
    *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
    *T = -s234*s5; T++;
    *T = -c234*s6 - s234*c5*c6; T++;
    *T = s234*c5*s6 - c234*c6; T++;
    *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
}

void Kinematics::forward_allHK(const double* q, double* T1, double* T2, double* T3,
                               double* T4, double* T5, double* T6) {
    double s1 = sin(*q), c1 = cos(*q); q++; // q1
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
    q234 += *q; q++; // q4
    double s5 = sin(*q), c5 = cos(*q); q++; // q5
    double s6 = sin(*q), c6 = cos(*q); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
    
    if(T1 != NULL) {
        *T1 = c1; T1++;
        *T1 = 0; T1++;
        *T1 = s1; T1++;
        *T1 = 0; T1++;
        *T1 = s1; T1++;
        *T1 = 0; T1++;
        *T1 = -c1; T1++;
        *T1 = 0; T1++;
        *T1 =       0; T1++;
        *T1 = 1; T1++;
        *T1 = 0; T1++;
        *T1 =d1; T1++;
        *T1 =       0; T1++;
        *T1 = 0; T1++;
        *T1 = 0; T1++;
        *T1 = 1; T1++;
    }
    
    if(T2 != NULL) {
        *T2 = c1*c2; T2++;
        *T2 = -c1*s2; T2++;
        *T2 = s1; T2++;
        *T2 =a2*c1*c2; T2++;
        *T2 = c2*s1; T2++;
        *T2 = -s1*s2; T2++;
        *T2 = -c1; T2++;
        *T2 =a2*c2*s1; T2++;
        *T2 =         s2; T2++;
        *T2 = c2; T2++;
        *T2 = 0; T2++;
        *T2 =   d1 + a2*s2; T2++;
        *T2 =               0; T2++;
        *T2 = 0; T2++;
        *T2 = 0; T2++;
        *T2 =                 1; T2++;
    }
    
    if(T3 != NULL) {
        *T3 = c23*c1; T3++;
        *T3 = -s23*c1; T3++;
        *T3 = s1; T3++;
        *T3 =c1*(a3*c23 + a2*c2); T3++;
        *T3 = c23*s1; T3++;
        *T3 = -s23*s1; T3++;
        *T3 = -c1; T3++;
        *T3 =s1*(a3*c23 + a2*c2); T3++;
        *T3 =         s23; T3++;
        *T3 = c23; T3++;
        *T3 = 0; T3++;
        *T3 =     d1 + a3*s23 + a2*s2; T3++;
        *T3 =                    0; T3++;
        *T3 = 0; T3++;
        *T3 = 0; T3++;
        *T3 =                                     1; T3++;
    }
    
    if(T4 != NULL) {
        *T4 = c234*c1; T4++;
        *T4 = s1; T4++;
        *T4 = s234*c1; T4++;
        *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
        *T4 = c234*s1; T4++;
        *T4 = -c1; T4++;
        *T4 = s234*s1; T4++;
        *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
        *T4 =         s234; T4++;
        *T4 = 0; T4++;
        *T4 = -c234; T4++;
        *T4 =                  d1 + a3*s23 + a2*s2; T4++;
        *T4 =                         0; T4++;
        *T4 = 0; T4++;
        *T4 = 0; T4++;
        *T4 =                                                  1; T4++;
    }
    
    if(T5 != NULL) {
        *T5 = s1*s5 + c234*c1*c5; T5++;
        *T5 = -s234*c1; T5++;
        *T5 = c5*s1 - c234*c1*s5; T5++;
        *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
        *T5 = c234*c5*s1 - c1*s5; T5++;
        *T5 = -s234*s1; T5++;
        *T5 = - c1*c5 - c234*s1*s5; T5++;
        *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
        *T5 =                           s234*c5; T5++;
        *T5 = c234; T5++;
        *T5 = -s234*s5; T5++;
        *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
        *T5 =                                                   0; T5++;
        *T5 = 0; T5++;
        *T5 = 0; T5++;
        *T5 =                                                                                 1; T5++;
    }
    
    if(T6 != NULL) {
        *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
        *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
        *T6 = c5*s1 - c234*c1*s5; T6++;
        *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
        *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
        *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
        *T6 = - c1*c5 - c234*s1*s5; T6++;
        *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
        *T6 =                                       c234*s6 + s234*c5*c6; T6++;
        *T6 = c234*c6 - s234*c5*s6; T6++;
        *T6 = -s234*s5; T6++;
        *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
        *T6 =                                                                                                   0; T6++;
        *T6 = 0; T6++;
        *T6 = 0; T6++;
        *T6 =                                                                                                                                            1; T6++;
    }
}

int Kinematics::inverseHK(const double* T, double* q_sols, double q6_des) {
    int num_sols = 0;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++;
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++;
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;
    
    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
        double A = d6*T12 - T13;
        double B = d6*T02 - T03;
        double R = A*A + B*B;
        if(fabs(A) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
                div = -SIGN(d4)*SIGN(B);
            else
                div = -d4/B;
            double arcsin = asin(div);
            if(fabs(arcsin) < ZERO_THRESH)
                arcsin = 0.0;
            if(arcsin < 0.0)
                q1[0] = arcsin + 2.0*PI;
            else
                q1[0] = arcsin;
            q1[1] = PI - arcsin;
        }
        else if(fabs(B) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
                div = SIGN(d4)*SIGN(A);
            else
                div = d4/A;
            double arccos = acos(div);
            q1[0] = arccos;
            q1[1] = 2.0*PI - arccos;
        }
        else if(d4*d4 > R) {
            return num_sols;
        }
        else {
            double arccos = acos(d4 / sqrt(R)) ;
            double arctan = atan2(-B, A);
            double pos = arccos + arctan;
            double neg = -arccos + arctan;
            if(fabs(pos) < ZERO_THRESH)
                pos = 0.0;
            if(fabs(neg) < ZERO_THRESH)
                neg = 0.0;
            if(pos >= 0.0)
                q1[0] = pos;
            else
                q1[0] = 2.0*PI + pos;
            if(neg >= 0.0)
                q1[1] = neg;
            else
                q1[1] = 2.0*PI + neg;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    
    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
        for(int i=0;i<2;i++) {
            double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
            double div;
            if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
                div = SIGN(numer) * SIGN(d6);
            else
                div = numer / d6;
            double arccos = acos(div);
            q5[i][0] = arccos;
            q5[i][1] = 2.0*PI - arccos;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    
    {
        for(int i=0;i<2;i++) {
            for(int j=0;j<2;j++) {
                double c1 = cos(q1[i]), s1 = sin(q1[i]);
                double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                double q6;
                ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                if(fabs(s5) < ZERO_THRESH)
                    q6 = q6_des;
                else {
                    q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
                               SIGN(s5)*(T00*s1 - T10*c1));
                    if(fabs(q6) < ZERO_THRESH)
                        q6 = 0.0;
                    if(q6 < 0.0)
                        q6 += 2.0*PI;
                }
                ////////////////////////////////////////////////////////////////////////////////
                
                double q2[2], q3[2], q4[2];
                ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                double c6 = cos(q6), s6 = sin(q6);
                double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
                double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
                double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
                T03*c1 + T13*s1;
                double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);
                
                double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
                if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
                    c3 = SIGN(c3);
                else if(fabs(c3) > 1.0) {
                    // TODO NO SOLUTION
                    continue;
                }
                double arccos = acos(c3);
                q3[0] = arccos;
                q3[1] = 2.0*PI - arccos;
                double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
                double s3 = sin(arccos);
                double A = (a2 + a3*c3), B = a3*s3;
                q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
                q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
                double c23_0 = cos(q2[0]+q3[0]);
                double s23_0 = sin(q2[0]+q3[0]);
                double c23_1 = cos(q2[1]+q3[1]);
                double s23_1 = sin(q2[1]+q3[1]);
                q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
                q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
                ////////////////////////////////////////////////////////////////////////////////
                for(int k=0;k<2;k++) {
                    if(fabs(q2[k]) < ZERO_THRESH)
                        q2[k] = 0.0;
                    else if(q2[k] < 0.0) q2[k] += 2.0*PI;
                    if(fabs(q4[k]) < ZERO_THRESH)
                        q4[k] = 0.0;
                    else if(q4[k] < 0.0) q4[k] += 2.0*PI;
                    q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k];
                    q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k];
                    q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6;
                    num_sols++;
                }
                
            }
        }
    }
    return num_sols;
}


// ----------------------------------------------------------
void Kinematics::setParams(float _a1, float _a2, float _b, float _c1, float _c2, float _c3, float _c4) {
    
    a1 = _a1;
    a2_2 = _a2;
    b = _b;
    c1 = _c1;
    c2 = _c2;
    c3 = _c3;
    c4 = _c4;
}

void Kinematics::setDH(float d1, float a2, float a3, float d4, float d5, float d6) {
    
    this->d1 = d1;
    this->a2 = a2;
    this->a3 = a3;
    this->d4 = d4;
    this->d5 = d5;
    this->d6 = d6;
}

// ----------------------------------------------------------
void Kinematics::forwardSW(double t1, double t2, double t3, double t4, double t5, double t6, ofMatrix4x4& sol) {
    
    double q[6];
    q[0] = t1 * sign_corrections[0] - offsets[0];
    q[1] = t2 * sign_corrections[1] - offsets[1];
    q[2] = t3 * sign_corrections[2] - offsets[2];
    q[3] = t4 * sign_corrections[3] - offsets[3];
    q[4] = t4 * sign_corrections[4] - offsets[4];
    q[5] = t5 * sign_corrections[5] - offsets[5];
    
    double psi3 = std::atan2(a2_2,c3);
    double k = std::sqrt(a2_2 *a2_2 +c3 *c3);
    
    double cx1 =c2 * std::sin(q[1]) + k * std::sin(q[1] + q[2] + psi3) +a1;
    double cy1 =b;
    double cz1 =c2 * std::cos(q[1]) + k * std::cos(q[1] + q[2] + psi3);
    
    double cx0 = cx1 * std::cos(q[0]) - cy1 * std::sin(q[0]);
    double cy0 = cx1 * std::sin(q[0]) + cy1 * std::cos(q[0]);
    double cz0 = cz1 +c1;
    
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
             s1 * c2_2 * c3_2 - s1 * s2 * s3, c1_2,  s1 * c2_2 * s3 + s1 * s2 * c3_2, 0,
             -s2 * c3_2 - c2_2 * s3, 0,  -s2 * s3 + c2_2 * c3_2, 0,
             0, 0, 0, 1);
    
    
    ofMatrix4x4 r_ce;
    r_ce.set(c4_2 * c5_2 * c6_2 - s4 * s6, -c4_2 * c5_2 * s6 - s4 * c6_2, c4_2 * s5, 0,
             s4 * c5_2 * c6_2 + c4_2 * s6, -s4 * c5_2 * s6 + c4_2 * c6_2, s4 * s5, 0,
             -s5 * c6_2, s5 * s6, c5_2, 0,
             0, 0, 0, 1);
    
    
    ofMatrix4x4 r_oe = r_0c * r_ce;

    
    ofVec3f u = ofVec3f(cx0, cy0, cz0) + c4 * ofVec3f(0, 0, 1);
    ofMatrix4x4 mat;
    mat.makeTranslationMatrix(u);
    sol = mat;
}



void Kinematics::inverseSW(ofMatrix4x4 pose, double * sol)
{


    ofVec3f c = pose.getTranslation() - c4 * ofVec3f(0, 0, 1);
    double nx1 = std::sqrt(c.x * c.x + c.y * c.y - b * b) - a1;

    // Compute theta1_i, theta1_ii
    double tmp1 = std::atan2(c.y, c.x);
    double tmp2 = std::atan2(b, nx1 + a1);
    double theta1_i = tmp1 - tmp2;
    double theta1_ii = tmp1 + tmp2 - PI;

    // theta2 i through iv
    double tmp3 = (c.z - c1);
    double s1_2 = nx1 * nx1 + tmp3 * tmp3;

    double tmp4 = nx1 + 2.0 * a1;
    double s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
    double kappa_2 = a2_2 * a2_2 + c3 * c3;

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
    double theta3_i = std::acos(tmp7 / tmp9) - std::atan2(a2_2, c3);

    double theta3_ii = -std::acos(tmp7 / tmp9) - std::atan2(a2_2, c3);

    double theta3_iii = std::acos(tmp8 / tmp9) - std::atan2(a2_2, c3);
    double theta3_iv = -std::acos(tmp8 / tmp9) - std::atan2(a2_2, c3);

    // Now for the orientation part...
    double s23[4];
    double c23[4];
    double sin1[4];
    double cos1[4];

    sin1[0] = std::sin(theta1_i);
    sin1[1] = std::sin(theta1_i);
    sin1[2] = std::sin(theta1_ii);  // ???
    sin1[3] = std::sin(theta1_ii);

    cos1[0] = std::cos(theta1_i);
    cos1[1] = std::cos(theta1_i);
    cos1[2] = std::cos(theta1_ii);  // ???
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

    double theta6_i = std::atan2(get(pose, 0, 1) * s23[0] * cos1[0] + get(pose, 1, 1) * s23[0] * sin1[0] + get(pose, 2, 1) * c23[0],
                            -get(pose, 0, 0) * s23[0] * cos1[0] - get(pose, 1, 0) * s23[0] * sin1[0] - get(pose, 2, 0) * c23[0]);

    double theta6_ii = std::atan2(get(pose, 0, 1) * s23[1] * cos1[1] + get(pose, 1, 1) * s23[1] * sin1[1] + get(pose, 2, 1) * c23[1],
                             -get(pose, 0, 0) * s23[1] * cos1[1] - get(pose, 1, 0) * s23[1] * sin1[1] - get(pose, 2, 0) * c23[1]);

    double theta6_iii = std::atan2(get(pose, 0, 1) * s23[2] * cos1[2] + get(pose, 1, 1) * s23[2] * sin1[2] + get(pose, 2, 1) * c23[2],
                              -get(pose, 0, 0) * s23[2] * cos1[2] - get(pose, 1, 0) * s23[2] * sin1[2] - get(pose, 2, 0) * c23[2]);

    double theta6_iv = std::atan2(get(pose, 0, 1) * s23[3] * cos1[3] + get(pose, 1, 1) * s23[3] * sin1[3] + get(pose, 2, 1) * c23[3],
                             -get(pose, 0, 0) * s23[3] * cos1[3] - get(pose, 1, 0) * s23[3] * sin1[3] - get(pose, 2, 0) * c23[3]);

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
    
    
    for(int i = 0; i < 8; i++){
        sol[6*i+0]  = ofClamp(sol[6*i+0], ofDegToRad(joint_limit_min[0])-offsets[0]* sign_corrections[0], ofDegToRad(joint_limit_max[0])-offsets[0]* sign_corrections[0]);
        sol[6*i+1]  = ofClamp(sol[6*i+1], ofDegToRad(joint_limit_min[1])-offsets[1]* sign_corrections[1], ofDegToRad(joint_limit_max[1])-offsets[1]* sign_corrections[1]);
        sol[6*i+2]  = ofClamp(sol[6*i+2], ofDegToRad(joint_limit_min[2])-offsets[2]* sign_corrections[2], ofDegToRad(joint_limit_max[2])-offsets[2]* sign_corrections[2]);
        sol[6*i+3]  = ofClamp(sol[6*i+3], ofDegToRad(joint_limit_min[3])-offsets[3]* sign_corrections[3], ofDegToRad(joint_limit_max[3])-offsets[3]* sign_corrections[3]);
        sol[6*i+4]  = ofClamp(sol[6*i+4], ofDegToRad(joint_limit_min[4])-offsets[4]* sign_corrections[4], ofDegToRad(joint_limit_max[4])-offsets[4]* sign_corrections[4]);
        sol[6*i+5]  = ofClamp(sol[6*i+5], ofDegToRad(joint_limit_min[5])-offsets[5]* sign_corrections[5], ofDegToRad(joint_limit_max[5])-offsets[5]* sign_corrections[5]);
    }
}

float Kinematics::get(ofMatrix4x4 mat, int row, int col) {
    
    return (mat.getPtr())[row*4+col];
}


