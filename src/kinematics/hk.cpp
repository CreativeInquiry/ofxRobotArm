//
//  hk.cpp
//  example-ik
//
//  Created by Dan Moore on 6/5/22.
//

#include "hk.h"
using namespace ofxRobotArm;

void HKIK::setup(vector<double> offsets,
           vector<double> sign_corrections,
           vector<double> joint_limit_min,
                vector<double> joint_limit_max){
    
}

void HKIK::setParams(vector<double> params){
    
    d1 = params[0];
    a2 = params[1];
    a3 = params[2];
    d4 = params[3];
    d5 = params[4];
    d6 = params[5];
    
}
void HKIK::computeParams(RobotModel model){
    d1 = (model.nodes[0].getZ() - model.nodes[1].getZ())/1000;
    ofLog()<<"d1 "<<d1<<endl;
    a2 = -1*(model.nodes[2].getZ() - model.nodes[1].getZ())/ 1000;
    ofLog()<<"a2 "<<a2<<endl;
    a3 = -1*(model.nodes[3].getZ() - model.nodes[4].getZ()) / 1000;
    ofLog()<<"a3 "<<a3<<endl;
    d4 = (model.nodes[3].getY()-model.nodes[2].getY()) / 1000;
    ofLog()<<"d4 "<<d4<<endl;
    d5 = (model.nodes[5].getZ()-model.nodes[4].getZ()) / 1000;
    ofLog()<<"d5 "<<d5<<endl;
    d6 = (model.nodes[6].getY()-model.nodes[5].getY()) / 1000;
    ofLog()<<"d6 "<<d6<<endl;
}

ofMatrix4x4 HKIK::forward(vector<double> pose){
    double *q = new double[6];
    for (int j = 0; j < 6; j++)
    {
        pose[j] = pose[j] * sign_corrections[j] - offsets[j];
        q[j] = pose[j];
    }
    
    
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
    double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);
    double *T = new double[16];
    *T = c234*c1*s5 - c5*s1;
    T++;
    *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
    *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
    *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
    *T = c1*c5 + c234*s1*s5; T++;
    *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
    *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
    *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
    *T = -s234*s5; T++;
    *T = -c234*s6 - s234*c5*c6; T++;
    *T = s234*c5*s6 - c234*c6; T++;
    *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
    ofMatrix4x4 sol = toOF(T);
    return sol;
}

vector<vector<double>> HKIK::inverse(ofMatrix4x4 pose){
    double *T = new double[16];
    T = toIK(pose);
    double q_sols[8 * 6];
    int num_sols = 0;
    double T02 = -*T;
    T++;
    double T00 = *T;
    T++;
    double T01 = *T;
    T++;
    double T03 = -*T;
    T++;
    double T12 = -*T;
    T++;
    double T10 = *T;
    T++;
    double T11 = *T;
    T++;
    double T13 = -*T;
    T++;
    double T22 = *T;
    T++;
    double T20 = -*T;
    T++;
    double T21 = -*T;
    T++;
    double T23 = *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
        double A = d6 * T12 - T13;
        double B = d6 * T02 - T03;
        double R = A * A + B * B;
        if (fabs(A) < ZERO_THRESH)
        {
            double div;
            if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
                div = -SIGN(d4) * SIGN(B);
            else
                div = -d4 / B;
            double arcsin = asin(div);
            if (fabs(arcsin) < ZERO_THRESH)
                arcsin = 0.0;
            if (arcsin < 0.0)
                q1[0] = arcsin + 2.0 * PI;
            else
                q1[0] = arcsin;
            q1[1] = PI - arcsin;
        }
        else if (fabs(B) < ZERO_THRESH)
        {
            double div;
            if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
                div = SIGN(d4) * SIGN(A);
            else
                div = d4 / A;
            double arccos = acos(div);
            q1[0] = arccos;
            q1[1] = 2.0 * PI - arccos;
        }
        else if (d4 * d4 > R)
        {
//            return num_sols;
        }
        else
        {
            double arccos = acos(d4 / sqrt(R));
            double arctan = atan2(-B, A);
            double pos = arccos + arctan;
            double neg = -arccos + arctan;
            if (fabs(pos) < ZERO_THRESH)
                pos = 0.0;
            if (fabs(neg) < ZERO_THRESH)
                neg = 0.0;
            if (pos >= 0.0)
                q1[0] = pos;
            else
                q1[0] = 2.0 * PI + pos;
            if (neg >= 0.0)
                q1[1] = neg;
            else
                q1[1] = 2.0 * PI + neg;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
        for (int i = 0; i < 2; i++)
        {
            double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - d4);
            double div;
            if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
                div = SIGN(numer) * SIGN(d6);
            else
                div = numer / d6;
            double arccos = acos(div);
            q5[i][0] = arccos;
            q5[i][1] = 2.0 * PI - arccos;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                double c1 = cos(q1[i]), s1 = sin(q1[i]);
                double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                double q6;
                ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                if (fabs(s5) < ZERO_THRESH)
                    q6 = 0.0;
                else
                {
                    q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1),
                               SIGN(s5) * (T00 * s1 - T10 * c1));
                    if (fabs(q6) < ZERO_THRESH)
                        q6 = 0.0;
                    if (q6 < 0.0)
                        q6 += 2.0 * PI;
                }
                ////////////////////////////////////////////////////////////////////////////////

                double q2[2], q3[2], q4[2];
                ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                double c6 = cos(q6), s6 = sin(q6);
                double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
                double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
                double p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d6 * (T02 * c1 + T12 * s1) +
                              T03 * c1 + T13 * s1;
                double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

                double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
                if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
                    c3 = SIGN(c3);
                else if (fabs(c3) > 1.0)
                {
                    // TODO NO SOLUTION
                    continue;
                }
                double arccos = acos(c3);
                q3[0] = arccos;
                q3[1] = 2.0 * PI - arccos;
                double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
                double s3 = sin(arccos);
                double A = (a2 + a3 * c3), B = a3 * s3;
                q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
                q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
                double c23_0 = cos(q2[0] + q3[0]);
                double s23_0 = sin(q2[0] + q3[0]);
                double c23_1 = cos(q2[1] + q3[1]);
                double s23_1 = sin(q2[1] + q3[1]);
                q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
                q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
                ////////////////////////////////////////////////////////////////////////////////
                for (int k = 0; k < 2; k++)
                {
                    if (fabs(q2[k]) < ZERO_THRESH)
                        q2[k] = 0.0;
                    else if (q2[k] < 0.0)
                        q2[k] += 2.0 * PI;
                    if (fabs(q4[k]) < ZERO_THRESH)
                        q4[k] = 0.0;
                    else if (q4[k] < 0.0)
                        q4[k] += 2.0 * PI;
                    q_sols[num_sols * 6 + 0] = q1[i];
                    q_sols[num_sols * 6 + 1] = q2[k];
                    q_sols[num_sols * 6 + 2] = q3[k];
                    q_sols[num_sols * 6 + 3] = q4[k];
                    q_sols[num_sols * 6 + 4] = q5[i][j];
                    q_sols[num_sols * 6 + 5] = q6;
                    num_sols++;
                }
            }
        }
    }

    vector<vector<double> > sols;
    for (int i = 0; i < num_sols; i++)
    {
        vector<double> fooSol;
        for (int j = 0; j < 6; j++)
        {
            q_sols[6 * i + j] = q_sols[6 * i + j] * sign_corrections[j] + offsets[j];
            fooSol.push_back(q_sols[6 * i + j]);
        }
        
        if (isValid(fooSol))
        {
            harmonizeTowardZero(fooSol);
            sols.push_back(fooSol);
        }
    }
  
    return sols;
}
