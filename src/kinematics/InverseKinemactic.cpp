#include "InverseKinemactic.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
using namespace ofxRobotArm;
InverseKinemactic::InverseKinemactic(ofxRobotArm::RobotType type){
    kinematics = Kinematics(type);
}
InverseKinemactic::InverseKinemactic(){
    // just pick a default Robot Type to start (FIX LATER)
    ofLog() << "PICKING RobotType::UR10 as default Kinematic Model" << endl;
    kinematics = Kinematics(ofxRobotArm::RobotType::UR10);
}
InverseKinemactic::~InverseKinemactic(){
    
}

/// \brief Converts a 4x4 matrix to a 1D array
/// \param input ofMatrix4x4 to convert
/// \return row-major array in UR World Cords
double* toIK(ofMatrix4x4 input){
    double* T = new double[16];
//    cout<<"toUR ==================="<<endl;
//    cout<<"ofMatrix ==================="<<endl;
//    cout<<input<<endl;
    for(int i = 0; i < 4; i++){
        T[i] = (double)input._mat[i][0];
        T[i+(4)] = (double)input._mat[i][1];
        T[i+(8)] = (double)input._mat[i][2];
        T[i+(12)] = (double)input._mat[i][3];
    }
    return T;
}

ofMatrix4x4 toOF(double * T){
    ofMatrix4x4 output;
    for(int i = 0; i < 4; i++){
        output._mat[i][0] = T[i];
        output._mat[i][1] = T[i+(4)];
        output._mat[i][2] = T[i+(8)];
        output._mat[i][3] = T[i+(12)];
    }
    return output;
}

int argMin(std::vector<double> vec)
{
    std::vector<double>::iterator mins = std::min_element(vec.begin(), vec.end()); //returns all mins
    double min = mins[0]; //select the zeroth min if multiple mins exist
    for(int i=0; i < vec.size(); i++)
    {
        //Note: could use fabs( (min - vec[i]) < 0.01) if worried about floating-point precision
        if(vec[i] == min)
            return i;
    }
    return -1;
}


int InverseKinemactic::selectSolution(vector<vector<double> > & inversePosition, vector<double> currentQ, vector<double> weight)
{
    int selectedSolution = 0;
    
    for(int i = 0; i < inversePosition.size(); i++){
        for(int j = 0; j < inversePosition[i].size(); j++){
            if(j == 0){
                inversePosition[i][j] = inversePosition[i][j]-PI;
            }
            if(j == 1 || j == 3){
                if(inversePosition[i][j] > PI){
                    inversePosition[i][j]  = ofMap(inversePosition[i][j], PI, TWO_PI, -PI, 0, true);
                }
            }
            if(preInversePosition.size() > 0){
                if(i == selectedSolution){
                    if(preInversePosition[i][j]-inversePosition[i][j] > 2*PI){
                        ofLog(OF_LOG_WARNING)<<"JOINT WRAPS SOL "<<ofToString(i)<<" Joint "<<ofToString(j)<<endl;
                    }
                }
            }
        }
    }
    vector<double> test_sol;
    vector<vector<double> > valid_sols;
    test_sol.assign(6, 9999.);
    vector<double> addAngle = {-1*TWO_PI, 0, TWO_PI};
    for(int i = 0; i < inversePosition.size(); i++){
        for(int j = 0; j < inversePosition[i].size(); j++){
            for(int k = 0; k < addAngle.size(); k++){
                float test_ang = inversePosition[i][j]+addAngle[k];
                if(fabs(test_ang - currentQ[j])  < fabs(test_sol[j] -  currentQ[j]) && abs(test_ang) <= TWO_PI){
                    test_sol[j] = test_ang;
                }
            }
        }
        bool testValid = false;
        for(int l = 0; l < test_sol.size(); l++){
            if(test_sol[l] != 9999){
                testValid = true;
            }else{
                testValid = false;
            }
        }
        if(testValid){
            valid_sols.push_back(test_sol);
      
        }
    }
   
    vector<double> sumsValid;
    sumsValid.assign(valid_sols.size(), 0);
    for(int i = 0; i < valid_sols.size(); i++){
        for(int j = 0; j < valid_sols[i].size(); j++){
            sumsValid[i] = pow(weight[j]*(valid_sols[i][j] - currentQ[j]), 2);
        }
    }
    
    
    preInversePosition = inversePosition;
    
    if(inversePosition.size() > 0){
        return 0;
    }else{
        return -1;
    }
}


vector<vector<double> > InverseKinemactic::inverseKinematics(double o, double t, double th, double f, double fi, double s)
{
    double q[6] = {o, t, th, f, fi, s};
    double* T = new double[16];
    double q_sols[8*6];
    int num_sols = kinematics.inverse(T, q_sols);
    vector<vector<double> > sols;
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        sols.push_back(fooSol);
    }
    return sols;
}


vector<vector<double> > InverseKinemactic::inverseKinematics(vector<double> input)
{
    if(input.size() == 6){
        return inverseKinematics(input[0], input[1], input[2], input[3], input[4], input[5]);
    }
    return vector<vector<double>>();
}

vector<vector<double> > InverseKinemactic::inverseKinematics(ofxRobotArm::Joint pose){
    ofMatrix4x4 matPose;
    ofMatrix4x4 matT, matR;
    matT.makeTranslationMatrix(pose.position);
    matR.makeRotationMatrix(pose.rotation);
    matPose = matR*matT;
    return inverseKinematics(matPose);
    
}

vector<vector<double> > InverseKinemactic::inverseKinematics(ofMatrix4x4 pose)
{
    double q_sols[8*6];
    double* T = new double[16];
    T = toIK(pose);
    int num_sols = kinematics.inverse(T, q_sols);
    vector<vector<double> > sols;
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        sols.push_back(fooSol);
    }
    return sols;
}

ofMatrix4x4 InverseKinemactic::forwardKinematics(vector<double> pose)
{
    currentPosition = pose;
    return toOF(forwardKinematics(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
}

double* InverseKinemactic::forwardKinematics(double o, double t, double th, double f, double fi, double s)
{
    double q[6] = {o, t, th, f, fi, s};
    double* T1 = new double[16];
    double* T2 = new double[16];
    double* T3 = new double[16];
    double* T4 = new double[16];
    double* T5 = new double[16];
    double* T6 = new double[16];
    
    kinematics.forward_all(q, T1, T2, T2, T4, T5, T6);
    return T6;
}


