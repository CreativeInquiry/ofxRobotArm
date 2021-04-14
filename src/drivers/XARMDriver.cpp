#include "XARMDriver.h"
using namespace ofxRobotArm;
XARMDriver::XARMDriver(){

}
XARMDriver::~XARMDriver(){

}
void XARMDriver::setAllowReconnect(bool bDoReconnect) {

}
void XARMDriver::setup() {

}
void XARMDriver::setup(string ipAddress, int port, double minPayload , double maxPayload ) {

}
void XARMDriver::setup(string ipAddress, double minPayload , double maxPayload ) {
    robot = new XArmAPI(ipAddress);
    robot->connect();
    robot->set_mode(0);
    robot->set_state(0);
    robot->motion_enable(true);
}
void XARMDriver::setup(int port, double minPayload , double maxPayload ) {

}
void XARMDriver::start() {

}
bool  XARMDriver::isConnected() {

}
void XARMDriver::disconnect() {

}
void XARMDriver::stopThread() {

}
void XARMDriver::toggleTeachMode() {

}
void XARMDriver::setTeachMode(bool enabled) {

}
void XARMDriver::threadedFunction() {
    while(isThreadRunning()){
        int ret;
        unsigned char version[40];
        ret = robot->get_version(version);
        printf("ret=%d, version: %s\n", ret, version);

        int state;
        ret = robot->get_state(&state);
        printf("ret=%d, state: %d, mode: %d\n", ret, state, robot->mode);

        int cmdnum;
        ret = robot->get_cmdnum(&cmdnum);
        printf("ret=%d, cmdnum: %d\n", ret, cmdnum);

        int err_warn[2];
        ret = robot->get_err_warn_code(err_warn);
        printf("ret=%d, err: %d, warn: %d\n", ret, err_warn[0], err_warn[1]);

        fp32 pose[6];
        ret = robot->get_position(pose);
        printf("ret=%d, ", ret);
        print_nvect("pose: ", pose, 6);

        fp32 angles[7];
        ret = robot->get_servo_angle(angles);
        printf("ret=%d, ", ret);
        print_nvect("angles: ", angles, 7);
        
        if(bMove && currentPose.size() > 0){
            timeNow = ofGetElapsedTimef();
            if( bMove || timeNow-lastTimeSentMove >= 1.0/60.0){
                currentPose = getAchievablePosition(currentPose);
                fp32 pose[currentPose.size()];
                int i = 0;
                for(auto angle : currentPose){
                    pose[i] = angle;
                    i++;
                }
                ret = robot->set_servo_angle(pose, false);
                printf("set_servo_angle, ret=%d\n", ret);
                if(!bMove){
                    deccelCount--;
                    if( deccelCount < 0){
                        deccelCount = 0;
                    }
                }
                lastTimeSentMove = timeNow;
            }
            bMove = false;
        }
    }
}
ofVec4f XARMDriver::getCalculatedTCPOrientation() {

}
vector<double> XARMDriver::getToolPointRaw() {

}
vector<double> XARMDriver::getCurrentPose() {

}
    
bool  XARMDriver::isDataReady() {
    return true;
}
float  XARMDriver::getThreadFPS() {
    return 0.0;
}

void XARMDriver::setSpeed(vector<double> speeds, double acceleration) {
    lock();
    currentSpeed = speeds;
    unlock();
}
void XARMDriver::setPose(vector<double> positions) {
    lock();
    currentPose = positions;
    unlock();
}
Pose  XARMDriver::getToolPose() {
    Pose p;
    return p;
}
vector<double>  XARMDriver::getInitPose() {
     vector<double> foo;
     return foo;
}


void XARMDriver::report_location_callback(const fp32* pose, const fp32* angles) {
    print_nvect("pose    = ", (fp32*)pose, 6);
    print_nvect("angles  = ", (fp32*)angles, 7);
}

void XARMDriver::connect_changed_callback(bool connected, bool reported) {
    printf("connected: %d, reported: %d\n", connected, reported);
}

void XARMDriver::state_changed_callback(int state) {
    printf("state: %d\n", state);
}

void XARMDriver::mode_changed_callback(int mode) {
    printf("mode: %d\n", mode);
}

void XARMDriver::mtable_mtbrake_changed_callback(int mtable, int mtbrake) {
    printf("mtable: %d, mtbrake: %d\n", mtable, mtbrake);
}

void XARMDriver::error_warn_changed_callback(int err, int warn) {
    printf("err: %d, warn: %d\n", err, warn);
}

void XARMDriver::cmdnum_changed_callback(int cmdnum) {
    printf("cmdnum: %d\n", cmdnum);
}
