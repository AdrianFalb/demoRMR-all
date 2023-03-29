#ifndef ROBOT_H
#define ROBOT_H

#define useCamera

#ifdef useCamera
    #include <opencv2/core/core.hpp>
    #include <opencv2/highgui/highgui.hpp>
    #include "opencv2/imgproc/imgproc.hpp"
    #include "opencv2/highgui/highgui.hpp"
    #include "opencv2/core/utility.hpp"
    #include "opencv2/videoio.hpp"
    #include "opencv2/imgcodecs.hpp"
#endif

#include "robot_global.h"
#include "rplidar.h"
#include "CKobuki.h"
#include <thread>
#include <functional>
#include <list>
#include <algorithm>
#include <utility>
#include <atomic>
#include <mutex>
#include <future>
#include <random>
#include <iostream>
#include <memory>

class ROBOT_EXPORT Robot {

public:

    ~Robot();
    Robot(std::string ipaddressLaser="127.0.0.1",int laserportRobot=52999, int laserportMe=5299,std::function<int(LaserMeasurement,int)> &lascallback=do_nothing_laser,std::string ipaddressRobot="127.0.0.1",int robotportRobot=53000, int robotportMe=5300,std::function<int(TKobukiData,int)> &robcallback=do_nothing_robot);

    // default functions.. please do not rewrite.. make your own callback
    static std::function<int(TKobukiData,int)> do_nothing_robot;
    static std::function<int(LaserMeasurement,int)> do_nothing_laser;

    void robotStart();
    void setLaserParameters(std::string ipaddress, int laserportRobot, int laserportMe, std::function<int(LaserMeasurement,int)> callback) {

        this->laser_ip_portOut = laserportRobot;
        this->laser_ip_portIn = laserportMe;
        this->laser_ipaddress = ipaddress;
        this->laser_callback = callback;
        this->wasLaserSet = 1;
    }

    void setRobotParameters(std::string ipaddress, int robotportRobot, int robotportMe, std::function<int(TKobukiData,int)> callback) {

        this->robot_ip_portOut = robotportRobot;
        this->robot_ip_portIn = robotportMe;
        this->robot_ipaddress = ipaddress;
        this->robot_callback = callback;
        this->wasRobotSet = 1;
    }

    void ramp(int max_speed);

    void setTranslationSpeed(int mmpersec);
    void setRotationSpeed(double radpersec);
    void setArcSpeed(int mmpersec,int radius);
    void setCameraParameters(std::string link,std::function<int(cv::Mat)> callback) {

        this->camera_link=link;
        this->camera_callback=callback;
        this->wasCameraSet=1;
    }

    void setMyRobotGroupIndex(unsigned short int index) {

        this->myRobotGroupIndex = index;
    }

    unsigned short int getMyRobotGroupIndex() {

        return this->myRobotGroupIndex;
    }

    std::string getIpAddress() {
        return this->robot_ipaddress;
    }

    void set_accept_commands(bool b) {
        this->accept_commands = b;
    }

    bool get_accept_commands() {
        return this->accept_commands;
    }

    void set_actual_speed(double speed) {
        this->actual_speed = speed;
    }

    double get_actual_speed() {
        return this->actual_speed;
    }

private:

    std::promise<void> ready_promise;
    std::shared_future<void> readyFuture;
    int wasLaserSet;
    int wasRobotSet;
    int wasCameraSet;
    //veci na laser
    LaserMeasurement copyOfLaserData;
    void laserprocess();
    std::string laser_ipaddress;
    int laser_ip_portOut;
    int laser_ip_portIn;
    std::thread laserThreadHandle;
    std::function<int(LaserMeasurement,int)> laser_callback = nullptr;

    unsigned short int myRobotGroupIndex;
    bool accept_commands;
    double actual_speed;

    //veci pre podvozok
    CKobuki robot;
    TKobukiData sens;
    std::string robot_ipaddress;
    int robot_ip_portOut;
    int robot_ip_portIn;
    std::thread robotThreadHandle;
    void robotprocess();
    std::function<int(TKobukiData,int)> robot_callback = nullptr;

    //veci pre kameru -- pozor na kameru, neotvarat ak nahodou chcete kameru pripojit na detekciu kostry...

    std::string camera_link;
    std::thread cameraThreadHandle;
    std::function<int(cv::Mat)> camera_callback = nullptr;
    void imageViewer();

    ///
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;


    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;

    int ske_s,  ske_recv_len;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;

    #ifdef _WIN32
        int rob_slen;
        int las_slen;
        int ske_slen;
    #else
        unsigned int rob_slen;
        unsigned int las_slen;
        unsigned int ske_slen;
    #endif
};

#endif // ROBOT_H
