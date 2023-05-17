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

    void robot_start();
    void set_laser_parameters(std::string ipaddress, int laserportRobot, int laserportMe, std::function<int(LaserMeasurement,int)> callback) {

        this->laser_ip_portOut = laserportRobot;
        this->laser_ip_portIn = laserportMe;
        this->laser_ipaddress = ipaddress;
        this->laser_callback = callback;
        this->was_laser_set = 1;
    }

    void set_robot_parameters(std::string ipaddress, int robotportRobot, int robotportMe, std::function<int(TKobukiData,int)> callback) {

        this->robot_ip_portOut = robotportRobot;
        this->robot_ip_portIn = robotportMe;
        this->robot_ipaddress = ipaddress;
        this->robot_callback = callback;
        this->was_robot_set = 1;
    }

    void ramp(double max_speed, int stopping, int rotating);

    void set_translation_speed(int mmpersec);
    void set_rotation_speed(double radpersec);
    void set_arc_speed(int mmpersec,int radius);
    void set_camera_parameters(std::string link,std::function<int(cv::Mat)> callback) {

        this->camera_link=link;
        this->camera_callback=callback;
        this->was_camera_set=1;
    }

    void set_my_robot_group_index(unsigned short int index) {

        this->my_robot_group_index = index;
    }

    unsigned short int get_my_robot_group_index() {

        return this->my_robot_group_index;
    }

    std::string getIpAddress() {
        return this->robot_ipaddress;
    }

    void set_awake_state(bool b) {
        this->accept_commands = b;
    }

    bool get_awake_state() {
        return this->accept_commands;
    }

    void set_actual_speed(double speed) {
        this->actual_speed = speed;
    }

    double get_actual_speed() {
        return this->actual_speed;
    }

    void set_follow_mode(bool b) {
        this->follow_mode = b;
    }

    bool get_follow_mode() {
        return this->follow_mode;
    }

    void set_doing_gesture(bool b) {
        this->doing_gesture = b;
    }

    bool get_doing_gesture() {
        return this->doing_gesture;
    }

    void set_current_command(std::string command) {
        this->previous_command = this->current_command;
        this->current_command = command;
    }

    std::string get_current_command() {
        return this->current_command;
    }

    std::string get_previous_command() {
        return this->previous_command;
    }

private:

    std::promise<void> ready_promise;
    std::shared_future<void> ready_future;
    int was_laser_set;
    int was_robot_set;
    int was_camera_set;
    //veci na laser
    LaserMeasurement copy_of_laser_data;
    void laserprocess();
    std::string laser_ipaddress;
    int laser_ip_portOut;
    int laser_ip_portIn;
    std::thread laser_thread_handle;
    std::function<int(LaserMeasurement,int)> laser_callback = nullptr;

    unsigned short int my_robot_group_index;
    bool accept_commands;
    bool follow_mode;
    double actual_speed;    
    bool doing_gesture;
    std::string current_command = "";
    std::string previous_command = "";

    //veci pre podvozok
    CKobuki robot;
    TKobukiData sens;
    std::string robot_ipaddress;
    int robot_ip_portOut;
    int robot_ip_portIn;
    std::thread robot_thread_handle;
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
