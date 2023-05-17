#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#endif

#include <thread>
#include <functional>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>


//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"

#include <QJoysticks.h>

union IP {
    unsigned int ip;
    struct {
      unsigned char d;
      unsigned char c;
      unsigned char b;
      unsigned char a;
    } ip2;
};

struct IpReturnMessage {
    bool ip_valid;
    std::string message;
};

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    bool use_camera1;
    int act_index;

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int process_this_lidar(LaserMeasurement laserData, int address);
    int process_this_robot(TKobukiData robotdata,int address);
    int process_this_camera(cv::Mat cameraData);    
    void process_this_message(sockaddr_in ske_si_me, sockaddr_in ske_si_other, sockaddr_in ske_si_posli, int ske_s, int ske_recv_len, int port);
    void issue_robot_command(std::string robot_id, std::string robot_command, std::string robot_follow_command);

    // Methods for reseting parameters
    void reset_control_parameters(double m);
    void reset_booleans_stop_command();
    void reset_collision_params();

    IpReturnMessage check_ip_address(std::string ip);

    std::thread robot_message_thread;
    std::thread th1;
    std::thread th2;
    std::thread th3;

    std::vector<int> used_robot_ips;    

    // Robot control
    double meters;
    int angle_delta;
    int old_angle;
    int angle;
    bool first_gyro_data = true;
    bool first_time = true;

    // Booleans for robot control
    bool meters_reset = false;
    bool rotation_reset = false;
    bool rotation_from_right = false;
    bool rotation_from_left = false;    

    // Booleans for collision detection
    bool collision_detected_front = false;
    bool collision_detected_back = false;    
    bool evading_collision = false;
    std::string avoided_collision_with_command = "";

    bool process_this_robot_allowed = false;

    // Lidar
    double lidar_dist;
    double shortest_lidar_distance;
    double shortest_lidar_angle;

    // Odometry
    int number_of_callbacks_encoder_data_was_not_changed = 0;
    int m_left_delta;
    int m_left_old;
    int m_right;
    int m_right_old;

    bool switch_button_was_enabled;

    int stopall;
    //int update_skeleton_picture;

    bool robot_not_moving; // daj si flag, ktory sa nastavi na true ked sa robot pohne a resetne sa iba ked je false

    struct sockaddr_in las_si_me, las_si_other,las_si_posli; // veci na broadcast laser
    int las_s,  las_recv_len;

    struct sockaddr_in ske_si_me1, ske_si_other1,ske_si_posli1;
    int ske_s1, ske_recv_len1;

    struct sockaddr_in ske_si_me2, ske_si_other2, ske_si_posli2;
    int ske_s2, ske_recv_len2;

    struct sockaddr_in ske_si_me3, ske_si_other3, ske_si_posli3;
    int ske_s3, ske_recv_len3;

    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli; // veci na broadcast robot
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

private slots:
    void on_pushButton_start_clicked();
    void on_pushButton_forward_clicked();
    void on_pushButton_back_clicked();
    void on_pushButton_left_clicked();
    void on_pushButton_right_clicked();
    void on_pushButton_stop_clicked();
    void on_pushButton_clicked();
    void on_pushButton_switch_robot_clicked();
    void on_pushButton_add_robot_clicked();
    void on_pushButton_follow_mode_clicked();
    void on_pushButton_accept_commands_clicked();
    void get_new_frame();

private:

    void add_new_robot_to_group(unsigned short int robot_index, unsigned short int number_of_robots);
    void set_index_of_current_robot(unsigned short int robot_index);
    void set_ip_address(std::string ip_address);
    void disable_buttons();

    //JOYINFO joystick_info;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
    int update_laser_picture;

    LaserMeasurement copy_of_laser_data1;
    LaserMeasurement copy_of_laser_data2;
    LaserMeasurement copy_of_laser_data3;

    std::string http_string;
    std::string port_string;
    std::string file_string;
    std::string ip_address;
    std::string camera_address;

    unsigned int laserParametersLaserPortOut;
    unsigned int laserParametersLaserPortIn;
    unsigned int robotParametersLaserPortOut;
    unsigned int robotParametersLaserPortIn;

    std::map<unsigned short int, Robot*> robot_group;
    unsigned short int index_of_current_robot;
    TKobukiData robot_data;
    int data_counter;
    QTimer *timer;

    QJoysticks *instance;

    double forward_speed; // mm/s
    double rotation_speed; // omega/s    

public slots:
    void set_ui_values(double robotX,double robotY,double robotFi);
    void set_robot_modes(bool mode, bool power_mode);
    void set_selected_robot(bool changed);
    void enable_buttons();

signals:
    void ui_values_changed(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
    void start_button_pressed(bool pressed);
    void robot_modes_changed(bool mode_changed, bool power_mode_changed);
    void selected_robot_changed(bool changed);

};

#endif // MAINWINDOW_H
