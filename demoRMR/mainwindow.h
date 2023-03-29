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
    //void process_this_message();
    void process_this_message(sockaddr_in ske_si_me, sockaddr_in ske_si_other, sockaddr_in ske_si_posli, int ske_s, int ske_recv_len, int port);
    void issue_robot_command(std::string robot_id, std::string robot_command);

    std::thread robot_message_thread;
    std::thread th1;
    std::thread th2;
    std::thread th3;

    std::vector<int> used_robot_ips;


    int stopall;
    //int update_skeleton_picture;

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

    //std::string received_message;
    //std::string robot_ip_address;
    //std::string robot_command;

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
    void on_pushButton_9_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_clicked();

    void on_pushButton_switch_robot_clicked();
    void on_pushButton_add_robot_clicked();
    void get_new_frame();

private:

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

    void add_new_robot_to_group(unsigned short int robot_index, unsigned short int number_of_robots);
    void set_index_of_current_robot(unsigned short int robot_index);
    void set_ip_address(std::string ip_address);

public slots:
    void setUiValues(double robotX,double robotY,double robotFi);
    void setButtonStates();

signals:
    void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
    void startButtonPressed(bool pressed);

};

#endif // MAINWINDOW_H
