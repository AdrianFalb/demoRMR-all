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
    bool useCamera1;
    int actIndex;

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData, int address);
    int processThisRobot(TKobukiData robotdata,int address);
    int processThisCamera(cv::Mat cameraData);
    void processThisMessage(sockaddr_in ske_si_me, sockaddr_in ske_si_other, sockaddr_in ske_si_posli, int ske_s, int ske_recv_len, int port);
    void issueRobotCommand(std::string robotId, std::string robotCommand, std::string robotFollowCommand);

    // Methods for reseting parameters
    void resetControlParameters(double m);
    void resetBooleansStopCommand();
    void resetCollisionParams();

    IpReturnMessage checkIpAddress(std::string ip);

    std::thread robotMessageThread;
    std::thread th1;
    std::thread th2;
    std::thread th3;

    std::vector<int> usedRobotIps;

    // Robot control
    double meters;
    int angleDelta;
    int oldAngle;
    int angle;
    bool firstGyroData = true;
    bool firstTime = true;

    // Booleans for robot control
    bool metersReset = false;
    bool rotationReset = false;
    bool rotationFromRight = false;
    bool rotationFromLeft = false;

    // Booleans for collision detection
    bool collisionDetectedFront = false;
    bool collisionDetectedBack = false;
    bool evadingCollision = false;
    bool collisionRotateLeft = false;
    bool collisionRotateRight = false;
    std::string avoidedCollisionWithCommand = "";

    bool processThisRobotAllowed = false;

    // Lidar
    double lidarDist;
    double shortestLidarDistance;
    double shortestLidarAngle;

    // Odometry
    int numberOfCallbacksEncoderDataWasNotChanged = 0;
    int mLeftDelta;
    int mLeftOld;
    int mRight;
    int mRightOld;

    bool switchButtonWasEnabled;

    int stopAll;
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

    void addNewRobotToGroup(unsigned short int robotIndex, unsigned short int numberOfRobots);
    void setIndexOfCurrentRobot(unsigned short int robotIndex);
    void setIpAddress(std::string ipAddress);
    void disableButtons();

    //JOYINFO joystick_info;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
    int updateLaserPicture;

    LaserMeasurement copyOfLaserData1;
    LaserMeasurement copyOfLaserData2;
    LaserMeasurement copyOfLaserData3;

    std::string httpString;
    std::string portString;
    std::string fileString;
    std::string ipAddress;
    std::string cameraAddress;

    unsigned int laserParametersLaserPortOut;
    unsigned int laserParametersLaserPortIn;
    unsigned int robotParametersLaserPortOut;
    unsigned int robotParametersLaserPortIn;

    std::map<unsigned short int, Robot*> robotGroup;
    unsigned short int indexOfCurrentRobot;
    TKobukiData robot_data;
    int data_counter;
    QTimer *timer;

    QJoysticks *instance;

    double forwardSpeed; // mm/s
    double rotationSpeed; // omega/s

public slots:
    void setUiValues(double robotX,double robotY,double robotFi);
    void setRobotModes(bool state, bool mode);
    void setSelectedRobot(bool changed);
    void enableButtons();

signals:
    void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
    void startButtonPressed(bool pressed);
    void robotModesChanged(bool stateChanged, bool modeChanged);
    void selectedRobotChanged(bool changed);

};

#endif // MAINWINDOW_H
