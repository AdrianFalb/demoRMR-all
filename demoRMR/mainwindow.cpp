#include "mainwindow.h"
#include "utilities.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

#ifdef _WIN32
    #ifndef M_PI
        #define M_PI 3.14159265358979323846
    #endif
    #include <windows.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
#endif

#define MAX_TRANSLATION_SPEED_CONTROL_MODE 300
#define MAX_ROTATION_SPEED_CONTROL_MODE (M_PI/6)
#define MAX_TRANSLATION_SPEED_FOLLOW_MODE 300
#define MAX_ROTATION_SPEED_FOLLOW_MODE (M_PI/2)
#define COLLISION_DETECTION_RANGE 300

MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui(new Ui::MainWindow) {   

    // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    this->httpString = "http://";
    this->fileString = "/stream.mjpg";

#ifndef SIMULATOR
    this->portString = ":8000";
    this->ipAddress = "192.168.1.";
#endif

#ifdef SIMULATOR
    this->portString = ":8889"; // simulator
    this->ipAddress = "127.0.0.1"; // Local host - default
#endif

    this->cameraAddress = this->httpString + this->ipAddress + this->portString + this->fileString; // Local host - Default
    this->indexOfCurrentRobot = 0;

    this->laserParametersLaserPortOut = 52999;
    this->laserParametersLaserPortIn = 5299;
    this->robotParametersLaserPortOut = 53000;
    this->robotParametersLaserPortIn = 5300;

    ui->setupUi(this);
    // this->data_counter = 0;
    this->actIndex = -1;
    this->useCamera1 = false;

    this->disableButtons();

    switchButtonWasEnabled = false;
    ui->lineEdit->setText(QString::fromStdString(this->ipAddress));

    auto message = std::bind(&MainWindow::processThisMessage, this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3,
                             std::placeholders::_4, std::placeholders::_5,
                             std::placeholders::_6);

    th1 = std::move(std::thread(message, ske_si_me1, ske_si_other1, ske_si_posli1, ske_s1, ske_recv_len1, 23432));
    th2 = std::move(std::thread(message, ske_si_me2, ske_si_other2, ske_si_posli2, ske_s2, ske_recv_len2, 23433));
    th3 = std::move(std::thread(message, ske_si_me3, ske_si_other3, ske_si_posli3, ske_s3, ske_recv_len3, 23434));
}

MainWindow::~MainWindow() {
    delete ui;    
}

void MainWindow::resetControlParameters(double m) {
    meters = m;
    angle = 0;
    numberOfCallbacksEncoderDataWasNotChanged = 0;
}

void MainWindow::resetBooleansStopCommand() {
    metersReset = false;
    rotationReset = false;
    rotationFromRight = false;
    rotationFromLeft = false;
    firstGyroData = true;
}

void MainWindow::resetCollisionParams() {
    collisionDetectedFront = false;
    collisionDetectedBack = false;
    evadingCollision = false;
}

void MainWindow::paintEvent(QPaintEvent *event) {

    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::cyan);
    QRect rect(20, 120, 700, 500);
    rect = ui->frame->geometry();
    rect.translate(0, 15);
    painter.drawRect(rect);

    LaserMeasurement copyOfLaserData = copyOfLaserData1;

    if (robotGroup.empty() == false && usedRobotIps.empty() == false) {

        if (robotGroup.at(indexOfCurrentRobot)->getIpAddress().empty() == false) {

#ifndef SIMULATOR
            char a = robotGroup.at(indexOfCurrentRobot)->getIpAddress().string::at(10);
            char b = robotGroup.at(indexOfCurrentRobot)->getIpAddress().string::at(11);
            std::string ip (1, a);
            ip = ip + b;
#endif
#ifdef SIMULATOR
            std::string ip = "1";
#endif
            // std::cout << "IP of currently selected robot: " << ip << std::endl;

            if (std::stoi(ip) == usedRobotIps.at(0)) {
                copyOfLaserData = copyOfLaserData1;
            } else if (std::stoi(ip) == usedRobotIps.at(1)) {
                copyOfLaserData = copyOfLaserData2;
            } else if (std::stoi(ip) == usedRobotIps.at(2)) {
                copyOfLaserData = copyOfLaserData3;
            }
        }
    }

    if (useCamera1 == true && actIndex > -1) {

        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );
        painter.drawImage(rect, image.rgbSwapped());

    } else {

        if (this->updateLaserPicture == 1) {

            this->updateLaserPicture = 0;
            painter.setPen(pero);

            // teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            // std::cout<<copyOfLaserData.numberOfScans<<std::endl;

            for (int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++) {

                /*  int dist=rand()%500;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-k)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-k)*3.14159/180.0))+rect.topLeft().y();
                */

                int dist = copyOfLaserData.Data[k].scanDistance / 20;
                int xp = rect.width() - (rect.width()/2 + dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0)) + rect.topLeft().x();
                int yp = rect.height() - (rect.height()/2 + dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0)) + rect.topLeft().y();
                if (rect.contains(xp, yp)) {
                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }
        }
    }
}

void MainWindow::setUiValues(double robotX,double robotY,double robotFi) {
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::setRobotModes(bool state, bool mode) {
    if (state) {
        ui->lineEdit_mode->setText("Awake state");
        ui->pushButton_accept_commands->setText("Put robot to sleep");
    } else {
        ui->lineEdit_mode->setText("Sleep state");
        ui->pushButton_accept_commands->setText("Wake up robot");
    }

    if (mode) {
        ui->lineEdit_power_mode->setText("Follow mode");
        ui->pushButton_follow_mode->setText("Turn off Follow Mode");
    } else {
        ui->lineEdit_power_mode->setText("Control mode");
        ui->pushButton_follow_mode->setText("Turn on Follow Mode");
    }
}

void MainWindow::disableButtons() {

    ui->pushButton_forward->setEnabled(false);
    ui->pushButton_right->setEnabled(false);
    ui->pushButton_left->setEnabled(false);
    ui->pushButton_back->setEnabled(false);
    ui->pushButton_stop->setEnabled(false);
    ui->pushButton->setEnabled(false);
    ui->pushButton_7->setEnabled(false);
    ui->pushButton_8->setEnabled(false);
    ui->pushButton_add_robot->setEnabled(false);
    ui->pushButton_switch_robot->setEnabled(false);
    ui->pushButton_follow_mode->setEnabled(false);
    ui->pushButton_accept_commands->setEnabled(false);
}

void MainWindow::enableButtons() {

    ui->pushButton_start->setEnabled(false);
    ui->pushButton_add_robot->setEnabled(true);
    ui->pushButton_follow_mode->setEnabled(true);
    ui->pushButton_accept_commands->setEnabled(true);
    ui->pushButton_forward->setEnabled(true);
    ui->pushButton_right->setEnabled(true);
    ui->pushButton_left->setEnabled(true);
    ui->pushButton_back->setEnabled(true);
    ui->pushButton_stop->setEnabled(true);
}

void MainWindow::setSelectedRobot(bool changed) {
    if (changed) {
        ui->lineEdit_selected_robot->setText(QString::fromStdString(robotGroup.at(indexOfCurrentRobot)->getIpAddress()));
    }
}

void MainWindow::setIpAddress(std::string ipAddress) {

    this->ipAddress = ipAddress;
    // Ulozenie prave pridanej IP adresy do pola, aby som ich mohol vyuzit na spravne kreslenie dat z lidaru
    int numberOfDots = 0;
    std::string buf = "";

    for (size_t i = 0; i < ipAddress.size(); i++) {

        if (numberOfDots == 3) {
            buf.push_back(ipAddress[i]);
        }

        if (ipAddress[i] == ('.')) {
            numberOfDots++;
        }
    }

    if (buf.empty() == false) {
        MainWindow::usedRobotIps.push_back(std::stoi(buf));
    }   

    cameraAddress = httpString + ipAddress + portString + fileString;
    ui->lineEdit->clear();
}

int MainWindow::processThisRobot(TKobukiData robotdata, int address) {

    IP ipcka;
    ipcka.ip = address;
    // std::cout << "doslo odtialto robot callback" << (int)ipcka.ip2.a << std::endl;

    LaserMeasurement copyOfLaserData = copyOfLaserData1;

    if (robotGroup.empty() == false && usedRobotIps.empty() == false) {

        if (robotGroup.at(indexOfCurrentRobot)->getIpAddress().empty() == false) {
#ifndef SIMULATOR
            char a = robotGroup.at(indexOfCurrentRobot)->getIpAddress().string::at(10);
            char b = robotGroup.at(indexOfCurrentRobot)->getIpAddress().string::at(11);
            std::string ip (1, a);
            ip = ip + b;
#endif
#ifdef SIMULATOR
            std::string ip = "1";
#endif            
            if (std::stoi(ip) == usedRobotIps.at(0)) {
                copyOfLaserData = copyOfLaserData1;
            } else if (std::stoi(ip) == usedRobotIps.at(1)) {
                copyOfLaserData = copyOfLaserData2;
            } else if (std::stoi(ip) == usedRobotIps.at(2)) {
                copyOfLaserData = copyOfLaserData3;
            }

            if (std::stoi(ip) == (int)ipcka.ip2.a) {
                processThisRobotAllowed = true;

            } else {
                processThisRobotAllowed = false;
            }
        }
    }

    if (processThisRobotAllowed) {

        // ============================================================================================== SIMPLE ODOMETRY
        if (firstTime) {
            mLeftOld = robotdata.EncoderLeft;
            mRightOld = robotdata.EncoderRight;
            firstTime = false;
        }

        mLeftDelta = mLeftOld - robotdata.EncoderLeft;

        if (mLeftDelta > (65535)/2) {
            mLeftDelta = (65535 - mLeftOld) + robotdata.EncoderLeft;
            //std::cout << "IF DOPREDU m_left_delta: " << mLeftDelta << std::endl;

        } else if (mLeftDelta < (-65535)/2) {
            mLeftDelta = (65535 - robotdata.EncoderLeft) + mLeftOld;
            //std::cout << "IF DOZADU m_left_delta: " << mLeftDelta << std::endl;
        }

        mLeftOld = robotdata.EncoderLeft;
        meters = meters + (std::abs(mLeftDelta) * 0.000085292090497737556558);

        // ============================================================================================== COLLISION DETECTION
        if (robotGroup.at(indexOfCurrentRobot)->getAwakeState()) { // A robot has to be awake to avoid collisions
            shortestLidarDistance = 10000;
            for (int k = 0; k < copyOfLaserData.numberOfScans; k++) {

                lidarDist = copyOfLaserData.Data[k].scanDistance;

                if (lidarDist < shortestLidarDistance && lidarDist > 0.0) {
                    shortestLidarDistance = lidarDist;
                    shortestLidarAngle = copyOfLaserData.Data[k].scanAngle;
                }

                if (!collisionDetectedFront) {
                    if ((shortestLidarAngle >= 0 && shortestLidarAngle <= 70) || (shortestLidarAngle >= 290 && shortestLidarAngle <= 360)) {
                        if (shortestLidarDistance <= COLLISION_DETECTION_RANGE) {
                            std::cout << "INFO: Collision Detected at the front at an angle: " << shortestLidarAngle << " Stopping" << std::endl;

                            if (robotGroup.at(indexOfCurrentRobot)->getPreviousCommand() == "RIGHT") {
                                collisionRotateRight = false;
                                collisionRotateLeft = true;

                            } else if (robotGroup.at(indexOfCurrentRobot)->getPreviousCommand() == "LEFT") {
                                collisionRotateRight = true;
                                collisionRotateLeft = false;

                            } else {
                                collisionRotateRight = false;
                                collisionRotateLeft = false;
                            }

                            // STOP COMMAND
                            resetControlParameters(0.7);
                            resetBooleansStopCommand();
                            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");
                            collisionDetectedFront = true;
                        }
                    }
                }

                if (!collisionDetectedBack) {
                    if ((shortestLidarAngle >= 110 && shortestLidarAngle <= 250)) {
                        if (shortestLidarDistance <= COLLISION_DETECTION_RANGE) {
                            std::cout << "INFO: Collision Detected at the back at an angle: " << shortestLidarAngle << " Stopping" << std::endl;

                            // STOP COMMAND
                            resetControlParameters(0.7);
                            resetBooleansStopCommand();
                            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");
                            collisionDetectedBack = true;
                        }
                    }
                }
            }
        }       

        // Unblocks the robot when the actual command is different to the command that caused the collision
        // This kills the bug which made the robot move back and forth if it detected a collision
        if (robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "BACKWARD" && avoidedCollisionWithCommand == "FORWARD") {
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");

        } else if (robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FORWARD" && avoidedCollisionWithCommand == "BACKWARD") {
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");

        } else if ((robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FORWARD" && avoidedCollisionWithCommand == "FORWARD") ||
                   (robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "BACKWARD" && avoidedCollisionWithCommand == "BACKWARD") ||
                   (robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "RIGHT" || robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "LEFT" || robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FOLLOW_LEFT" || robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FOLLOW_RIGHT")) {
            avoidedCollisionWithCommand.clear();
        }        

        // ============================================================================================== FINISH TRANSLATION MOVEMENT
        if (meters >= 1 && !rotationFromLeft && !rotationFromRight) { // Stopping when command is FORWARD or BACKWARD
            if (evadingCollision) {
                avoidedCollisionWithCommand = robotGroup.at(indexOfCurrentRobot)->getCurrentCommand();
            }            

            resetCollisionParams();

            // STOP
            // Happens during normal operation
            if (!collisionRotateLeft && !collisionRotateRight) {
                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
                resetControlParameters(0);
                resetBooleansStopCommand();
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");

            // Happens when the robot detects a collision while performing RIGHT or LEFT command
            } else {
                robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 0);
                robotGroup.at(indexOfCurrentRobot)->setArcSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed(), 0);

                // if first rotation was right I go left
                if (!rotationReset && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && collisionRotateRight) {
                    resetControlParameters(0);
                    rotationReset = true;
                    firstGyroData = true;
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("RIGHT");

                // if first rotation was left I go right
                } else if (!rotationReset && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && collisionRotateLeft) {
                    resetControlParameters(0);
                    rotationReset = true;
                    firstGyroData = true;
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("LEFT");
                }
            }

        } else if (meters >= 1 && (rotationFromLeft || rotationFromRight)) { // Stops FORWARD movement after first rotation

            if (evadingCollision) {
                avoidedCollisionWithCommand = robotGroup.at(indexOfCurrentRobot)->getCurrentCommand();
            }

            resetCollisionParams();
            robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 0);
            robotGroup.at(indexOfCurrentRobot)->setArcSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed(), 0);

            // if first rotation was right I go left
            if (!rotationReset && metersReset && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && rotationFromRight) {
                resetControlParameters(0);
                rotationReset = true;
                firstGyroData = true;
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("LEFT");

            // if first rotation was left I go right
            } else if (!rotationReset && metersReset && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && rotationFromLeft) {
                resetControlParameters(0);
                rotationReset = true;
                firstGyroData = true;
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("RIGHT");
            }

        } else if (meters != 0 && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && !robotGroup.at(indexOfCurrentRobot)->getFollowMode()) { // Toto sa stane ked sa vykona LEFT alebo RIGHT
            robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
            resetBooleansStopCommand();
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");
        }

        // ============================================================================================== AUTOMATIC TRANSLATION AFTER COLLISION WAS DETECTED
        if (robotGroup.at(indexOfCurrentRobot)->getAwakeState()) {
            if (collisionDetectedFront && collisionDetectedBack) {

                // STOP COMMAND
                resetControlParameters(0);
                resetBooleansStopCommand();
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");

            } else if ((collisionDetectedFront && !collisionDetectedBack)) {
                evadingCollision = true;
                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("BACKWARD");

            } else if ((collisionDetectedBack && !collisionDetectedFront)) {
                evadingCollision = true;
                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FORWARD");
            }

            if ((collisionDetectedFront && collisionDetectedBack) && (mLeftOld == robotdata.EncoderLeft)) {
                std::cout << "INFO: Robot is not moving" << std::endl;
                resetCollisionParams();
                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
            }
        }

        // ============================================================================================== FINISH ROTATION MOVEMENT
        if (robotGroup.at(indexOfCurrentRobot)->getFollowMode() == false) { // Control Mode
            if (angle >= 120 && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == true && rotationReset && (!collisionRotateLeft && !collisionRotateRight)) { // Stops the second rotation during RIGHT or LEFT command

                // Po tom ako robot zrotuje naspat sa prestane vykonavat prikaz
                if (rotationReset && (rotationFromRight || rotationFromLeft)) {
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
                }

                robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 1);
                robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());

            } else if (angle >= 90 && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == true && (collisionRotateLeft || collisionRotateRight)) { // Stops the rotation after a collision

                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
                robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 1);
                robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());

                if (robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0) {
                    collisionRotateRight = false;
                    collisionRotateLeft = false;
                    resetBooleansStopCommand();
                }

            } else if (angle >= 90 && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == true && !rotationReset) { // Stops the first rotation durign RIGHT or LEFT command

                robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 1);
                robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());

                if (!metersReset && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0) { // Forward command
                    resetControlParameters(0);
                    metersReset = true;
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FORWARD");
                }

            } else if (angle != 0 && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == true) {
                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");

            } else if (angle >= 90 && robotGroup.at(indexOfCurrentRobot)->getActualSpeed() == 0 && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false) {
                angle = 0;
                resetBooleansStopCommand();
            }

        } else { // Follow mode
            if (angle >= 7 && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == true) { // Stops rotation
                robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
                robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(0);
                robotGroup.at(indexOfCurrentRobot)->setActualSpeed(0);

                resetControlParameters(0);
                resetBooleansStopCommand();
                robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");
            }
        }        

        // ============================================================================================== COMMANDS
        if (robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "STOP") {
            robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 0);
            robotGroup.at(indexOfCurrentRobot)->setArcSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed(), 0);

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FORWARD") {

            if (robotGroup.at(indexOfCurrentRobot)->getFollowMode()) {
                robotGroup.at(indexOfCurrentRobot)->ramp(MAX_TRANSLATION_SPEED_FOLLOW_MODE, 0, 0);
            } else {
                robotGroup.at(indexOfCurrentRobot)->ramp(MAX_TRANSLATION_SPEED_CONTROL_MODE, 0, 0);
            }

            robotGroup.at(indexOfCurrentRobot)->setArcSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed(), 0);

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "BACKWARD") {

            if (robotGroup.at(indexOfCurrentRobot)->getFollowMode()) {
                robotGroup.at(indexOfCurrentRobot)->ramp(-MAX_TRANSLATION_SPEED_FOLLOW_MODE, 0, 0);
            } else {
                robotGroup.at(indexOfCurrentRobot)->ramp(-MAX_TRANSLATION_SPEED_CONTROL_MODE, 0, 0);
            }

            robotGroup.at(indexOfCurrentRobot)->setArcSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed(), 0);

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "RIGHT") {

            if (firstGyroData) {
                oldAngle = robotdata.GyroAngle/100;
                firstGyroData = false;

            if (!rotationFromLeft) {
                    rotationFromRight = true;
                }
            }

            robotGroup.at(indexOfCurrentRobot)->ramp((-MAX_ROTATION_SPEED_CONTROL_MODE), 0, 1);
            robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());
            angleDelta = (oldAngle - (robotdata.GyroAngle/100));

            if (angleDelta < oldAngle) {
                angleDelta = (std::abs(oldAngle) - (robotdata.GyroAngle/100));
            }

            oldAngle = robotdata.GyroAngle/100;
            angle += angleDelta;
            std::cout << "Prejdeny uhol: " << angle << std::endl;

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "LEFT") {

            if (firstGyroData) {
                oldAngle = robotdata.GyroAngle/100;
                firstGyroData = false;

                if (!rotationFromRight) {
                    rotationFromLeft = true;
                }
            }

            robotGroup.at(indexOfCurrentRobot)->ramp((MAX_ROTATION_SPEED_CONTROL_MODE), 0, 1);
            robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());
            angleDelta = (robotdata.GyroAngle/100) - oldAngle;

            if (angleDelta < robotdata.GyroAngle/100) {
                angleDelta = std::abs((robotdata.GyroAngle/100)) - oldAngle;
            }

            oldAngle = robotdata.GyroAngle/100;
            angle += angleDelta;
            std::cout << "Prejdeny uhol: " << angle << std::endl;

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FOLLOW_LEFT") {

            if (firstGyroData) {
                oldAngle = robotdata.GyroAngle/100;
                firstGyroData = false;
            }

            robotGroup.at(indexOfCurrentRobot)->ramp((MAX_ROTATION_SPEED_FOLLOW_MODE), 0, 1);
            robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());
            angleDelta = (robotdata.GyroAngle/100) - oldAngle;

            if (angleDelta < robotdata.GyroAngle/100) {
                angleDelta = std::abs((robotdata.GyroAngle/100)) - oldAngle;
            }

            oldAngle = robotdata.GyroAngle/100;
            angle += angleDelta;

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FOLLOW_RIGHT") {

            if (firstGyroData) {
                oldAngle = robotdata.GyroAngle/100;
                firstGyroData = false;
            }

            robotGroup.at(indexOfCurrentRobot)->ramp((-MAX_ROTATION_SPEED_FOLLOW_MODE), 0, 1);
            robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());
            angleDelta = (oldAngle - (robotdata.GyroAngle/100));

            if (angleDelta < oldAngle) {
                angleDelta = (std::abs(oldAngle) - (robotdata.GyroAngle/100));
            }

            oldAngle = robotdata.GyroAngle/100;
            angle += angleDelta;

        } else if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false && robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "FOLLOW_RIGHT") {
            robotGroup.at(indexOfCurrentRobot)->ramp(0, 1, 1);
            robotGroup.at(indexOfCurrentRobot)->setRotationSpeed(robotGroup.at(indexOfCurrentRobot)->getActualSpeed());
        }

        // COUNTING NUMBER OF CALLBACKS THE ROBOT HAS NOT BEEN MOVING
        if (mLeftOld == robotdata.EncoderLeft) {
            numberOfCallbacksEncoderDataWasNotChanged++;
        }

        // RESET ACTUAL SPEED OF ROBOT WHEN STOPPED ABRUPTLY
        if (robotGroup.at(indexOfCurrentRobot)->getCurrentCommand() == "STOP" && (numberOfCallbacksEncoderDataWasNotChanged >= 25)) {
            std::cout << "INFO: Resetting robot's actual speed" << std::endl;
            resetControlParameters(0);
            robotGroup.at(indexOfCurrentRobot)->setActualSpeed(0);
            robotGroup.at(indexOfCurrentRobot)->setDoingGesture(false);
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("NULL");
            numberOfCallbacksEncoderDataWasNotChanged = 0;
        }        
    }

    /*
    if (this->data_counter % 5) {

        emit uiValuesChanged(this->robot_data.EncoderLeft, 11, 12);
    }

    this->data_counter++;
    */

    return 0;
}

int MainWindow::processThisLidar(LaserMeasurement laserData, int address) {

    IP ipcka;
    ipcka.ip = address;
    //std::cout << "doslo odtialto callback" << (int)ipcka.ip2.a << std::endl;

    if (usedRobotIps.empty() == false) {

        if ((int)ipcka.ip2.a == usedRobotIps.front()) {
            memcpy(&copyOfLaserData1, &laserData, sizeof(LaserMeasurement));
        }

        if (usedRobotIps.size() >= 2) {

            if ((int)ipcka.ip2.a == usedRobotIps.at(1)) {
                memcpy(&copyOfLaserData2, &laserData, sizeof(LaserMeasurement));
            }

        }

        if (usedRobotIps.size() == 3) {

            if ((int)ipcka.ip2.a == usedRobotIps.at(2)) {
                memcpy(&copyOfLaserData3, &laserData, sizeof(LaserMeasurement));
            }
        }
    }

    // tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    this->updateLaserPicture = 1;
    update(); //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

int MainWindow::processThisCamera(cv::Mat cameraData) {

    /*cameraData.copyTo(frame[(actIndex + 1) % 3]);
    actIndex = (actIndex + 1) % 3;
    this->update_laser_picture = 1;*/
    return 0;
}

void MainWindow::processThisMessage(sockaddr_in ske_si_me, sockaddr_in ske_si_other, sockaddr_in ske_si_posli, int ske_s, int ske_recv_len, int port) {

#ifdef _WIN32
    WSADATA wsaData = {0};
    int i_result = WSAStartup(MAKEWORD(2, 2), &wsaData); // Initialize Winsock
#else
#endif

    ske_slen = sizeof(ske_si_other);
    if ((ske_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {

    }

    char ske_broadcastene = 1;

#ifdef _WIN32
    DWORD timeout = 100;
    std::cout << setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout) << std::endl;
    std::cout << setsockopt(ske_s, SOL_SOCKET, SO_BROADCAST, &ske_broadcastene, sizeof(ske_broadcastene)) << std::endl;
#else
    setsockopt(ske_s, SOL_SOCKET, SO_BROADCAST, &ske_broadcastene, sizeof(ske_broadcastene));
#endif

    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(port);
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(port);
    ske_si_posli.sin_addr.s_addr = inet_addr(ipAddress.data()); //htonl(INADDR_BROADCAST);
    std::cout << ::bind(ske_s, (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) ) << std::endl;

    char buf[500];

    std::string receivedMessage;
    std::string robotIpAddress;
    std::string robotCommand;
    std::string robotFollowCommand;
    std::string msgBuffer;

    while(true) {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&buf, sizeof(buf), 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1) {
            std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }

        bool recordCommand = false;

        for (int i = 0; i < ske_recv_len; i++) {
               receivedMessage = receivedMessage + buf[i];

               if (isdigit(receivedMessage[i]) || receivedMessage[i] == '.') {
                   robotIpAddress = robotIpAddress + receivedMessage[i];

               } else if (receivedMessage[i-1] == ':') {
                   recordCommand = true;

               } else if (receivedMessage[i] == ';') {
                   recordCommand = false;

                   if (robotCommand.empty()) {
                       robotCommand = msgBuffer;
                   }

                   msgBuffer.clear();
               }

               if (recordCommand) {
                   msgBuffer = msgBuffer + receivedMessage[i];
               }
        }

        // updateSkeletonPicture=1;
        // std::cout<<"doslo " << buf << std::endl;
        // continue;

        robotFollowCommand = msgBuffer;

        //std::cout << "Robot IP: " << robotIpAddress << std::endl;
        std::cout << "Robot Command:" << robotCommand << std::endl;
        std::cout << "Robot Follow Command:" <<robotFollowCommand << std::endl;
        //std::cout << "Robot Buffer: " <<msgBuffer << std::endl;
        //std::cout << receivedMessage << endl;

        issueRobotCommand(robotIpAddress, robotCommand, robotFollowCommand);

        receivedMessage.clear();
        robotIpAddress.clear();
        robotCommand.clear();
        robotFollowCommand.clear();
        msgBuffer.clear();
    }

    std::cout << "koniec thread" << std::endl;
}

void MainWindow::issueRobotCommand(std::string robotIpAddress, std::string robotCommand, std::string robotFollowCommand) {

    bool commandAllowed = true;

    if (robotGroup.empty()) {
        return;
    }

    for (size_t i = 0; i < robotGroup.size(); i++) {

       if (robotIpAddress == robotGroup[i]->getIpAddress()) {
           indexOfCurrentRobot = robotGroup[i]->getRobotGroupIndex();
           commandAllowed = true;

           if (robotCommand == "WAKE_UP") {
               robotGroup.at(indexOfCurrentRobot)->setAwakeState(true);
               emit selectedRobotChanged(true);
           }
           break;

       } else {
           commandAllowed = false;
           robotGroup.at(i)->setAwakeState(false);
       }
    }

    if (commandAllowed == false) {
        std::cout << "INFO: No such robot id was found!" << endl;
        return;
    }

    if (robotCommand == "STOP") {
        if (!evadingCollision) {
            resetControlParameters(0);
            resetBooleansStopCommand();
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");
        }
    }

    if (robotGroup.at(indexOfCurrentRobot)->getAwakeState() == true) {
        if (robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false) {

            // Turning follow mode on/ off
            if (robotCommand == "FOLLOW_MODE") {
                robotGroup.at(indexOfCurrentRobot)->setFollowMode(true);

            } else if (robotCommand == "CONTROL_MODE") {
                robotGroup.at(indexOfCurrentRobot)->setFollowMode(false);
            }

            emit robotModesChanged(robotGroup.at(indexOfCurrentRobot)->getAwakeState(),
                                   robotGroup.at(indexOfCurrentRobot)->getFollowMode());

            if (robotGroup.at(indexOfCurrentRobot)->getFollowMode() == true) {
                // ====================================================================================== FOLLOW MODE
                if (robotFollowCommand == "FOLLOW_FORWARD") {
                    resetControlParameters(0.8);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FORWARD");

                } else if (robotFollowCommand == "FOLLOW_FORWARD_FAR") {
                    resetControlParameters(0.3);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FORWARD");

                } else if (robotFollowCommand == "FOLLOW_BACKWARD") {
                    resetControlParameters(0.3);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("BACKWARD");

                } else if (robotFollowCommand == "FOLLOW_LEFT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_LEFT");

                } else if (robotFollowCommand == "FOLLOW_RIGHT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_RIGHT");

                } else if (robotFollowCommand == "FOLLOW_BACKWARD_LEFT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_LEFT");

                } else if (robotFollowCommand == "FOLLOW_FORWARD_LEFT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_LEFT");

                } else if (robotFollowCommand == "FOLLOW_BACKWARD_RIGHT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_RIGHT");

                } else if (robotFollowCommand == "FOLLOW_FORWARD_RIGHT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_RIGHT");
                }

            } else if (robotGroup.at(indexOfCurrentRobot)->getFollowMode() == false) {
                // ====================================================================================== CONTROL MODE
                if (robotCommand == "FORWARD") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand(robotCommand);

                } else if (robotCommand == "BACKWARD") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand(robotCommand);

                } else if (robotCommand == "RIGHT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand(robotCommand);

                } else if (robotCommand == "LEFT") {
                    resetControlParameters(0);
                    robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
                    robotGroup.at(indexOfCurrentRobot)->setCurrentCommand(robotCommand);
                }
            }
        }
    }
}

void MainWindow::setIndexOfCurrentRobot(unsigned short int robotIndex) {
    this->indexOfCurrentRobot = robotIndex;
}

void MainWindow::addNewRobotToGroup(unsigned short int robotIndex, unsigned short int numberOfRobots) {

    MainWindow::robotGroup.insert(std::map<unsigned short int, Robot*>::value_type(robotIndex, new Robot()));
    MainWindow::robotGroup.at(robotIndex)->setLaserParameters(this->ipAddress, this->laserParametersLaserPortOut, this->laserParametersLaserPortIn, /*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar, this, std::placeholders::_1,std::placeholders::_2));
    MainWindow::robotGroup.at(robotIndex)->setRobotParameters(this->ipAddress, this->robotParametersLaserPortOut, this->robotParametersLaserPortIn, std::bind(&MainWindow::processThisRobot, this, std::placeholders::_1, std::placeholders::_2));
    MainWindow::robotGroup.at(robotIndex)->setCameraParameters(this->cameraAddress, std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setRobotGroupIndex(robotIndex);
    MainWindow::robotGroup.at(robotIndex)->setAwakeState(false);

    MainWindow::setIndexOfCurrentRobot(robotIndex);
    MainWindow::robotGroup.at(indexOfCurrentRobot)->robotStart();

    if (MainWindow::robotGroup.size() > 1 && !switchButtonWasEnabled) {
        ui->pushButton_switch_robot->setEnabled(true);        
        switchButtonWasEnabled = true;
    }
}

void MainWindow::on_pushButton_switch_robot_clicked() {

    // Bude to take tlacidlo, ktore pojde iba jednym smerom
    if (indexOfCurrentRobot < robotGroup.size() - 1) {
        indexOfCurrentRobot += 1;
    } else {
        indexOfCurrentRobot = 0;
    }

    this->resetBooleansStopCommand();
    this->resetCollisionParams();
    this->resetControlParameters(0);
    this->firstTime = true;

    emit robotModesChanged(robotGroup.at(indexOfCurrentRobot)->getAwakeState(),
                           robotGroup.at(indexOfCurrentRobot)->getFollowMode());

    emit selectedRobotChanged(true);
}

void MainWindow::on_pushButton_add_robot_clicked() {        

    if (!ui->lineEdit->text().isEmpty()) {

        IpReturnMessage msg = MainWindow::checkIpAddress(ui->lineEdit->text().toStdString());

        if (msg.ip_valid) {
            this->setIpAddress(ui->lineEdit->text().toStdString());
        } else {
            std::cout << msg.message << std::endl;
            ui->lineEdit->clear();
            return;
        }
    }

    while (indexOfCurrentRobot < robotGroup.size() - 1) {

        this->indexOfCurrentRobot += 1; // tuto iba zabezpecujem, aby bol ten index na max pred pridanim dalsieho robota
    }

    this->indexOfCurrentRobot += 1;

    // Porty sa menia iba v simulatore, kazdy robot ma rovnake porty
#ifdef SIMULATOR
    this->laserParametersLaserPortIn += 10;
    this->laserParametersLaserPortOut += 10;
    this->robotParametersLaserPortIn += 10;
    this->robotParametersLaserPortOut += 10;    
#endif

    MainWindow::addNewRobotToGroup(indexOfCurrentRobot, robotGroup.size() + 1);

    this->resetBooleansStopCommand();
    this->resetCollisionParams();
    this->resetControlParameters(0);
    this->firstTime = true;

    emit selectedRobotChanged(true);
    emit robotModesChanged(robotGroup.at(indexOfCurrentRobot)->getAwakeState(),
                           robotGroup.at(indexOfCurrentRobot)->getFollowMode());

    // Vypne lineEdit po pridani troch robotov
    if (this->usedRobotIps.size() == 3) {
        std::cout << "INFO: Sorry, you cannot add another robot. There already are " << this->usedRobotIps.size() << " robots enrolled." << std::endl;
        this->ui->lineEdit->setEnabled(false);
        this->ui->pushButton_add_robot->setEnabled(false);
    }
}

IpReturnMessage MainWindow::checkIpAddress(std::string ip) {

    IpReturnMessage return_data;

    // Nie je mozne pridat uz pouzivanu IPcku
    for (int i = 0; i < robotGroup.size(); i++) {
        if (robotGroup[i]->getIpAddress().compare(ip) == 0) {
            return_data.ip_valid = false;
            return_data.message = "INFO: You cannot connect the same robot twice!";
            return return_data;
        }
    }    

    if (validateIp(ip)) {
        return_data.ip_valid = true;
        return return_data;

    } else {
        return_data.message = "INFO: Ip address is not in correct format!";
        return_data.ip_valid = false;
        return return_data;
    }
}

void MainWindow::on_pushButton_start_clicked() { // start button

    // Check if user entered an ip address
    if (!ui->lineEdit->text().isEmpty()) {

        IpReturnMessage msg = MainWindow::checkIpAddress(ui->lineEdit->text().toStdString());

        if (msg.ip_valid) {
            this->setIpAddress(ui->lineEdit->text().toStdString());
        } else {
            std::cout << msg.message << std::endl;
            ui->lineEdit->clear();
            return;
        }
    }

    this->forwardSpeed = 0;
    this->rotationSpeed = 0;

    // tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    /*
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
    */
    connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double))); // pripaja signal k slotu
    connect(this, SIGNAL(startButtonPressed(bool)), this, SLOT(enableButtons()));
    connect(this, SIGNAL(robotModesChanged(bool, bool)), this, SLOT(setRobotModes(bool, bool)));
    connect(this, SIGNAL(selectedRobotChanged(bool)), this, SLOT(setSelectedRobot(bool)));

    MainWindow::addNewRobotToGroup(indexOfCurrentRobot, 1);

    instance = QJoysticks::getInstance();

    /* Enable the virtual joystick */
    /*  instance->setVirtualJoystickRange(1);
    instance->setVirtualJoystickEnabled(true);
    instance->setVirtualJoystickAxisSensibility(0.7);*/
    //instance->
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardSpeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationSpeed=-value*(3.14159/2.0);}}
    );

    emit startButtonPressed(true);
    emit selectedRobotChanged(true);
    emit robotModesChanged(robotGroup.at(indexOfCurrentRobot)->getAwakeState(),
                           robotGroup.at(indexOfCurrentRobot)->getFollowMode());
}

void MainWindow::on_pushButton_forward_clicked() { // forward

    if (robotGroup.at(indexOfCurrentRobot)->getAwakeState() && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false) {

        resetControlParameters(0);
        robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
        robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FORWARD");
    }
}

void MainWindow::on_pushButton_back_clicked() { // back

    if (robotGroup.at(indexOfCurrentRobot)->getAwakeState() && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false) {

        resetControlParameters(0);
        robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);
        robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("BACKWARD");
    }
}

void MainWindow::on_pushButton_left_clicked() { // left

    if (robotGroup.at(indexOfCurrentRobot)->getAwakeState() && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false) {

        resetControlParameters(0);
        robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);

        if (robotGroup.at(indexOfCurrentRobot)->getFollowMode()) {
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_LEFT");
        } else {
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("LEFT");
        }
    }
}

void MainWindow::on_pushButton_right_clicked() { // right

    if (robotGroup.at(indexOfCurrentRobot)->getAwakeState() && robotGroup.at(indexOfCurrentRobot)->getDoingGesture() == false) {

        resetControlParameters(0);
        robotGroup.at(indexOfCurrentRobot)->setDoingGesture(true);

        if (robotGroup.at(indexOfCurrentRobot)->getFollowMode()) {
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("FOLLOW_RIGHT");
        } else {
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("RIGHT");
        }
    }
}

void MainWindow::on_pushButton_stop_clicked() { // stop

    if (robotGroup.at(indexOfCurrentRobot)->getAwakeState()) {
        if (!evadingCollision) {
            resetControlParameters(0);
            resetBooleansStopCommand();
            robotGroup.at(indexOfCurrentRobot)->setCurrentCommand("STOP");
        }
    }
}

void MainWindow::on_pushButton_follow_mode_clicked() {

    if (!this->robotGroup.empty()) {

        if (robotGroup.at(this->indexOfCurrentRobot)->getFollowMode() == false) {
            robotGroup.at(this->indexOfCurrentRobot)->setFollowMode(true);
            ui->pushButton_follow_mode->setText("Turn off Follow Mode");            

        } else {
            robotGroup.at(this->indexOfCurrentRobot)->setFollowMode(false);
            ui->pushButton_follow_mode->setText("Turn on Follow Mode");
        }
    }

    emit robotModesChanged(this->robotGroup.at(this->indexOfCurrentRobot)->getAwakeState(),
                             this->robotGroup.at(this->indexOfCurrentRobot)->getFollowMode());
}

void MainWindow::on_pushButton_accept_commands_clicked() {

    if (!robotGroup.empty()) {

        for (size_t i = 0; i < robotGroup.size(); i++) {

            if (robotGroup.at(i) == robotGroup.at(indexOfCurrentRobot)) {
                if (robotGroup.at(indexOfCurrentRobot)->getAwakeState() == false) {
                    robotGroup.at(indexOfCurrentRobot)->setAwakeState(true);
                    ui->pushButton_accept_commands->setText("Put robot to sleep");

                } else {
                    robotGroup.at(indexOfCurrentRobot)->setAwakeState(false);
                    ui->pushButton_accept_commands->setText("Wake up robot");
                }

            } else {
                robotGroup.at(i)->setAwakeState(false);
            }
        }
    }

    emit robotModesChanged(robotGroup.at(indexOfCurrentRobot)->getAwakeState(),
                           robotGroup.at(indexOfCurrentRobot)->getFollowMode());
}

void MainWindow::on_pushButton_clicked() {

    // useCamera1 == false -> laser
    // useCamera1 == true -> camera

    // Kameru nebudem vyuzivat
    /*
    if (this->useCamera1 == true) {

        this->useCamera1 = false;
        ui->pushButton->setText("Use camera");

    } else {

        this->useCamera1 = true;
        ui->pushButton->setText("Use laser");
    }
    */
}

void MainWindow::get_new_frame() {

}
