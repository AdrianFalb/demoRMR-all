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
    this->http_string = "http://";
    this->file_string = "/stream.mjpg";

#ifndef SIMULATOR
    this->port_string = ":8000";
    this->ip_address = "192.168.1.";
#endif

#ifdef SIMULATOR
    this->port_string = ":8889"; // simulator
    this->ip_address = "127.0.0.1"; // Local host - default
#endif

    this->camera_address = this->http_string + this->ip_address + this->port_string + this->file_string; // Local host - Default
    this->index_of_current_robot = 0;

    this->laserParametersLaserPortOut = 52999;
    this->laserParametersLaserPortIn = 5299;
    this->robotParametersLaserPortOut = 53000;
    this->robotParametersLaserPortIn = 5300;

    ui->setupUi(this);
    this->data_counter = 0;
    this->act_index = -1;
    this->use_camera1 = false;

    this->disable_buttons();

    switch_button_was_enabled = false;
    ui->lineEdit->setText(QString::fromStdString(this->ip_address));

    auto message = std::bind(&MainWindow::process_this_message, this, std::placeholders::_1,
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

void MainWindow::reset_control_parameters(double m) {
    this->meters = m;
    this->angle = 0;
    this->number_of_callbacks_encoder_data_was_not_changed = 0;
}

void MainWindow::reset_booleans_stop_command() {
    this->meters_reset = false;
    this->rotation_reset = false;
    this->rotation_from_right = false;
    this->rotation_from_left = false;
    this->first_gyro_data = true;
}

void MainWindow::reset_collision_params() {
    this->collision_detected_front = false;
    this->collision_detected_back = false;    
    this->evading_collision = false;
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

    LaserMeasurement copy_of_laser_data = copy_of_laser_data1;

    if (this->robot_group.empty() == false && this->used_robot_ips.empty() == false) {

        if (this->robot_group.at(this->index_of_current_robot)->getIpAddress().empty() == false) {

#ifndef SIMULATOR
            char a = this->robot_group.at(this->index_of_current_robot)->getIpAddress().string::at(10);
            char b = this->robot_group.at(this->index_of_current_robot)->getIpAddress().string::at(11);
            std::string ip (1, a);
            ip = ip + b;
#endif
#ifdef SIMULATOR
            std::string ip = "1";
#endif
            // std::cout << "IP of currently selected robot: " << ip << std::endl;

            if (std::stoi(ip) == used_robot_ips.at(0)) {
                copy_of_laser_data = copy_of_laser_data1;
            } else if (std::stoi(ip) == used_robot_ips.at(1)) {
                copy_of_laser_data = copy_of_laser_data2;
            } else if (std::stoi(ip) == used_robot_ips.at(2)) {
                copy_of_laser_data = copy_of_laser_data3;
            }
        }
    }

    if (use_camera1 == true && act_index > -1) {

        std::cout<<act_index<<std::endl;
        QImage image = QImage((uchar*)frame[act_index].data, frame[act_index].cols, frame[act_index].rows, frame[act_index].step, QImage::Format_RGB888  );
        painter.drawImage(rect, image.rgbSwapped());

    } else {

        if (this->update_laser_picture == 1) {

            this->update_laser_picture = 0;
            painter.setPen(pero);

            // teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            // std::cout<<copyOfLaserData.numberOfScans<<std::endl;

            for (int k = 0; k < copy_of_laser_data.numberOfScans/*360*/; k++) {

                /*  int dist=rand()%500;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-k)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-k)*3.14159/180.0))+rect.topLeft().y();
                */

                int dist = copy_of_laser_data.Data[k].scanDistance / 20;
                int xp = rect.width() - (rect.width()/2 + dist*2*sin((360.0-copy_of_laser_data.Data[k].scanAngle)*3.14159/180.0)) + rect.topLeft().x();
                int yp = rect.height() - (rect.height()/2 + dist*2*cos((360.0-copy_of_laser_data.Data[k].scanAngle)*3.14159/180.0)) + rect.topLeft().y();
                if (rect.contains(xp, yp)) {
                    painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }
        }
    }
}

void MainWindow::set_ui_values(double robotX,double robotY,double robotFi) {
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::set_robot_modes(bool mode, bool power_mode) {
    if (mode) {
        ui->lineEdit_mode->setText("Awake state");
        ui->pushButton_accept_commands->setText("Put robot to sleep");
    } else {
        ui->lineEdit_mode->setText("Sleep state");
        ui->pushButton_accept_commands->setText("Wake up robot");
    }

    if (power_mode) {
        ui->lineEdit_power_mode->setText("Follow mode");
        ui->pushButton_follow_mode->setText("Turn off Follow Mode");
    } else {
        ui->lineEdit_power_mode->setText("Control mode");
        ui->pushButton_follow_mode->setText("Turn on Follow Mode");
    }
}

void MainWindow::disable_buttons() {

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

void MainWindow::enable_buttons() {

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

void MainWindow::set_selected_robot(bool changed) {
    if (changed) {
        ui->lineEdit_selected_robot->setText(QString::fromStdString(robot_group.at(index_of_current_robot)->getIpAddress()));
    }
}

void MainWindow::set_ip_address(std::string ip_address) {

    this->ip_address = ip_address;
    // Ulozenie prave pridanej IP adresy do pola, aby som ich mohol vyuzit na spravne kreslenie dat z lidaru
    int number_of_dots = 0;
    std::string buf = "";

    for (int i = 0; i < ip_address.size(); i++) {

        if (number_of_dots == 3) {
            buf.push_back(ip_address[i]);
        }

        if (ip_address[i] == ('.')) {
            number_of_dots++;
        }
    }

    if (buf.empty() == false) {
        MainWindow::used_robot_ips.push_back(std::stoi(buf));
    }   

    this->camera_address = this->http_string + this->ip_address + this->port_string + this->file_string;
    ui->lineEdit->clear();
}

int MainWindow::process_this_robot(TKobukiData robotdata, int address) {    

    IP ipcka;
    ipcka.ip = address;
    // std::cout << "doslo odtialto robot callback" << (int)ipcka.ip2.a << std::endl;

    LaserMeasurement copy_of_laser_data = copy_of_laser_data1;

    if (this->robot_group.empty() == false && this->used_robot_ips.empty() == false) {

        if (this->robot_group.at(this->index_of_current_robot)->getIpAddress().empty() == false) {
#ifndef SIMULATOR
            char a = this->robot_group.at(this->index_of_current_robot)->getIpAddress().string::at(10);
            char b = this->robot_group.at(this->index_of_current_robot)->getIpAddress().string::at(11);
            std::string ip (1, a);
            ip = ip + b;
#endif
#ifdef SIMULATOR
            std::string ip = "1";
#endif
            // std::cout << "IP of currently selected robot: " << ip << std::endl;

            if (std::stoi(ip) == used_robot_ips.at(0)) {
                copy_of_laser_data = copy_of_laser_data1;
            } else if (std::stoi(ip) == used_robot_ips.at(1)) {
                copy_of_laser_data = copy_of_laser_data2;
            } else if (std::stoi(ip) == used_robot_ips.at(2)) {
                copy_of_laser_data = copy_of_laser_data3;
            }

            if (std::stoi(ip) == (int)ipcka.ip2.a) {
                process_this_robot_allowed = true;

            } else {
                process_this_robot_allowed = false;
            }
        }
    }

    if (process_this_robot_allowed) {

        // ============================================================================================== SIMPLE ODOMETRY
        if (first_time) {
            m_left_old = robotdata.EncoderLeft;
            m_right_old = robotdata.EncoderRight;
            first_time = false;
        }

        m_left_delta = m_left_old - robotdata.EncoderLeft;

        if (m_left_delta > (65535)/2) {
            m_left_delta = (65535 - m_left_old) + robotdata.EncoderLeft;
            //std::cout << "IF DOPREDU m_left_delta: " << m_left_delta << std::endl;

        } else if (m_left_delta < (-65535)/2) {
            m_left_delta = (65535 - robotdata.EncoderLeft) + m_left_old;
            //std::cout << "IF DOZADU m_left_delta: " << m_left_delta << std::endl;
        }

        m_left_old = robotdata.EncoderLeft;
        meters = meters + (std::abs(m_left_delta) * 0.000085292090497737556558);

        // ============================================================================================== COLLISION DETECTION
        if (robot_group.at(index_of_current_robot)->get_awake_state()) { // A robot has to be awake to avoid collisions
            shortest_lidar_distance = 10000;
            for (int k = 0; k < copy_of_laser_data.numberOfScans; k++) {

                lidar_dist = copy_of_laser_data.Data[k].scanDistance;

                if (lidar_dist < shortest_lidar_distance && lidar_dist > 0.0) {
                    shortest_lidar_distance = lidar_dist;
                    shortest_lidar_angle = copy_of_laser_data.Data[k].scanAngle;
                }

                if (!collision_detected_front) {
                    if ((shortest_lidar_angle >= 0 && shortest_lidar_angle <= 70) || (shortest_lidar_angle >= 290 && shortest_lidar_angle <= 360)) {
                        if (shortest_lidar_distance <= COLLISION_DETECTION_RANGE) {
                            std::cout << "INFO: Collision Detected at the front at an angle: " << shortest_lidar_angle << " Stopping" << std::endl;

                            if (robot_group.at(index_of_current_robot)->get_previous_command() == "RIGHT") {
                                collision_rotate_right = false;
                                collision_rotate_left = true;

                            } else if (robot_group.at(index_of_current_robot)->get_previous_command() == "LEFT") {
                                collision_rotate_right = true;
                                collision_rotate_left = false;

                            } else {
                                collision_rotate_right = false;
                                collision_rotate_left = false;
                            }

                            // STOP COMMAND
                            this->reset_control_parameters(0.7);
                            this->reset_booleans_stop_command();
                            robot_group.at(this->index_of_current_robot)->set_current_command("STOP");
                            collision_detected_front = true;
                        }
                    }
                }

                if (!collision_detected_back) {
                    if ((shortest_lidar_angle >= 110 && shortest_lidar_angle <= 250)) {
                        if (shortest_lidar_distance <= COLLISION_DETECTION_RANGE) {
                            std::cout << "INFO: Collision Detected at the back at an angle: " << shortest_lidar_angle << " Stopping" << std::endl;

                            // STOP COMMAND
                            this->reset_control_parameters(0.7);
                            this->reset_booleans_stop_command();
                            robot_group.at(this->index_of_current_robot)->set_current_command("STOP");
                            collision_detected_back = true;
                        }
                    }
                }
            }
        }       

        // Unblocks the robot when the actual command is different to the command that caused the collision
        // This kills the bug which made the robot move back and forth if it detected a collision
        if (robot_group.at(index_of_current_robot)->get_current_command() == "BACKWARD" && avoided_collision_with_command == "FORWARD") {
            robot_group.at(index_of_current_robot)->set_current_command("STOP");

        } else if (robot_group.at(index_of_current_robot)->get_current_command() == "FORWARD" && avoided_collision_with_command == "BACKWARD") {
            robot_group.at(index_of_current_robot)->set_current_command("STOP");

        } else if ((robot_group.at(index_of_current_robot)->get_current_command() == "FORWARD" && avoided_collision_with_command == "FORWARD") ||
                   (robot_group.at(index_of_current_robot)->get_current_command() == "BACKWARD" && avoided_collision_with_command == "BACKWARD") ||
                   (robot_group.at(index_of_current_robot)->get_current_command() == "RIGHT" || robot_group.at(index_of_current_robot)->get_current_command() == "LEFT" || robot_group.at(index_of_current_robot)->get_current_command() == "FOLLOW_LEFT" || robot_group.at(index_of_current_robot)->get_current_command() == "FOLLOW_RIGHT")) {
            avoided_collision_with_command.clear();
        }        

        // ============================================================================================== FINISH TRANSLATION MOVEMENT
        if (meters >= 1 && !rotation_from_left && !rotation_from_right) { // Stopping when command is FORWARD or BACKWARD
            if (evading_collision) {
                avoided_collision_with_command = robot_group.at(this->index_of_current_robot)->get_current_command();
            }

            std::cout << "Collision left: " << collision_rotate_left << std::endl;
            std::cout << "Collision right: " << collision_rotate_right << std::endl;

            this->reset_collision_params();

            // STOP
            // Happens during normal operation
            if (!collision_rotate_left && !collision_rotate_right) {
                robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
                this->reset_control_parameters(0);
                this->reset_booleans_stop_command();
                robot_group.at(this->index_of_current_robot)->set_current_command("STOP");

            // Happens when the robot detects a collision while performing RIGHT or LEFT command
            } else {
                robot_group.at(this->index_of_current_robot)->ramp(0, 1, 0);
                robot_group.at(this->index_of_current_robot)->set_arc_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed(), 0);

                // if first rotation was right I go left
                if (!rotation_reset && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && collision_rotate_right) {
                    this->reset_control_parameters(0);
                    rotation_reset = true;
                    first_gyro_data = true;
                    robot_group.at(this->index_of_current_robot)->set_current_command("RIGHT");

                // if first rotation was left I go right
                } else if (!rotation_reset && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && collision_rotate_left) {
                    this->reset_control_parameters(0);
                    rotation_reset = true;
                    first_gyro_data = true;
                    robot_group.at(this->index_of_current_robot)->set_current_command("LEFT");
                }
            }

        } else if (meters >= 1 && (rotation_from_left || rotation_from_right)) { // Stops FORWARD movement after first rotation

            if (evading_collision) {
                avoided_collision_with_command = robot_group.at(this->index_of_current_robot)->get_current_command();
            }

            this->reset_collision_params();
            robot_group.at(this->index_of_current_robot)->ramp(0, 1, 0);
            robot_group.at(this->index_of_current_robot)->set_arc_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed(), 0);

            // if first rotation was right I go left
            if (!rotation_reset && meters_reset && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && rotation_from_right) {
                this->reset_control_parameters(0);
                rotation_reset = true;
                first_gyro_data = true;
                robot_group.at(this->index_of_current_robot)->set_current_command("LEFT");                

            // if first rotation was left I go right
            } else if (!rotation_reset && meters_reset && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && rotation_from_left) {
                this->reset_control_parameters(0);                
                rotation_reset = true;
                first_gyro_data = true;
                robot_group.at(this->index_of_current_robot)->set_current_command("RIGHT");                
            }

        } else if (meters != 0 && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && !robot_group.at(this->index_of_current_robot)->get_follow_mode()) { // Toto sa stane ked sa vykona LEFT alebo RIGHT
            robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
            this->reset_booleans_stop_command();
            robot_group.at(this->index_of_current_robot)->set_current_command("STOP");
        }

        // ============================================================================================== AUTOMATIC TRANSLATION AFTER COLLISION WAS DETECTED
        if (robot_group.at(index_of_current_robot)->get_awake_state()) {
            if (collision_detected_front && collision_detected_back) {

                // STOP COMMAND
                this->reset_control_parameters(0);
                this->reset_booleans_stop_command();
                robot_group.at(this->index_of_current_robot)->set_current_command("STOP");

            } else if ((collision_detected_front && !collision_detected_back)) {
                evading_collision = true;
                robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                robot_group.at(this->index_of_current_robot)->set_current_command("BACKWARD");

            } else if ((collision_detected_back && !collision_detected_front)) {
                evading_collision = true;
                robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                robot_group.at(this->index_of_current_robot)->set_current_command("FORWARD");
            }

            if ((collision_detected_front && collision_detected_back) && (m_left_old == robotdata.EncoderLeft)) {
                std::cout << "INFO: Robot is not moving" << std::endl;
                this->reset_collision_params();
                this->robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
            }
        }

        // ============================================================================================== FINISH ROTATION MOVEMENT
        if (robot_group.at(index_of_current_robot)->get_follow_mode() == false) { // Control Mode
            if (angle >= 120 && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == true && rotation_reset && (!collision_rotate_left && !collision_rotate_right)) { // Stops the second rotation during RIGHT or LEFT command

                // Po tom ako robot zrotuje naspat sa prestane vykonavat prikaz
                if (rotation_reset && (rotation_from_right || rotation_from_left)) {
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
                }

                robot_group.at(this->index_of_current_robot)->ramp(0, 1, 1);
                robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());

            } else if (angle >= 90 && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == true && (collision_rotate_left || collision_rotate_right)) { // Stops the rotation after a collision

                robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
                robot_group.at(this->index_of_current_robot)->ramp(0, 1, 1);
                robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());

                if (robot_group.at(index_of_current_robot)->get_actual_speed() == 0) {
                    collision_rotate_right = false;
                    collision_rotate_left = false;
                    this->reset_booleans_stop_command();
                }

            } else if (angle >= 90 && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == true && !rotation_reset) { // Stops the first rotation durign RIGHT or LEFT command

                robot_group.at(this->index_of_current_robot)->ramp(0, 1, 1);
                robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());

                if (!meters_reset && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0) { // Forward command
                    this->reset_control_parameters(0);
                    meters_reset = true;
                    robot_group.at(this->index_of_current_robot)->set_current_command("FORWARD");
                }

            } else if (angle != 0 && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == true) {
                robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
                robot_group.at(this->index_of_current_robot)->set_current_command("STOP");

            } else if (angle >= 90 && robot_group.at(this->index_of_current_robot)->get_actual_speed() == 0 && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == false) {
                angle = 0;
                this->reset_booleans_stop_command();
            }

        } else { // Follow mode
            if (angle >= 7 && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == true) { // Stops rotation
                robot_group.at(this->index_of_current_robot)->set_doing_gesture(false);
                robot_group.at(this->index_of_current_robot)->set_rotation_speed(0);
                robot_group.at(this->index_of_current_robot)->set_actual_speed(0);

                this->reset_control_parameters(0);
                this->reset_booleans_stop_command();
                robot_group.at(this->index_of_current_robot)->set_current_command("STOP");
            }
        }        

        // ============================================================================================== COMMANDS
        if (robot_group.at(this->index_of_current_robot)->get_current_command() == "STOP") {
            robot_group.at(this->index_of_current_robot)->ramp(0, 1, 0);
            robot_group.at(this->index_of_current_robot)->set_arc_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed(), 0);

        } else if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() && robot_group.at(this->index_of_current_robot)->get_current_command() == "FORWARD") {

            if (robot_group.at(index_of_current_robot)->get_follow_mode()) {
                robot_group.at(this->index_of_current_robot)->ramp(MAX_TRANSLATION_SPEED_FOLLOW_MODE, 0, 0);
            } else {
                robot_group.at(this->index_of_current_robot)->ramp(MAX_TRANSLATION_SPEED_CONTROL_MODE, 0, 0);
            }

            robot_group.at(this->index_of_current_robot)->set_arc_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed(), 0);

        } else if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() && robot_group.at(this->index_of_current_robot)->get_current_command() == "BACKWARD") {

            if (robot_group.at(index_of_current_robot)->get_follow_mode()) {
                robot_group.at(this->index_of_current_robot)->ramp(-MAX_TRANSLATION_SPEED_FOLLOW_MODE, 0, 0);
            } else {
                robot_group.at(this->index_of_current_robot)->ramp(-MAX_TRANSLATION_SPEED_CONTROL_MODE, 0, 0);
            }

            robot_group.at(this->index_of_current_robot)->set_arc_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed(), 0);

        } else if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() && robot_group.at(this->index_of_current_robot)->get_current_command() == "RIGHT") {

            if (first_gyro_data) {
                old_angle = robotdata.GyroAngle/100;
                first_gyro_data = false;

            if (!rotation_from_left) {
                    rotation_from_right = true;
                }
            }

            robot_group.at(this->index_of_current_robot)->ramp((-MAX_ROTATION_SPEED_CONTROL_MODE), 0, 1);
            robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());
            angle_delta = (old_angle - (robotdata.GyroAngle/100));

            if (angle_delta < old_angle) {
                angle_delta = (std::abs(old_angle) - (robotdata.GyroAngle/100));
            }

            old_angle = robotdata.GyroAngle/100;
            angle += angle_delta;
            std::cout << "Prejdeny uhol: " << angle << std::endl;

        } else if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() && robot_group.at(this->index_of_current_robot)->get_current_command() == "LEFT") {

            if (first_gyro_data) {
                old_angle = robotdata.GyroAngle/100;
                first_gyro_data = false;

                if (!rotation_from_right) {
                    rotation_from_left = true;
                }
            }

            robot_group.at(this->index_of_current_robot)->ramp((MAX_ROTATION_SPEED_CONTROL_MODE), 0, 1);
            robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());
            angle_delta = (robotdata.GyroAngle/100) - old_angle;

            if (angle_delta < robotdata.GyroAngle/100) {
                angle_delta = std::abs((robotdata.GyroAngle/100)) - old_angle;
            }

            old_angle = robotdata.GyroAngle/100;
            angle += angle_delta;
            std::cout << "Prejdeny uhol: " << angle << std::endl;

        } else if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() && robot_group.at(this->index_of_current_robot)->get_current_command() == "FOLLOW_LEFT") {

            if (first_gyro_data) {
                old_angle = robotdata.GyroAngle/100;
                first_gyro_data = false;
            }

            robot_group.at(this->index_of_current_robot)->ramp((MAX_ROTATION_SPEED_FOLLOW_MODE), 0, 1);
            robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());
            angle_delta = (robotdata.GyroAngle/100) - old_angle;

            if (angle_delta < robotdata.GyroAngle/100) {
                angle_delta = std::abs((robotdata.GyroAngle/100)) - old_angle;
            }

            old_angle = robotdata.GyroAngle/100;
            angle += angle_delta;

        } else if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() && robot_group.at(this->index_of_current_robot)->get_current_command() == "FOLLOW_RIGHT") {

            if (first_gyro_data) {
                old_angle = robotdata.GyroAngle/100;
                first_gyro_data = false;
            }

            robot_group.at(this->index_of_current_robot)->ramp((-MAX_ROTATION_SPEED_FOLLOW_MODE), 0, 1);
            robot_group.at(this->index_of_current_robot)->set_rotation_speed(robot_group.at(this->index_of_current_robot)->get_actual_speed());
            angle_delta = (old_angle - (robotdata.GyroAngle/100));

            if (angle_delta < old_angle) {
                angle_delta = (std::abs(old_angle) - (robotdata.GyroAngle/100));
            }

            old_angle = robotdata.GyroAngle/100;
            angle += angle_delta;
        }

        // COUNTING NUMBER OF CALLBACKS THE ROBOT HAS NOT BEEN MOVING
        if (m_left_old == robotdata.EncoderLeft) {
            number_of_callbacks_encoder_data_was_not_changed++;
        }

        // RESET ACTUAL SPEED OF ROBOT WHEN STOPPED ABRUPTLY
        if (this->robot_group.at(index_of_current_robot)->get_current_command() == "STOP" && (number_of_callbacks_encoder_data_was_not_changed >= 25)) {
            std::cout << "INFO: Resetting robot's actual speed" << std::endl;
            this->reset_control_parameters(0);
            this->robot_group.at(index_of_current_robot)->set_actual_speed(0);
            this->robot_group.at(index_of_current_robot)->set_doing_gesture(false);
            this->robot_group.at(index_of_current_robot)->set_current_command("NULL");
            number_of_callbacks_encoder_data_was_not_changed = 0;
        }        
    }

    /*
    if (this->data_counter % 5) {

        emit ui_values_changed(this->robot_data.EncoderLeft, 11, 12);
    }

    this->data_counter++;
    */

    return 0;
}

int MainWindow::process_this_lidar(LaserMeasurement laserData, int address) {

    IP ipcka;
    ipcka.ip = address;
    //std::cout << "doslo odtialto callback" << (int)ipcka.ip2.a << std::endl;

    if (this->used_robot_ips.empty() == false) {

        if ((int)ipcka.ip2.a == used_robot_ips.front()) {
            memcpy(&copy_of_laser_data1, &laserData, sizeof(LaserMeasurement));
        }

        if (this->used_robot_ips.size() >= 2) {

            if ((int)ipcka.ip2.a == used_robot_ips.at(1)) {
                memcpy(&copy_of_laser_data2, &laserData, sizeof(LaserMeasurement));
            }

        }

        if (this->used_robot_ips.size() == 3) {

            if ((int)ipcka.ip2.a == used_robot_ips.at(2)) {
                memcpy(&copy_of_laser_data3, &laserData, sizeof(LaserMeasurement));
            }
        }
    }

    // tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    this->update_laser_picture = 1;
    update(); //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

int MainWindow::process_this_camera(cv::Mat cameraData) {

    /*cameraData.copyTo(frame[(act_index + 1) % 3]);
    act_index = (act_index + 1) % 3;
    this->update_laser_picture = 1;*/
    return 0;
}

void MainWindow::process_this_message(sockaddr_in ske_si_me, sockaddr_in ske_si_other, sockaddr_in ske_si_posli, int ske_s, int ske_recv_len, int port) {

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
    ske_si_posli.sin_addr.s_addr = inet_addr(ip_address.data()); //htonl(INADDR_BROADCAST);
    std::cout << ::bind(ske_s, (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) ) << std::endl;

    char buf[500];

    std::string received_message;
    std::string robot_ip_address;
    std::string robot_command;
    std::string robot_follow_command;
    std::string msg_buffer;

    while(true) {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&buf, sizeof(buf), 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1) {
            std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }

        bool record_command = false;        

        for (int i = 0; i < ske_recv_len; i++) {
               received_message = received_message + buf[i];

               if (isdigit(received_message[i]) || received_message[i] == '.') {
                   robot_ip_address = robot_ip_address + received_message[i];

               } else if (received_message[i-1] == ':') {
                   record_command = true;

               } else if (received_message[i] == ';') {
                   record_command = false;

                   if (robot_command.empty()) {
                       robot_command = msg_buffer;
                   }

                   msg_buffer.clear();
               }

               if (record_command) {
                   msg_buffer = msg_buffer + received_message[i];
               }
        }

        // updateSkeletonPicture=1;
        // std::cout<<"doslo " << buf << std::endl;
        // continue;

        robot_follow_command = msg_buffer;

        //std::cout << "Robot IP: " << robot_ip_address << std::endl;
        std::cout << "Robot Command:" << robot_command << std::endl;
        std::cout << "Robot Follow Command:" <<robot_follow_command << std::endl;
        //std::cout << "Robot Buffer: " <<msg_buffer << std::endl;
        //std::cout << received_message << endl;

        this->issue_robot_command(robot_ip_address, robot_command, robot_follow_command);

        received_message.clear();
        robot_ip_address.clear();
        robot_command.clear();
        robot_follow_command.clear();
        msg_buffer.clear();
    }

    std::cout << "koniec thread" << std::endl;
}

void MainWindow::issue_robot_command(std::string robot_ip_address, std::string robot_command, std::string robot_follow_command) {

    bool command_allowed = true;

    if (this->robot_group.empty()) {
        return;
    }

    for (unsigned short int i = 0; i < robot_group.size(); i++) {

       if (robot_ip_address == robot_group[i]->getIpAddress()) {
           this->index_of_current_robot = robot_group[i]->get_my_robot_group_index();
           command_allowed = true;

           if (robot_command == "WAKE_UP") {
               robot_group.at(this->index_of_current_robot)->set_awake_state(true);
               emit selected_robot_changed(true);
           }
           break;

       } else {
           command_allowed = false;
           robot_group.at(i)->set_awake_state(false);
       }
    }

    if (command_allowed == false) {
        std::cout << "No such robot id was found!" << endl;
        return;
    }

    if (robot_command == "STOP") {
        if (!evading_collision) {
            this->reset_control_parameters(0);
            this->reset_booleans_stop_command();
            robot_group.at(this->index_of_current_robot)->set_current_command("STOP");
        }
    }

    if (robot_group.at(this->index_of_current_robot)->get_awake_state() == true) {
        if (robot_group.at(this->index_of_current_robot)->get_doing_gesture() == false) {

            // Turning follow mode on/ off
            if (robot_command == "FOLLOW_MODE") {
                robot_group.at(this->index_of_current_robot)->set_follow_mode(true);

            } else if (robot_command == "CONTROL_MODE") {
                robot_group.at(this->index_of_current_robot)->set_follow_mode(false);
            }

            emit robot_modes_changed(this->robot_group.at(this->index_of_current_robot)->get_awake_state(),
                                    this->robot_group.at(this->index_of_current_robot)->get_follow_mode());

            if (robot_group.at(this->index_of_current_robot)->get_follow_mode() == true) {
                // ====================================================================================== FOLLOW MODE
                if (robot_follow_command == "FOLLOW_FORWARD") {                    
                    this->reset_control_parameters(0.8);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("FORWARD");

                } else if (robot_follow_command == "FOLLOW_FORWARD_FAR") {
                    this->reset_control_parameters(0.3);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("FORWARD");

                } else if (robot_follow_command == "FOLLOW_BACKWARD") {
                    this->reset_control_parameters(0.3);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("BACKWARD");

                } else if (robot_follow_command == "FOLLOW_LEFT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command(robot_follow_command);

                } else if (robot_follow_command == "FOLLOW_RIGHT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command(robot_follow_command);

                } else if (robot_follow_command == "FOLLOW_BACKWARD_LEFT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("FOLLOW_LEFT");

                } else if (robot_follow_command == "FOLLOW_FORWARD_LEFT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("FOLLOW_LEFT");

                } else if (robot_follow_command == "FOLLOW_BACKWARD_RIGHT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("FOLLOW_RIGHT");

                } else if (robot_follow_command == "FOLLOW_FORWARD_RIGHT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command("FOLLOW_RIGHT");
                }

            } else if (robot_group.at(this->index_of_current_robot)->get_follow_mode() == false) {
                // ====================================================================================== CONTROL MODE
                if (robot_command == "FORWARD") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command(robot_command);

                } else if (robot_command == "BACKWARD") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command(robot_command);

                } else if (robot_command == "RIGHT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command(robot_command);

                } else if (robot_command == "LEFT") {
                    this->reset_control_parameters(0);
                    robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
                    robot_group.at(this->index_of_current_robot)->set_current_command(robot_command);
                }
            }
        }
    }
}

void MainWindow::set_index_of_current_robot(unsigned short int robot_index) {
    this->index_of_current_robot = robot_index;
}

void MainWindow::add_new_robot_to_group(unsigned short int robot_index, unsigned short int number_of_robots) {

    MainWindow::robot_group.insert(std::map<unsigned short int, Robot*>::value_type(robot_index, new Robot()));
    MainWindow::robot_group.at(robot_index)->set_laser_parameters(this->ip_address, this->laserParametersLaserPortOut, this->laserParametersLaserPortIn, /*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::process_this_lidar, this, std::placeholders::_1,std::placeholders::_2));
    MainWindow::robot_group.at(robot_index)->set_robot_parameters(this->ip_address, this->robotParametersLaserPortOut, this->robotParametersLaserPortIn, std::bind(&MainWindow::process_this_robot, this, std::placeholders::_1, std::placeholders::_2));
    MainWindow::robot_group.at(robot_index)->set_camera_parameters(this->camera_address, std::bind(&MainWindow::process_this_camera, this, std::placeholders::_1));
    MainWindow::robot_group.at(robot_index)->set_my_robot_group_index(robot_index);
    MainWindow::robot_group.at(robot_index)->set_awake_state(false);

    MainWindow::set_index_of_current_robot(robot_index);
    MainWindow::robot_group.at(this->index_of_current_robot)->robot_start();

    if (MainWindow::robot_group.size() > 1 && !switch_button_was_enabled) {
        ui->pushButton_switch_robot->setEnabled(true);        
        switch_button_was_enabled = true;
    }
}

void MainWindow::on_pushButton_switch_robot_clicked() {

    // Bude to take tlacidlo, ktore pojde iba jednym smerom
    if (this->index_of_current_robot < this->robot_group.size() - 1) {
        this->index_of_current_robot += 1;
    } else {
        this->index_of_current_robot = 0;
    }

    this->reset_booleans_stop_command();
    this->reset_collision_params();
    this->reset_control_parameters(0);
    this->first_time = true;    

    emit robot_modes_changed(this->robot_group.at(this->index_of_current_robot)->get_awake_state(),
                             this->robot_group.at(this->index_of_current_robot)->get_follow_mode());    

    emit selected_robot_changed(true);
}

void MainWindow::on_pushButton_add_robot_clicked() {        

    if (!ui->lineEdit->text().isEmpty()) {

        IpReturnMessage msg = MainWindow::check_ip_address(ui->lineEdit->text().toStdString());

        if (msg.ip_valid) {
            this->set_ip_address(ui->lineEdit->text().toStdString());
        } else {
            std::cout << msg.message << std::endl;
            ui->lineEdit->clear();
            return;
        }
    }

    while (this->index_of_current_robot < this->robot_group.size() - 1) {

        this->index_of_current_robot += 1; // tuto iba zabezpecujem, aby bol ten index na max pred pridanim dalsieho robota
    }

    this->index_of_current_robot += 1;    

    // Porty sa menia iba v simulatore, kazdy robot ma rovnake porty
#ifdef SIMULATOR
    this->laserParametersLaserPortIn += 10;
    this->laserParametersLaserPortOut += 10;
    this->robotParametersLaserPortIn += 10;
    this->robotParametersLaserPortOut += 10;    
#endif

    MainWindow::add_new_robot_to_group(this->index_of_current_robot, this->robot_group.size() + 1);

    this->reset_booleans_stop_command();
    this->reset_collision_params();
    this->reset_control_parameters(0);
    this->first_time = true;    

    emit selected_robot_changed(true);
    emit robot_modes_changed(this->robot_group.at(this->index_of_current_robot)->get_awake_state(),
                             this->robot_group.at(this->index_of_current_robot)->get_follow_mode());

    // Vypne lineEdit po pridani troch robotov
    if (this->used_robot_ips.size() == 3) {
        std::cout << "Sorry, you cannot add another robot. There already are " << this->used_robot_ips.size() << " robots being controlled." << std::endl;
        this->ui->lineEdit->setEnabled(false);
        this->ui->pushButton_add_robot->setEnabled(false);
    }
}

IpReturnMessage MainWindow::check_ip_address(std::string ip) {

    IpReturnMessage return_data;

    // Nie je mozne pridat uz pouzivanu IPcku
    for (int i = 0; i < this->robot_group.size(); i++) {
        if (this->robot_group[i]->getIpAddress().compare(ip) == 0) {
            return_data.ip_valid = false;
            return_data.message = "You cannot connect the same robot twice!";
            return return_data;
        }
    }    

    if (validate_ip(ip)) {
        return_data.ip_valid = true;
        return return_data;

    } else {
        return_data.message = "Ip address is not in correct format!";
        return_data.ip_valid = false;
        return return_data;
    }
}

void MainWindow::on_pushButton_start_clicked() { // start button

    // Check if user entered an ip address
    if (!ui->lineEdit->text().isEmpty()) {

        IpReturnMessage msg = MainWindow::check_ip_address(ui->lineEdit->text().toStdString());

        if (msg.ip_valid) {
            this->set_ip_address(ui->lineEdit->text().toStdString());
        } else {
            std::cout << msg.message << std::endl;
            ui->lineEdit->clear();
            return;
        }
    }

    this->forward_speed = 0;
    this->rotation_speed = 0;

    // tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    /*
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
    */
    connect(this, SIGNAL(ui_values_changed(double, double, double)), this, SLOT(set_ui_values(double, double, double))); // pripaja signal k slotu
    connect(this, SIGNAL(start_button_pressed(bool)), this, SLOT(enable_buttons()));
    connect(this, SIGNAL(robot_modes_changed(bool, bool)), this, SLOT(set_robot_modes(bool, bool)));
    connect(this, SIGNAL(selected_robot_changed(bool)), this, SLOT(set_selected_robot(bool)));

    MainWindow::add_new_robot_to_group(this->index_of_current_robot, 1);

    instance = QJoysticks::getInstance();

    /* Enable the virtual joystick */
    /*  instance->setVirtualJoystickRange(1);
    instance->setVirtualJoystickEnabled(true);
    instance->setVirtualJoystickAxisSensibility(0.7);*/
    //instance->
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forward_speed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotation_speed=-value*(3.14159/2.0);}}
    );

    emit start_button_pressed(true);
    emit selected_robot_changed(true);
    emit robot_modes_changed(this->robot_group.at(this->index_of_current_robot)->get_awake_state(),
                             this->robot_group.at(this->index_of_current_robot)->get_follow_mode());
}

void MainWindow::on_pushButton_forward_clicked() { // forward

    if (robot_group.at(this->index_of_current_robot)->get_awake_state() && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == false) {

        this->reset_control_parameters(0);
        robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
        robot_group.at(this->index_of_current_robot)->set_current_command("FORWARD");        
    }
}

void MainWindow::on_pushButton_back_clicked() { // back

    if (robot_group.at(this->index_of_current_robot)->get_awake_state() && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == false) {

        this->reset_control_parameters(0);
        robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);
        robot_group.at(this->index_of_current_robot)->set_current_command("BACKWARD");        
    }
}

void MainWindow::on_pushButton_left_clicked() { // left

    if (robot_group.at(this->index_of_current_robot)->get_awake_state() && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == false) {

        this->reset_control_parameters(0);
        robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);

        if (robot_group.at(index_of_current_robot)->get_follow_mode()) {
            robot_group.at(this->index_of_current_robot)->set_current_command("FOLLOW_LEFT");
        } else {
            robot_group.at(this->index_of_current_robot)->set_current_command("LEFT");
        }
    }
}

void MainWindow::on_pushButton_right_clicked() { // right

    if (robot_group.at(this->index_of_current_robot)->get_awake_state() && robot_group.at(this->index_of_current_robot)->get_doing_gesture() == false) {

        this->reset_control_parameters(0);
        robot_group.at(this->index_of_current_robot)->set_doing_gesture(true);        

        if (robot_group.at(index_of_current_robot)->get_follow_mode()) {
            robot_group.at(this->index_of_current_robot)->set_current_command("FOLLOW_RIGHT");
        } else {
            robot_group.at(this->index_of_current_robot)->set_current_command("RIGHT");
        }
    }
}

void MainWindow::on_pushButton_stop_clicked() { // stop

    if (!evading_collision) {
        this->reset_control_parameters(0);
        this->reset_booleans_stop_command();
        robot_group.at(this->index_of_current_robot)->set_current_command("STOP");
    }
}

void MainWindow::on_pushButton_follow_mode_clicked() {

    if (!this->robot_group.empty()) {

        if (robot_group.at(this->index_of_current_robot)->get_follow_mode() == false) {
            robot_group.at(this->index_of_current_robot)->set_follow_mode(true);
            ui->pushButton_follow_mode->setText("Turn off Follow Mode");            

        } else {
            robot_group.at(this->index_of_current_robot)->set_follow_mode(false);
            ui->pushButton_follow_mode->setText("Turn on Follow Mode");
        }
    }

    emit robot_modes_changed(this->robot_group.at(this->index_of_current_robot)->get_awake_state(),
                             this->robot_group.at(this->index_of_current_robot)->get_follow_mode());
}

void MainWindow::on_pushButton_accept_commands_clicked() {

    if (!this->robot_group.empty()) {

        if (robot_group.at(this->index_of_current_robot)->get_awake_state() == false) {
            robot_group.at(this->index_of_current_robot)->set_awake_state(true);
            ui->pushButton_accept_commands->setText("Put robot to sleep");            

        } else {
            robot_group.at(this->index_of_current_robot)->set_awake_state(false);
            ui->pushButton_accept_commands->setText("Wake up robot");            
        }
    }

    emit robot_modes_changed(this->robot_group.at(this->index_of_current_robot)->get_awake_state(),
                             this->robot_group.at(this->index_of_current_robot)->get_follow_mode());
}

void MainWindow::on_pushButton_clicked() {

    // use_camera1 == false -> laser
    // use_camera1 == true -> camera

    // Kameru nebudem vyuzivat
    /*
    if (this->use_camera1 == true) {

        this->use_camera1 = false;
        ui->pushButton->setText("Use camera");

    } else {

        this->use_camera1 = true;
        ui->pushButton->setText("Use laser");
    }
    */
}

void MainWindow::get_new_frame() {

}
