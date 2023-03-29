#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#ifdef _WIN32
    #include <windows.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
#endif

union IP {
    unsigned int ip;
    struct {
      unsigned char d;
      unsigned char c;
      unsigned char b;
      unsigned char a;
    } ip2;
};

MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui(new Ui::MainWindow) {   

    // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    this->http_string = "http://";
    this->file_string = "/stream.mjpg";
    this->port_string = ":8000";
    //this->port_string = ":8889"; // simulator

    //this->ip_address = "127.0.0.1"; // Local host - default
    this->ip_address = "192.168.1.";
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

    this->data_counter = 0;

    ui->pushButton_add_robot->setEnabled(false);
    ui->pushButton_switch_robot->setEnabled(false);
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

void MainWindow::paintEvent(QPaintEvent *event) {

    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20, 120, 700, 500);
    rect = ui->frame->geometry();
    rect.translate(0, 15);
    painter.drawRect(rect);

    LaserMeasurement copy_of_laser_data = copy_of_laser_data1;

    if (this->robot_group.empty() == false && this->used_robot_ips.empty() == false) {

        if (this->robot_group.at(this->index_of_current_robot)->getIpAddress().empty() == false) {

            char a = this->robot_group.at(this->index_of_current_robot)->getIpAddress().string::at(10);
            char b = this->robot_group.at(this->index_of_current_robot)->getIpAddress().string::at(11);
            std::string ip (1, a);
            ip = ip + b;

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

void MainWindow::setUiValues(double robotX,double robotY,double robotFi) {

     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::setButtonStates() {

    ui->pushButton_9->setEnabled(false);
    ui->pushButton_add_robot->setEnabled(true);
    ui->pushButton_switch_robot->setEnabled(true);    
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
        /*
        for (int i = 0; i < used_robot_ips.size(); i++) {
            std::cout << used_robot_ips[i] << std::endl;
        }*/

    }

    //

    this->camera_address = this->http_string + this->ip_address + this->port_string + this->file_string;
    ui->lineEdit->clear();
}

int MainWindow::process_this_robot(TKobukiData robotdata,int address) {

    IP ipcka;
    ipcka.ip = address;
    //std::cout << "doslo odtialto robot callback" << (int)ipcka.ip2.a << std::endl;

    // Joystick

    /*
    if (this->forwardspeed == 0 && this->rotationspeed != 0) {push_back

        robotGroup.at(0)->setRotationSpeed(this->rotationspeed);

    } else if (this->forwardspeed != 0 && this->rotationspeed == 0) {break;

        robotGroup.at(0)->setTranslationSpeed(this->forwardspeed);

    } else if ((this->forwardspeed != 0 && this->rotationspeed != 0)) {

        robotGroup.at(0)->setArcSpeed(this->forwardspeed, this->forwardspeed/this->rotationspeed);
if (this->used_robot_ips.size() >= 2) {
    } else {

        robotGroup.at(0)->setTranslationSpeed(0);
    }
    */

    if (this->data_counter % 5) {

        emit uiValuesChanged(this->robot_data.EncoderLeft, 11, 12);
    }

    this->data_counter++;
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

    cameraData.copyTo(frame[(act_index + 1) % 3]);
    act_index = (act_index + 1) % 3;
    this->update_laser_picture = 1;
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
               } else if (received_message[i - 1] == ':' && !isdigit(received_message[i])) {
                   record_command = true;
               }

               if (record_command) {
                   robot_command = robot_command + received_message[i];
               }
        }

        // updateSkeletonPicture=1;
        // std::cout<<"doslo " << buf << std::endl;
        // continue;

        std::cout << robot_ip_address << endl;
        std::cout << robot_command << endl;
        // std::cout << received_message << endl;

        this->issue_robot_command(robot_ip_address, robot_command);

        received_message.clear();
        robot_ip_address.clear();
        robot_command.clear();
    }

    std::cout << "koniec thread" << std::endl;
}

void MainWindow::issue_robot_command(std::string robot_ip_address, std::string robot_command) {

    bool command_allowed = true;

    for(unsigned short int i = 0; i < robot_group.size(); i++) {

       if (robot_ip_address == robot_group[i]->getIpAddress()) {
           this->index_of_current_robot = robot_group[i]->getMyRobotGroupIndex();
           command_allowed = true;

           if (robot_command == "WAKE_UP") {
               robot_group.at(this->index_of_current_robot)->set_accept_commands(true);
           }
           break;

       } else {
           command_allowed = false;
           robot_group.at(i)->set_accept_commands(false);
       }
    }

    if (command_allowed == false) {
        std::cout << "No such robot id was found!" << endl;
        return;
    }

    if (robot_command == "STOP" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setArcSpeed(0, 0);

    } else if (robot_command == "FORWARD" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setArcSpeed(150, 0);

    } else if (robot_command == "BACKWARD" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setArcSpeed(-150, 0);

    } else if (robot_command == "RIGHT" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setRotationSpeed((-1.5707/4));

    } else if (robot_command == "LEFT" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setRotationSpeed((1.5707/4));

    } else if (robot_command == "OPERATOR_RIGHT" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setRotationSpeed((-1.5707/4));

    } else if (robot_command == "OPERATOR_LEFT" && robot_group.at(this->index_of_current_robot)->get_accept_commands() == true) {
        robot_group.at(this->index_of_current_robot)->setRotationSpeed((1.5707/4));
    }
}

void MainWindow::set_index_of_current_robot(unsigned short int robot_index) {

    this->index_of_current_robot = robot_index;

}

void MainWindow::add_new_robot_to_group(unsigned short int robot_index, unsigned short int number_of_robots) {

    //MainWindow::robotGroup.resize(number_of_robots, new Robot());
    MainWindow::robot_group.insert(std::map<unsigned short int, Robot*>::value_type(robot_index, new Robot()));
    MainWindow::robot_group.at(robot_index)->setLaserParameters(this->ip_address, this->laserParametersLaserPortOut, this->laserParametersLaserPortIn, /*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::process_this_lidar, this, std::placeholders::_1,std::placeholders::_2));
    MainWindow::robot_group.at(robot_index)->setRobotParameters(this->ip_address, this->robotParametersLaserPortOut, this->robotParametersLaserPortIn, std::bind(&MainWindow::process_this_robot, this, std::placeholders::_1, std::placeholders::_2));
    MainWindow::robot_group.at(robot_index)->setCameraParameters(this->camera_address, std::bind(&MainWindow::process_this_camera, this, std::placeholders::_1));
    MainWindow::robot_group.at(robot_index)->setMyRobotGroupIndex(robot_index);
    MainWindow::robot_group.at(robot_index)->set_accept_commands(false);

    MainWindow::set_index_of_current_robot(robot_index);
    MainWindow::robot_group.at(this->index_of_current_robot)->robotStart();
}

void MainWindow::on_pushButton_switch_robot_clicked() {

    // Bude to take tlacidlo, ktore pojde iba jednym smerom
    if (this->index_of_current_robot < this->robot_group.size() - 1) {
        this->index_of_current_robot += 1;
    } else {
        this->index_of_current_robot = 0;
    }

    // std::cout << "index of current robot: " << this->indexOfCurrentRobot << std::endl;
}

void MainWindow::on_pushButton_add_robot_clicked() {        

    if (!ui->lineEdit->text().isEmpty()) {

        IpReturnMessage msg = MainWindow::check_ip_address(ui->lineEdit->text().toStdString());

        if (msg.b) {
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
    /*
    this->laserParametersLaserPortIn += 10;
    this->laserParametersLaserPortOut += 10;
    this->robotParametersLaserPortIn += 10;
    this->robotParametersLaserPortOut += 10;
    */

    MainWindow::add_new_robot_to_group(this->index_of_current_robot, this->robot_group.size() + 1);

    // Vypne lineEdit po pridani troch robotov
    if (this->used_robot_ips.size() == 3) {
        std::cout << "Sorry, you cannot add another robot. There already are " << this->used_robot_ips.size() << " robots being controlled." << std::endl;
        this->ui->lineEdit->setEnabled(false);
    }
}

IpReturnMessage MainWindow::check_ip_address(std::string ip) {

    IpReturnMessage return_data;

    // Nie je mozne pridat uz pouzivanu IPcku
    for (int i = 0; i < this->robot_group.size(); i++) {
        if (this->robot_group[i]->getIpAddress().compare(ip) == 0) {
            return_data.b = false;
            return_data.message = "You cannot connect the same robot twice!";
            return return_data;
        }
    }

    // Kontrola ci je IPcka v spravnom formate
    int number_of_dots = 0;
    for (int i = 0; i < ip.size(); i++) {

        if (ip[i] == ('.')) {
            number_of_dots++;
        }
    }

    return_data.message = "Ip address is not in correct format!";

    if (number_of_dots == 3) {
        return_data.b = true;
        return return_data;

    } else {
        return_data.b = false;
        return return_data;
    }
}

void MainWindow::on_pushButton_9_clicked() { // start button

    // Check if user entered an ip address
    if (!ui->lineEdit->text().isEmpty()) {

        IpReturnMessage msg = MainWindow::check_ip_address(ui->lineEdit->text().toStdString());

        if (msg.b) {
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
    connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double))); // pripaja signal k slotu
    connect(this, SIGNAL(startButtonPressed(bool)), this, SLOT(setButtonStates()));

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

    emit startButtonPressed(true); // vyslanie signalu
}

void MainWindow::on_pushButton_2_clicked() { //forward

    robot_group.at(this->index_of_current_robot)->setTranslationSpeed(500);
}

void MainWindow::on_pushButton_3_clicked() { // back

    robot_group.at(this->index_of_current_robot)->setTranslationSpeed(-250);    
}

void MainWindow::on_pushButton_6_clicked() { // left

    robot_group.at(this->index_of_current_robot)->setRotationSpeed(3.14159/2);    
}

void MainWindow::on_pushButton_5_clicked() { // right

    robot_group.at(this->index_of_current_robot)->setRotationSpeed(-3.14159/2); 
}

void MainWindow::on_pushButton_4_clicked() { // stop

    robot_group.at(this->index_of_current_robot)->setTranslationSpeed(0);
}

void MainWindow::on_pushButton_clicked() {

    if (this->use_camera1 == true) {

        this->use_camera1 = false;
        ui->pushButton->setText("Use camera");

    } else {

        this->use_camera1 = true;
        ui->pushButton->setText("Use laser");
    }
}

void MainWindow::get_new_frame() {

}
