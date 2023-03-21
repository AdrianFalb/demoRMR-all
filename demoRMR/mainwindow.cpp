#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui(new Ui::MainWindow) {   

    // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    this->http_string = "http://";
    this->file_string = "/stream.mjpg";
    this->port_string = ":8000";
    //this->port_string = ":8889"; // simulator

    this->ip_address = "127.0.0.1"; // Local host - default
    this->camera_address = this->http_string + this->ip_address + this->port_string + this->file_string; // Local host - Default

    this->indexOfCurrentRobot = 0;

    this->laserParametersLaserPortOut = 52999;
    this->laserParametersLaserPortIn = 5299;

    this->robotParametersLaserPortOut = 53000;
    this->robotParametersLaserPortIn = 5300;

    ui->setupUi(this);
    this->datacounter = 0;    
    this->actIndex = -1;
    this->useCamera1 = false;

    this->datacounter = 0;

    ui->pushButton_add_robot->setEnabled(false);
    ui->pushButton_switch_robot->setEnabled(false);

    //this->start_message_thread();

    auto message = std::bind(&MainWindow::process_this_message, this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3,
                             std::placeholders::_4, std::placeholders::_5,
                             std::placeholders::_6);

    th1 = std::move(std::thread(message, ske_si_me1, ske_si_other1, ske_si_posli1, ske_s1, ske_recv_len1, 23431));
    th2 = std::move(std::thread(message, ske_si_me2, ske_si_other2, ske_si_posli2, ske_s2, ske_recv_len2, 23432));
    th3 = std::move(std::thread(message, ske_si_me3, ske_si_other3, ske_si_posli3, ske_s3, ske_recv_len3, 23433));
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

void MainWindow::setButtonStates() {

    ui->pushButton_9->setEnabled(false);
    ui->pushButton_add_robot->setEnabled(true);
    ui->pushButton_switch_robot->setEnabled(true);    
}

void MainWindow::set_ip_address(std::string ipAddress) {

    this->ip_address = ipAddress;
    this->camera_address = this->http_string + this->ip_address + this->port_string + this->file_string;
    ui->lineEdit->clear();
}

int MainWindow::process_this_robot(TKobukiData robotdata) {

    // Joystick

    /*
    if (this->forwardspeed == 0 && this->rotationspeed != 0) {

        robotGroup.at(0)->setRotationSpeed(this->rotationspeed);

    } else if (this->forwardspeed != 0 && this->rotationspeed == 0) {

        robotGroup.at(0)->setTranslationSpeed(this->forwardspeed);

    } else if ((this->forwardspeed != 0 && this->rotationspeed != 0)) {

        robotGroup.at(0)->setArcSpeed(this->forwardspeed, this->forwardspeed/this->rotationspeed);

    } else {

        robotGroup.at(0)->setTranslationSpeed(0);
    }
    */

    if (this->datacounter % 5) {

        emit uiValuesChanged(this->robotdata.EncoderLeft, 11, 12);
    }

    this->datacounter++;
    return 0;
}

int MainWindow::process_this_lidar(LaserMeasurement laserData) {

    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    // tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    this->updateLaserPicture=1;
    update(); //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

int MainWindow::process_this_camera(cv::Mat cameraData) {

    cameraData.copyTo(frame[(actIndex + 1) % 3]);
    actIndex = (actIndex + 1) % 3;
    this->updateLaserPicture = 1;
    return 0;
}

/*
void MainWindow::start_message_thread() {

    std::function<void(void)> f = std::bind(&MainWindow::process_this_message, this);
    this->robot_message_thread = std::move(std::thread(f));

}*/

void MainWindow::process_this_message(sockaddr_in ske_si_me, sockaddr_in ske_si_other, sockaddr_in ske_si_posli, int ske_s, int ske_recv_len, int port) {

#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); // Initialize Winsock
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
    ske_si_me.sin_port = htons(port); //binds to port 23433
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(port);
    ske_si_posli.sin_addr.s_addr = inet_addr(ip_address.data()); //htonl(INADDR_BROADCAST);
    std::cout << ::bind(ske_s, (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) ) << std::endl;

    char buf[500];
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

/*
void MainWindow::process_this_message() {

#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); // Initialize Winsock
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
    setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene));
#endif
    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(23433); // binds to port 23433
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(23433);
    ske_si_posli.sin_addr.s_addr = inet_addr(ip_address.data()); //htonl(INADDR_BROADCAST);
    std::cout << ::bind(ske_s, (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) ) << std::endl;

    char buf[500];
    while(true) {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&buf, sizeof(buf), 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1) {

            // std::cout<<"problem s prijatim"<<std::endl;
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


        // memcpy(message, buf, 500);
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
*/

void MainWindow::issue_robot_command(std::string robot_ip_address, std::string robot_command) {

    bool command_allowed = true;

    for(unsigned short int i = 0; i < robotGroup.size(); i++) {

       if (robot_ip_address == robotGroup[i]->getIpAddress()) {
           this->indexOfCurrentRobot = robotGroup[i]->getMyRobotGroupIndex();
           command_allowed = true;

           if (robot_command == "WAKE_UP") {
               robotGroup.at(this->indexOfCurrentRobot)->set_accept_commands(true);
           }
           break;

       } else {
           command_allowed = false;
           robotGroup.at(i)->set_accept_commands(false);
       }
    }

    if (command_allowed == false) {
        std::cout << "No such robot id was found!" << endl;
        return;
    }

    if (robot_command == "STOP" && robotGroup.at(this->indexOfCurrentRobot)->get_accept_commands() == true) {
        robotGroup.at(this->indexOfCurrentRobot)->setTranslationSpeed(0);

    } else if (robot_command == "FORWARD" && robotGroup.at(this->indexOfCurrentRobot)->get_accept_commands() == true) {
        robotGroup.at(this->indexOfCurrentRobot)->setTranslationSpeed(250);

    } else if (robot_command == "BACKWARD" && robotGroup.at(this->indexOfCurrentRobot)->get_accept_commands() == true) {
        robotGroup.at(this->indexOfCurrentRobot)->setTranslationSpeed(-250);

    } else if (robot_command == "RIGHT" && robotGroup.at(this->indexOfCurrentRobot)->get_accept_commands() == true) {
        robotGroup.at(this->indexOfCurrentRobot)->setRotationSpeed((-3.14159/2));

    } else if (robot_command == "LEFT" && robotGroup.at(this->indexOfCurrentRobot)->get_accept_commands() == true) {
        robotGroup.at(this->indexOfCurrentRobot)->setRotationSpeed((3.14159/2));
    }
}

void MainWindow::set_index_of_current_robot(unsigned short int robotIndex) {

    this->indexOfCurrentRobot = robotIndex;

}

void MainWindow::add_new_robot_to_group(unsigned short int robotIndex, unsigned short int numberOfRobots) {

    //MainWindow::robotGroup.resize(numberOfRobots, new Robot());
    MainWindow::robotGroup.insert(std::map<unsigned short int, Robot*>::value_type(robotIndex, new Robot()));
    MainWindow::robotGroup.at(robotIndex)->setLaserParameters(this->ip_address, this->laserParametersLaserPortOut, this->laserParametersLaserPortIn, /*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::process_this_lidar, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setRobotParameters(this->ip_address, this->robotParametersLaserPortOut, this->robotParametersLaserPortIn, std::bind(&MainWindow::process_this_robot, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setCameraParameters(this->camera_address, std::bind(&MainWindow::process_this_camera, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setMyRobotGroupIndex(robotIndex);
    MainWindow::robotGroup.at(robotIndex)->set_accept_commands(false);

    MainWindow::set_index_of_current_robot(robotIndex);

    MainWindow::robotGroup.at(this->indexOfCurrentRobot)->robotStart();
}

void MainWindow::on_pushButton_switch_robot_clicked() {

    // Bude to take tlacidlo, ktore pojde iba jednym smerom
    if (this->indexOfCurrentRobot < this->robotGroup.size() - 1) {
        this->indexOfCurrentRobot += 1;
    } else {
        this->indexOfCurrentRobot = 0;
    }

    //std::cout << "index of current robot: " << this->indexOfCurrentRobot << std::endl;
}

void MainWindow::on_pushButton_add_robot_clicked() {

    while (this->indexOfCurrentRobot < this->robotGroup.size() - 1) {

        this->indexOfCurrentRobot += 1; // tuto iba zabezpecujem, aby bol ten index na max pred pridanim dalsieho robota
    }

    this->indexOfCurrentRobot += 1;

    if (!ui->lineEdit->text().isEmpty()) {

        this->set_ip_address(ui->lineEdit->text().toStdString());
    }

    // Porty sa menia iba v simulatore, kazdy robot ma rovnake porty
    /*
    this->laserParametersLaserPortIn += 10;
    this->laserParametersLaserPortOut += 10;
    this->robotParametersLaserPortIn += 10;
    this->robotParametersLaserPortOut += 10;
    */


    MainWindow::add_new_robot_to_group(this->indexOfCurrentRobot, this->robotGroup.size() + 1);
}

void MainWindow::on_pushButton_9_clicked() { // start button

    this->forward_speed = 0;
    this->rotation_speed = 0;

    // tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    /*
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
    */
    connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double))); // pripaja signal k slotu
    connect(this, SIGNAL(startButtonPressed(bool)), this, SLOT(setButtonStates()));

    // Check if user entered an ip address
    if (!ui->lineEdit->text().isEmpty()) {

        this->set_ip_address(ui->lineEdit->text().toStdString());
    }

    MainWindow::add_new_robot_to_group(this->indexOfCurrentRobot, 1);

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

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robotGroup.at(this->indexOfCurrentRobot)->setTranslationSpeed(500);

    /*std::vector<unsigned char> mess=robot.setTranslationSpeed(500);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_3_clicked() { // back

    robotGroup.at(this->indexOfCurrentRobot)->setTranslationSpeed(-250);

    /*  std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_6_clicked() { // left

    robotGroup.at(this->indexOfCurrentRobot)->setRotationSpeed(3.14159/2);

    /*  std::vector<unsigned char> mess=robot.setRotationSpeed(3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_5_clicked() { // right

    robotGroup.at(this->indexOfCurrentRobot)->setRotationSpeed(-3.14159/2);

    /* std::vector<unsigned char> mess=robot.setRotationSpeed(-3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_4_clicked() { // stop

    robotGroup.at(this->indexOfCurrentRobot)->setTranslationSpeed(0);

    /*  std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_clicked() {

    if (this->useCamera1 == true) {

        this->useCamera1 = false;
        ui->pushButton->setText("Use camera");

    } else {

        this->useCamera1 = true;
        ui->pushButton->setText("Use laser");
    }
}

void MainWindow::get_new_frame() {

}
