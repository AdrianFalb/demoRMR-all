#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui(new Ui::MainWindow) {

    // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna

    this->httpString = "http://";
    this->fileString = "/stream.mjpg";
    this->portString = ":8000";

    this->ipAddress = "127.0.0.1"; // Local host - default
    this->cameraAddress = "http://127.0.0.1:8889/stream.mjpg"; // Local host - Default

    this->indexOfCurrentRobot = 0;

    this->laserParametersLaserPortOut = 52999;
    this->laserParametersLaserPortIn = 5299;

    this->robotParametersLaserPortOut = 53000;
    this->robotParametersLaserPortIn = 5300;

    // cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    this->datacounter = 0;
    // timer = new QTimer(this);
    // connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    this->actIndex = -1;
    this->useCamera1 = false;

    this->datacounter = 0;
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

void MainWindow::setIpAddress(std::string ipAddress) {

    this->ipAddress = ipAddress;
    this->cameraAddress = this->httpString + this->ipAddress + this->portString + this->fileString;
}

int MainWindow::processThisRobot(TKobukiData robotdata) {

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

int MainWindow::processThisLidar(LaserMeasurement laserData) {

    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    // tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    this->updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}

int MainWindow::processThisCamera(cv::Mat cameraData) {

    cameraData.copyTo(frame[(actIndex + 1) % 3]);
    actIndex = (actIndex + 1) % 3;
    this->updateLaserPicture = 1;
    return 0;
}

void MainWindow::setIndexOfCurrentRobot(unsigned short int robotIndex) {

    this->indexOfCurrentRobot = robotIndex;

}

void MainWindow::addNewRobotToGroup(unsigned short int robotIndex, unsigned short int numberOfRobots) {

    MainWindow::robotGroup.resize(numberOfRobots, new Robot());
    MainWindow::robotGroup.at(robotIndex)->setLaserParameters(this->ipAddress, this->laserParametersLaserPortOut, this->laserParametersLaserPortIn, /*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setRobotParameters(this->ipAddress, this->robotParametersLaserPortOut, this->robotParametersLaserPortIn, std::bind(&MainWindow::processThisRobot, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setCameraParameters(this->cameraAddress, std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
    MainWindow::robotGroup.at(robotIndex)->setMyRobotGroupIndex(robotIndex);

    MainWindow::setIndexOfCurrentRobot(robotIndex);

    MainWindow::robotGroup.at(robotIndex)->robotStart();
}

void MainWindow::on_pushButton_9_clicked() { // start button

    this->forwardspeed = 0;
    this->rotationspeed = 0;

    // tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    /*
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);
    */
    connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double)));

    // Check if user entered an ip address
    if (!ui->lineEdit->text().isEmpty()) {

        this->setIpAddress(ui->lineEdit->text().toStdString());
    }

    MainWindow::addNewRobotToGroup(this->indexOfCurrentRobot, 1);

    instance = QJoysticks::getInstance();

    /* Enable the virtual joystick */
    /*  instance->setVirtualJoystickRange(1);
    instance->setVirtualJoystickEnabled(true);
    instance->setVirtualJoystickAxisSensibility(0.7);*/
    //instance->
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
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

void MainWindow::getNewFrame() {

}
