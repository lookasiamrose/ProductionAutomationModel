#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStackedLayout>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pressedKey = false;
    connect(this,SIGNAL(staticResizeSignal(void)),this,SLOT(staticResize()));

    scene = new QGraphicsScene(this);
    model = new RoboArm(this);
    controller = new UARTController(this);
    connect(controller,SIGNAL(dataUpdated(void)),this,SLOT(updateLCDs()));

    view = new QGraphicsView(this);
    view->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    view->setScene(scene);
    view->setStyleSheet("background: transparent");

    QStackedLayout *stackedLayout = new QStackedLayout(ui->viewGrid);
    stackedLayout->setStackingMode(QStackedLayout::StackAll);
    leds = model->setSceneReturnLEDs(scene);
    stackedLayout->addWidget(leds);
    stackedLayout->addWidget(view);

    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->ensureVisible(scene->sceneRect());

    this->setFocus();
    emit staticResizeSignal();
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::mousePressEvent(QMouseEvent *)
{
    //QPoint mousePos = this->mapFromGlobal(QCursor::pos());
    //qDebug()<<(-(view->width()/2-mousePos.x()+151+8))<<" , "<<(-(view->height()/2-mousePos.y()+12+40)+30);
    this->setFocus();
}
void MainWindow::keyPressEvent(QKeyEvent* e)
{
    if( !(e->isAutoRepeat()) && !pressedKey)
    {
        pressedKey = true;
        if(model->keyPressed(e->key()) != 12)
        {
            quint16 direction_motor = model->keyPressed(e->key());
            controller->moveMotor(direction_motor);
        }
    }
}
void MainWindow::keyReleaseEvent(QKeyEvent* e)
{
    if( !(e->isAutoRepeat()) && pressedKey)
    {
        pressedKey = false;
        model->setOffLeds();
        switch(e->key())
        {
        case 16777236: //right
            controller->writeByte(';');
            break;
        case 16777234: //left
            controller->writeByte(';');
            break;
        default:
            break;
        }
    }
}
void MainWindow::staticResize()
{
    QSize temporary = this->size();
    temporary.setHeight(temporary.height()+150);
    temporary.setWidth(temporary.width()+50);
    this->resize(temporary);
    view->fitInView( view->rect(), Qt::KeepAspectRatio );
    model->resizeModel();
}
void MainWindow::resizeEvent(QResizeEvent *)
{
    model->resizeModel();
    view->fitInView( view->rect(), Qt::KeepAspectRatio );
}

void MainWindow::on_testLeftButton_clicked()
{
    controller->data[0] = 0;
    ui->lcdNumber_6->display(0);
    this->setFocus();
}

void MainWindow::on_testRightButton_clicked()
{
    controller->data[1] = 0;
    ui->lcdNumber_5->display(0);
    this->setFocus();
}

void MainWindow::on_pushButton_clicked()
{
    controller->data[2] = 0;
    ui->lcdNumber_4->display(0);
    this->setFocus();
}

void MainWindow::on_pushButton_2_clicked()
{
    controller->data[3] = 0;
    ui->lcdNumber_3->display(0);
    this->setFocus();
}

void MainWindow::on_pushButton_3_clicked()
{
    controller->data[4] = 0;
    ui->lcdNumber_2->display(0);
    this->setFocus();
}

void MainWindow::on_pushButton_4_clicked()
{
    controller->data[5] = 0;
    ui->lcdNumber->display(0);
    this->setFocus();
}
void MainWindow::updateLCDs()
{
    ui->lcdNumber->display(controller->data[5]);
    ui->lcdNumber_2->display(controller->data[4]);
    ui->lcdNumber_3->display(controller->data[3]);
    ui->lcdNumber_4->display(controller->data[2]);
    ui->lcdNumber_5->display(controller->data[1]);
    ui->lcdNumber_6->display(controller->data[0]);
}

void MainWindow::on_pushButton_5_clicked()
{
    foreach (int element, controller->data)
    {
        element = 0;
    }
    ui->lcdNumber->display(0);
    ui->lcdNumber_2->display(0);
    ui->lcdNumber_3->display(0);
    ui->lcdNumber_4->display(0);
    ui->lcdNumber_5->display(0);
    ui->lcdNumber_6->display(0);
}

void MainWindow::on_pushButton_6_clicked()
{
    char start = (char)153;
    char stop = (char)170;
    controller->writeByte(start);
    controller->writeByte('c');
    controller->writeByte(stop);
    controller->writeByte(((start ^ stop)));
    //qDebug()<<quint8(start);
    //qDebug()<<quint8('c');
    //qDebug()<<quint8(stop);
    //qDebug()<<quint8(start | stop);
}
