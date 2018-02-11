#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QKeyEvent>
#include "roboarm.h"
#include "uartcontroller.h"
namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    void resizeEvent(QResizeEvent *);
    void mousePressEvent(QMouseEvent *);
    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    RoboArm* model;
    UARTController* controller;
    QGraphicsView* view;
    QGraphicsScene* scene;
    QWidget* leds;
    bool pressedKey;
public slots:
    void staticResize();
signals:
    void staticResizeSignal(void);
private slots:
    void on_testLeftButton_clicked();
    void on_testRightButton_clicked();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void updateLCDs();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
};

#endif // MAINWINDOW_H
