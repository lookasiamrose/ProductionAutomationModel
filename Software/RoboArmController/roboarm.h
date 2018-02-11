#ifndef ROBOARM_H
#define ROBOARM_H

#include <QObject>
#include <QGraphicsScene>
#include <QAbstractGraphicsShapeItem>
#include <QGridLayout>
#include "led_indicator/ledindicator.h"

class RoboArm : public QObject
{
    Q_OBJECT
    Q_ENUMS(Options)
public:
    enum Options{Grip=1,ElbowMove=2,ElbowTurn=3,Knee=4,Pelvis=5,Turnaround=6};
    explicit RoboArm(QObject *parent = 0);
    QWidget* setSceneReturnLEDs(QGraphicsScene* scene_arg);
    void resizeModel();
    quint16 keyPressed(int);

    void incrOption(Options &opt);
    void decrOption(Options &opt);
    void processOption();
    void clearOptions();
    void setOffLeds();
private:
    QVector<LedIndicator*> leds;
    QGraphicsScene* scene;
    Options cpartNumber;
    QWidget* ledPlaceholder;
    quint16 ledIndex;

    QVector<QPen*> pens;
    QVector<QAbstractGraphicsShapeItem*> parts;
    QVector<QVector<qint32> > pos;
signals:

public slots:
};

#endif // ROBOARM_H
