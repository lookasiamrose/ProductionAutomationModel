#include "roboarm.h"
#include <QGraphicsEllipseItem>
#include <QDebug>
#define NUM_OF_LEDS 12

RoboArm::RoboArm(QObject *parent) : QObject(parent)
{
    for(quint16 i=0;i<NUM_OF_LEDS;i++)
    {
        QVector<qint32> temp;
        temp.append(0);
        temp.append(0);
        pos.append(temp);
    }
    cpartNumber = Options::Grip;
    ledIndex = 0;

    pos[1][0] = -16;
    pos[1][1] = -190;
    pos[0][0] = -53;
    pos[0][1] = -178;

    pos[2][0] = -51;
    pos[2][1] = -120;
    pos[3][0] = 20;
    pos[3][1] = -120;

    pos[4][0] = -68;
    pos[4][1] = -86;
    pos[5][0] = 39;
    pos[5][1] = -86;

    pos[6][0] = -65;
    pos[6][1] = -35;
    pos[7][0] = 17;
    pos[7][1] = -35;

    pos[8][0] = -65;
    pos[8][1] = 80;
    pos[9][0] = 18;
    pos[9][1] = 80;

    pos[10][0] = -68;
    pos[10][1] = 149;
    pos[11][0] = 60;
    pos[11][1] = 149;

    for(quint16 i=0;i<NUM_OF_LEDS;i++)
    {
        LedIndicator* temp = new LedIndicator();
        leds.append(temp);
    }
    foreach(LedIndicator* element, leds)
    {
        element->setOnPattern(Qt::Dense2Pattern);
        element->setOffPattern(Qt::Dense2Pattern);
        element->setOnColor(Qt::yellow);
        element->setOffColor(Qt::darkRed);
        element->setLedSize(16);
    }
    pens.append(new QPen(Qt::black));
    pens[0]->setWidth(2);
    pens.append(new QPen(Qt::darkYellow));
}
QWidget* RoboArm::setSceneReturnLEDs(QGraphicsScene* scene_arg)
{
    scene = scene_arg;

    ledPlaceholder = new QWidget();

    foreach (LedIndicator* element, leds)
    {
        element->setParent(ledPlaceholder);
    }


    parts.append( scene->addEllipse(0,0,80,15,*pens[0]) );
    parts.append( scene->addRect(20,15,15,100,*pens[0]) );
    parts.append( scene->addRect(55,10,15,100,*pens[0]) );

    parts.append( scene->addRect(18,-55,12,50,*pens[0]) );
    parts.append( scene->addRect(47,-55,12,50,*pens[0]) );

    parts.append( scene->addRect(-57,126,15,100,*pens[0]) );
    parts.append( scene->addRect(-23,137,15,100,*pens[0]) );

    parts.append( scene->addEllipse(0,237,100,20,*pens[0]) );
    QPolygonF polygon;
    polygon<<QPoint(18,-60)<<QPoint(30,-90)<<QPoint(30,-60)<<QPoint(24,-60)<<QPoint(24,-40)<<QPoint(24,-60);
    parts.append( scene->addPolygon(polygon,*pens[0]) );
    QPolygonF polygon2;
    polygon2<<QPoint(18,-60)<<QPoint(18,-90)<<QPoint(30,-60)<<QPoint(24,-60)<<QPoint(24,-40)<<QPoint(24,-60);
    parts.append( scene->addPolygon(polygon2,*pens[0]) );
    parts[8]->moveBy(35,-7);
    parts[9]->moveBy(0,-31);

    parts[1]->setRotation(15);
    parts[2]->setRotation(15);
    parts[5]->setRotation(-20);
    parts[6]->setRotation(-20);
    parts[8]->setRotation(-30);
    parts[9]->setRotation(30);

    QPolygonF polygon3;
    polygon3<<QPoint(-80,270)<<QPoint(170,270);
    parts.append( scene->addPolygon(polygon3,*pens[0]) );

    for(quint16 i=0;i<NUM_OF_LEDS;i++)
    {
        leds[i]->move(ledPlaceholder->width()/2+pos[i][0],ledPlaceholder->height()/2+pos[i][1]);
    }

    parts[8]->setPen(*pens[1]);
    parts[9]->setPen(*pens[1]);

    foreach( QAbstractGraphicsShapeItem* item, parts)
    {
        item->setFlag(QGraphicsItem::ItemIsSelectable);
    }

    //parts[9]->setSelected(true);
    //parts[8]->setSelected(true);

    return ledPlaceholder;
}
void RoboArm::resizeModel()
{
    for(quint16 i=0;i<NUM_OF_LEDS;i++)
    {
        leds[i]->move(ledPlaceholder->width()/2+pos[i][0],ledPlaceholder->height()/2+pos[i][1]);
    }
}
quint16 RoboArm::keyPressed(int key)
{
    switch (key) {
    case 16777236: //right
        leds[ledIndex+1]->setState(true);
        return (ledIndex+1);
        break;
    case 16777234: //left
        leds[ledIndex]->setState(true);
        return ledIndex;
        break;
    case 16777237: //down
        incrOption(cpartNumber);
        processOption();
        break;
    case 16777235: //up
        decrOption(cpartNumber);
        processOption();
        break;
    default:
        qDebug()<<key;
        break;
    }
    return 12;
}
void RoboArm::setOffLeds()
{
    foreach (LedIndicator* element, leds)
    {
        element->setState(false);
    }
}
void RoboArm::processOption()
{
    clearOptions();

    switch (cpartNumber) {
    case Options::Grip:
        ledIndex = 0;
        parts[8]->setPen(*pens[1]);
        parts[9]->setPen(*pens[1]);

        break;
    case Options::ElbowMove:
        ledIndex = 2;
        parts[3]->setPen(*pens[1]);
        parts[4]->setPen(*pens[1]);

        break;
    case Options::ElbowTurn:
        ledIndex = 4;
        parts[0]->setPen(*pens[1]);
        break;
    case Options::Knee:
        ledIndex = 6;
        parts[1]->setPen(*pens[1]);
        parts[2]->setPen(*pens[1]);

        break;
    case Options::Pelvis:
        ledIndex = 8;
        parts[5]->setPen(*pens[1]);
        parts[6]->setPen(*pens[1]);

        break;
    case Options::Turnaround:
        ledIndex = 10;
        parts[7]->setPen(*pens[1]);

        break;
    default:
        break;
    }
}
void RoboArm::clearOptions()
{
    foreach( QAbstractGraphicsShapeItem* item, parts)
    {
        item->setPen(*pens[0]);
    }
}
void RoboArm::incrOption(Options &opt)
{
    switch (opt) {
    case Options::Grip: opt = Options::ElbowMove; break;
    case Options::ElbowMove: opt = Options::ElbowTurn; break;
    case Options::ElbowTurn: opt = Options::Knee; break;
    case Options::Knee: opt = Options::Pelvis; break;
    case Options::Pelvis: opt = Options::Turnaround; break;

    case Options::Turnaround: opt = Options::Turnaround; break;
    default:
        break;
    }
}
void RoboArm::decrOption(Options &opt)
{
    switch (opt) {
    case Options::Grip: opt = Options::Grip; break;

    case Options::ElbowMove: opt = Options::Grip; break;
    case Options::ElbowTurn: opt = Options::ElbowMove; break;
    case Options::Knee: opt = Options::ElbowTurn; break;
    case Options::Pelvis: opt = Options::Knee; break;
    case Options::Turnaround: opt = Options::Pelvis; break;
    default:
        break;
    }
}
