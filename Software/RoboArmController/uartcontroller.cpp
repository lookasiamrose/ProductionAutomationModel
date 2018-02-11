#include "uartcontroller.h"
#include <QDebug>
#define MOTORS_NUM 6

UARTController::UARTController(QObject *parent) : QObject(parent)
{
    //checkAvailablePorts();

    openPort();

    for(quint16 i=0;i<MOTORS_NUM;i++)
    {
        data.append(0);
    }
}
UARTController::~UARTController()
{
    if(port->isOpen()) port->close();

    delete port;
}
void UARTController::moveMotor(quint16 command_arg)
{
    char command;
    char direction;
    if(command_arg%2==0) direction = '>';
    else
    {
        command_arg--;
        direction = '<';
    }
    switch (command_arg)
    {
    case 0:
        command = 'a';
        if(direction == '<') direction = '>';
            else direction = '<';
        break;
    case 2:
        command = 'c';
        break;
    case 4:
        command = 'b';
        break;
    case 6:
        command = 'd';
        break;
    case 8:
        command = 'e';
        break;
    case 10:
        command = 'f';
        if(direction == '<') direction = '>';
            else direction = '<';
        break;
    default:
        break;
    }
    this->writeByte(command);
    this->writeByte(direction);
    this->writeByte('.');
}
void UARTController::writeByte(char byte_arg)
{
    if(port->isOpen())
    {
        QByteArray tosend;
        tosend.append(byte_arg);
        port->write(tosend,1);
        port->waitForBytesWritten(-1);
        tosend.clear();
    }
}
void UARTController::readData()
{
    array += port->readAll();
    //while(port->waitForReadyRead(100)) array += port->readAll();
    if(array.size() == 14)
    {
        if( (array.at(0) == '!') && (array.at(13) == '.') )
        {
            QByteArray temp;
            QByteArray rary;
            quint16 di = 0;
            for(quint16 i=2;i<13;i+=2)
            {
                temp = temp.append(array[i-1]);
                unsigned short msb = temp.toUShort();

                rary.append(array[i]);
                unsigned short lsb = rary.toUShort();

                int combined = (msb << 8 ) | (lsb & 0xff);
                if(combined == 0) data[di] += combined;
                else
                    data[di] = combined;
                di++;
                //qDebug()<<temp<<" "<<rary;
                temp.clear();
                rary.clear();
            }
            emit dataUpdated();
        }
        array.clear();
    }
}
void UARTController::openPort()
{
    bool isopen = false;
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        if(serialPortInfo.portName() == "COM6")
        {
            port = new QSerialPort(this);
            port->setPort(serialPortInfo);
            port->setBaudRate(1200);
            //port->setParity(QSerialPort::EvenParity);
            port->open(QIODevice::ReadWrite);

            connect(port,SIGNAL(readyRead()),this,SLOT(readData()));

            isopen = true;
            break;
        }
    }
    if(!isopen)
    {
        qDebug()<<"Cannot open serial port!";
    }
}
void UARTController::checkAvailablePorts()
{
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
       qDebug() << "\nPort:" << serialPortInfo.portName();
       qDebug() << "Location:" << serialPortInfo.systemLocation();
       qDebug() << "Description:" << serialPortInfo.description();
       qDebug() << "Manufacturer:" << serialPortInfo.manufacturer();
       qDebug() << "Vendor Identifier:" << (serialPortInfo.hasVendorIdentifier() ? QByteArray::number(serialPortInfo.vendorIdentifier(), 16) : QByteArray());
       qDebug() << "Product Identifier:" << (serialPortInfo.hasProductIdentifier() ? QByteArray::number(serialPortInfo.productIdentifier(), 16) : QByteArray());
       qDebug() << "Busy:" << (serialPortInfo.isBusy() ? QObject::tr("Yes") : QObject::tr("No"));

       QSerialPort *port = new QSerialPort(serialPortInfo);
       if (port->open(QIODevice::ReadWrite))
       {
           qDebug() << "Baud rate:" << port->baudRate();
           qDebug() << "Data bits:" << port->dataBits();
           qDebug() << "Stop bits:" << port->stopBits();
           qDebug() << "Parity:" << port->parity();
           qDebug() << "Flow control:" << port->flowControl();
           qDebug() << "Read buffer size:" << port->readBufferSize();
           port->close();
       } else
           {
               qDebug() << "Unable to open port, error code" << port->error();
           }
       delete port;
    }
}
