#ifndef UARTCONTROLLER_H
#define UARTCONTROLLER_H

#include <QObject>
#include <QVector>
#include <QSerialPort>
#include <QSerialPortInfo>

class UARTController : public QObject
{
    Q_OBJECT
public:
    explicit UARTController(QObject *parent = 0);
    ~UARTController();
    void checkAvailablePorts();
    void openPort();
    void moveMotor(quint16 command_arg);
    void writeByte(char byte_arg);
private:
    QSerialPort *port;
    QByteArray array;

signals:
    void dataUpdated(void);
public slots:
    void readData();
public:
    QVector<int> data;
};

#endif // UARTCONTROLLER_H
