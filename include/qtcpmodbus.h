#pragma once
#include <QtCore/QObject>
#include <QAbstractModbus>
#include <QtNetwork/QTcpSocket>
#include <QtCore/QString>
#include <QtCore/QList>
#include <time.h>

/*!
* The QiTcpModbus class talks to remote attached (IP network) modbus/TCP slave or Tcp-Modbus gateway devices using the
* modbus TCP protocol.
*
* \headerfile qitcpmodbus.h QiTcpModbus
*/
class QTcpModbus: public QObject, public QAbstractModbus {
    Q_OBJECT

  private:
    mutable QTcpSocket _socket;     // Socket used for communication.
    int _timeout;                   // Timeout to use in TCP communication.
    int _connectTimeout;            // TCP connect timeout.

  public:
    QTcpModbus();

    ~QTcpModbus();

    bool isOpen() const {
        return isConnected();
    }

    /*!
    * Connects to the given hostname (DNS lookups are done automatically) or ip encoded as string (Ex. 127.0.0.1) and
    * the given port (Modbus default port is 502).
    *
    * \param host IP address or DNS name of the host to connect to.
    * \param port Port to be used for TCP connection. Modbus default is 502.
    * \return True if the connection could be established, false otherwise.
    */
    bool connect(const QString& host, quint16 port = 502);

    /*!
    * Returns true of the connection to the Modbus device or gateway is alive.
    *
    * \return True if connection is active.
    */
    bool isConnected() const;

    /*!
    * Closes the connection to the device or gateway.
    */
    void disconnect();

    /*!
    * Returns the active timeout when connecting. This means if the Modbus/TCP device or gateway does not accepts the
    * connection in this time, the connect() method will fail. Default is 2 seconds.
    *
    * \return Connection timeout.
    */
    int connectTimeout() const;

    /*!
    * Changes the active timeout when connecting.
    *
    * \param timeout New connection timeout in milliseconds.
    */
    void setConnectTimeout(int timeout);

    unsigned int timeout() const;
    void setTimeout(unsigned int timeout);
    QList<bool> readCoils(quint8 deviceAddress, quint16 startingAddress, quint16 quantityOfCoils,
                          quint8* const status = NULL) const;
    QList<bool> readDiscreteInputs(quint8 deviceAddress, quint16 startingAddress, quint16 quantityOfInputs,
                                   quint8* const status = NULL) const;
    QList<quint16> readHoldingRegisters(quint8 deviceAddress, quint16 startingAddress, quint16 quantityOfRegisters,
                                        quint8* const status = NULL) const;
    QList<quint16> readInputRegisters(quint8 deviceAddress, quint16 startingAddress, quint16 quantityOfInputRegisters,
                                      quint8* const status = NULL) const;
    bool writeSingleCoil(quint8 deviceAddress, quint16 outputAddress, bool outputValue,
                         quint8* const status = NULL) const;
    bool writeSingleRegister(quint8 deviceAddress, quint16 registerAddress, quint16 registerValue,
                             quint8* const status = NULL) const;
    bool writeMultipleCoils(quint8 deviceAddress, quint16 startingAddress, const QList<bool>& outputValues,
                            quint8* const status = NULL) const;
    bool writeMultipleRegisters(quint8 deviceAddress, quint16 startingAddress,const QList<quint16>& registersValues,
                                quint8* const status = NULL) const;
    bool maskWriteRegister(quint8 deviceAddress, quint16 referenceAddress, quint16 andMask, quint16 orMask,
                           quint8* const status = NULL) const;
    QList<quint16> writeReadMultipleRegisters(quint8 deviceAddress, quint16 writeStartingAddress,
                                              const QList<quint16>& writeValues, quint16 readStartingAddress,
                                              quint16 quantityToRead, quint8* const status = NULL) const;
    QList<quint16> readFifoQueue(quint8 deviceAddress, quint16 fifoPointerAddress, quint8* const status = NULL) const;
    QByteArray executeCustomFunction(quint8 deviceAddress, quint8 modbusFunction, const QByteArray& data,
                                     quint8* const status = NULL) const;
    QByteArray executeRaw(const QByteArray& data, quint8* const status = NULL) const;
    QByteArray calculateCheckSum(const QByteArray& data) const;

signals:
    /*!
    * This signal is emitted if the connection to the modbus device was lost.
    */
    void connectionLost();
};
