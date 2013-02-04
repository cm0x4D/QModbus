/***********************************************************************************************************************
* QiTcpModbus : Support for TCP Modbus devices and TCP/Modbus gateways.                                                *
************************************************************************************************************************
* Author : Michael Clausen (michael.clausen@hevs.ch)                                                                   *
* Date   : 2009/07/14                                                                                                  *
* Changes: -                                                                                                           *
***********************************************************************************************************************/
#pragma once

/*** Base classes *****************************************************************************************************/
#include <QtCore/QObject>
#include <QAbstractModbus>


/*** Qt includes & prototypes *****************************************************************************************/
#include <QtNetwork/QTcpSocket>
#include <QtCore/QString>
#include <QtCore/QList>


/*** System includes **************************************************************************************************/
#include <time.h>

/*** QiTcpModbus class declaration and help ***************************************************************************/
/*!
* The QiTcpModbus class talks to remote attached (IP network) modbus/TCP slave or Tcp-Modbus gateway devices using the
* modbus TCP protocol.
* \headerfile qitcpmodbus.h QiTcpModbus
*/
class QTcpModbus : public QObject , public QAbstractModbus
{
    Q_OBJECT;

private:
    mutable QTcpSocket _socket;     // Socket used for communication.
    int _timeout;                   // Timeout to use in TCP communication.
    int _connectTimeout;            // TCP connect timeout.

public:
    /*!
    * Constructor.
    */
    QTcpModbus();

    /*!
    * Destructor.
    */
    virtual ~QTcpModbus();

    /*!
    * Connects to the given hostname (DNS lookups are done automatically) or ip encoded as string (Ex. 127.0.0.1) and
    * the given port (Modbus default port is 502).
    * \param host IP address or DNS name of the host to connect to.
    * \param port Port to be used for TCP connection. Modbus default is 502.
    * \return True if the connection could be established, false otherwise.
    */
    bool connect( const QString &host , const quint16 port = 502 );

    /*!
    * Returns true of the connection to the Modbus device or gateway is alive.
    * \return True if connection is active.
    */
    bool isConnected( void ) const;

    /*!
    * Closes the connection to the device or gateway.
    */
    void disconnect( void );

    /*!
    * Returns the active timeout when connecting. This means if the Modbus/TCP device or gateway does not accepts the
    * connection in this time, the connect() method will fail. Default is 2 seconds.
    * \return Connection timeout.
    */
    int connectTimeout( void ) const;

    /*!
    * Changes the active timeout when connecting.
    * \param timeout New connection timeout in milliseconds.
    */
    void setConnectTimeout( const int timeout );

    // Interface implementation (QiAbstractModbus).
    unsigned int timeout( void ) const;

    // Interface implementation (QiAbstractModbus).
    void setTimeout( const unsigned int timeout );

    // Interface implementation (QiAbstractModbus).
    QList<bool> readCoils( const quint8 deviceAddress ,
                           const quint16 startingAddress ,
                           const quint16 quantityOfCoils ,
                           quint8 *const status = NULL
                         ) const;

    // Interface implementation (QiAbstractModbus).
    QList<bool> readDiscreteInputs( const quint8 deviceAddress ,
                                    const quint16 startingAddress ,
                                    const quint16 quantityOfInputs ,
                                    quint8 *const status = NULL
                                  ) const;

    // Interface implementation (QiAbstractModbus).
    QList<quint16> readHoldingRegisters( const quint8 deviceAddress ,
                                         const quint16 startingAddress ,
                                         const quint16 quantityOfRegisters ,
                                         quint8 *const status = NULL
                                       ) const;

    // Interface implementation (QiAbstractModbus).
    QList<quint16> readInputRegisters( const quint8 deviceAddress ,
                                       const quint16 startingAddress ,
                                       const quint16 quantityOfInputRegisters ,
                                       quint8 *const status = NULL
                                     ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeSingleCoil( const quint8 deviceAddress ,
                          const quint16 outputAddress ,
                          const bool outputValue ,
                          quint8 *const status = NULL
                        ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeSingleRegister( const quint8 deviceAddress ,
                              const quint16 registerAddress ,
                              const quint16 registerValue ,
                              quint8 *const status = NULL
                            ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeMultipleCoils( const quint8 deviceAddress ,
                             const quint16 startingAddress ,
                             const QList<bool> & outputValues ,
                             quint8 *const status = NULL
                           ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeMultipleRegisters( const quint8 deviceAddress ,
                                 const quint16 startingAddress ,
                                 const QList<quint16> & registersValues ,
                                 quint8 *const status = NULL
                               ) const;

    // Interface implementation (QiAbstractModbus).
    bool maskWriteRegister( const quint8 deviceAddress ,
                            const quint16 referenceAddress ,
                            const quint16 andMask ,
                            const quint16 orMask ,
                            quint8 *const status = NULL
                          ) const;

    // Interface implementation (QiAbstractModbus).
    QList<quint16> writeReadMultipleRegisters( const quint8 deviceAddress ,
                                               const quint16 writeStartingAddress ,
                                               const QList<quint16> & writeValues ,
                                               const quint16 readStartingAddress ,
                                               const quint16 quantityToRead ,
                                               quint8 *const status = NULL
                                             ) const;


    // Interface implementation (QiAbstractModbus).
    QList<quint16> readFifoQueue( const quint8 deviceAddress ,
                                  const quint16 fifoPointerAddress ,
                                  quint8 *const status = NULL
                                ) const;

    // Interface implementation (QiAbstractModbus).
    QByteArray executeCustomFunction( const quint8 deviceAddress , const quint8 modbusFunction ,
                                      QByteArray &data , quint8 *const status = NULL ) const;

    // Interface implementation (QiAbstractModbus).
    QByteArray executeRaw( QByteArray &data , quint8 *const status = NULL ) const;

    // Interface implementation (QiAbstractModbus).
    QByteArray calculateCheckSum( QByteArray &data ) const;

signals:
    /*!
    * This signal is emitted if the connection to the modbus device was lost.
    */
    void connectionLost();
};
