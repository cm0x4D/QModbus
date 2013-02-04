/***********************************************************************************************************************
* QTcpModbus imlpementation.                                                                                          *
***********************************************************************************************************************/
#include <QTcpModbus>


/*** Qt includes ******************************************************************************************************/
#include <QtCore/QByteArray>
#include <QtCore/QDataStream>


/*** Class implementation *********************************************************************************************/
QTcpModbus::QTcpModbus() : _timeout( 500 ) , _connectTimeout( 1000 )
{
    // Initialize random number generator to use for transaction ID generation.
    qsrand( time( NULL ) );

    // Connect the socket's connection lost signal to my connection lost signal.
    QObject::connect( &_socket , SIGNAL( disconnected() ) , this , SIGNAL( connectionLost() ) );
}

QTcpModbus::~QTcpModbus()
{
    // Finaly disconnect.
    disconnect();
}

bool QTcpModbus::connect( const QString &host , const quint16 port )
{
    // Connect the socket to the host.
    _socket.connectToHost( host , port );

    // Wait until we have the connection established.
    return _socket.waitForConnected( _connectTimeout );
}

bool QTcpModbus::isConnected( void ) const
{
    // Ask the socket if it is connected.
    return ( _socket.state() == QAbstractSocket::ConnectedState );
}

void QTcpModbus::disconnect( void )
{
    // Close the socket's connection.
    _socket.close();
}

int QTcpModbus::connectTimeout( void ) const
{
    return _connectTimeout;
}

void QTcpModbus::setConnectTimeout( const int timeout )
{
    _connectTimeout = timeout;
}

unsigned int QTcpModbus::timeout( void ) const
{
    return _timeout;
}

void QTcpModbus::setTimeout( const unsigned int timeout )
{
    _timeout = timeout;
}

QList<bool> QTcpModbus::readCoils( const quint8 deviceAddress , const quint16 startingAddress ,
                                    const quint16 quantityOfCoils , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QList<bool>();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create tcp/modbus read coil status pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)6
              << deviceAddress << (quint8)0x01 << startingAddress << quantityOfCoils;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfCoils / 8;
    if ( quantityOfCoils % 8 ) neededRxBytes++;
    pdu.clear();
    while ( pdu.size() < neededRxBytes + 9 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( neededRxBytes + 9 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 9 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength;
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == neededRxBytes + 3 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x01 && byteCount == neededRxBytes )
        {
            // Convert data.
            QList<bool> list;
            quint8 tmp;
            for ( int i = 0 ; i < quantityOfCoils ; i++ )
            {
                if ( i % 8 == 0 ) rxStream >> tmp;
                list.append( tmp & ( 0x01 << ( i % 8 ) ) );
            }
            if ( status ) *status = Ok;
            return list;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QList<bool>();
}

QList<bool> QTcpModbus::readDiscreteInputs( const quint8 deviceAddress , const quint16 startingAddress ,
                                             const quint16 quantityOfInputs , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QList<bool>();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create tcp/modbus read input status pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)6
              << deviceAddress << (quint8)0x02 << startingAddress << quantityOfInputs;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfInputs / 8;
    if ( quantityOfInputs % 8 ) neededRxBytes++;
    pdu.clear();
    while ( pdu.size() < neededRxBytes + 9 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( neededRxBytes + 9 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 9 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength;
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == neededRxBytes + 3 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x02 && byteCount == neededRxBytes )
        {
            // Convert data.
            QList<bool> list;
            quint8 tmp;
            for ( int i = 0 ; i < quantityOfInputs ; i++ )
            {
                if ( i % 8 == 0 ) rxStream >> tmp;
                list.append( tmp & ( 0x01 << ( i % 8 ) ) );
            }
            if ( status ) *status = Ok;
            return list;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QList<bool>();
}

QList<quint16> QTcpModbus::readHoldingRegisters( const quint8 deviceAddress , const quint16 startingAddress ,
                                                  const quint16 quantityOfRegisters , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create tcp/modbus read holding registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)6
              << deviceAddress << (quint8)0x03 << startingAddress << quantityOfRegisters;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfRegisters * 2;
    pdu.clear();
    while ( pdu.size() < neededRxBytes + 9 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( neededRxBytes + 9 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 9 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength;
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == neededRxBytes + 3 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x03 && byteCount == neededRxBytes )
        {
            // Convert data.
            QList<quint16> list;
            quint16 tmp;
            for ( int i = 0 ; i < quantityOfRegisters ; i++ )
            {
                rxStream >> tmp;
                list.append( tmp );
            }
            if ( status ) *status = Ok;
            return list;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QList<quint16>();
}

QList<quint16> QTcpModbus::readInputRegisters( const quint8 deviceAddress , const quint16 startingAddress ,
                                                const quint16 quantityOfInputRegisters , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus read input status pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)6
              << deviceAddress << (quint8)0x04 << startingAddress << quantityOfInputRegisters;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfInputRegisters * 2;
    pdu.clear();
    while ( pdu.size() < neededRxBytes + 9 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( neededRxBytes + 9 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 9 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength;
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == neededRxBytes + 3 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x04 && byteCount == neededRxBytes )
        {
            // Convert data.
            QList<quint16> list;
            quint16 tmp;
            for ( int i = 0 ; i < quantityOfInputRegisters ; i++ )
            {
                rxStream >> tmp;
                list.append( tmp );
            }
            if ( status ) *status = Ok;
            return list;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QList<quint16>();
}

bool QTcpModbus::writeSingleCoil( const quint8 deviceAddress , const quint16 outputAddress ,
                                   const bool outputValue , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus write single coil pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)6
              << deviceAddress << (quint8)0x05 << outputAddress << ( outputValue ? (quint16)0xFF00 : (quint16)0x0000 );

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    while ( pdu.size() < 12 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( 12 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == 12 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength , rxOutputAddress , rxOutputValue;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode
                 >> rxOutputAddress >> rxOutputValue;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == 6 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x05 && rxOutputAddress == outputAddress &&
             ( ( outputValue && rxOutputValue == 0xFF00 ) || ( !outputValue && rxOutputValue == 0x0000 ) ) )
        {
            // Ok, done.
            if ( status ) *status = Ok;
            return true;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return false;
}

bool QTcpModbus::writeSingleRegister( const quint8 deviceAddress , const quint16 outputAddress ,
                                       const quint16 registerValue , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus write single coil pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)6
              << deviceAddress << (quint8)0x06 << outputAddress << registerValue;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    while ( pdu.size() < 12 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( 12 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == 12 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength , rxOutputAddress , rxRegisterValue;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode
                 >> rxOutputAddress >> rxRegisterValue;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == 6 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x06 && rxOutputAddress == outputAddress &&
             rxRegisterValue == registerValue )
        {
            // Ok, done.
            if ( status ) *status = Ok;
            return true;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return false;
}

bool QTcpModbus::writeMultipleCoils( const quint8 deviceAddress , const quint16 startingAddress ,
                                      const QList<bool> & outputValues , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus write multiple coil pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    quint8 txBytes = outputValues.count() / 8;
    if ( outputValues.count() % 8 != 0 ) txBytes++;
    pduStream << transactionId << (quint16)0 << (quint16)( txBytes + 7 )
              << deviceAddress << (quint8)0x0F << startingAddress << (quint16)outputValues.count() << txBytes;

    // Encode the binary values.
    quint8 tmp = 0;
    for ( int i = 0 ; i < outputValues.count() ; i++ )
    {
        if ( i % 8 == 0 )
        {
            if ( i != 0 ) pduStream << tmp;
            tmp = 0;
        }
        if ( outputValues[i] ) tmp |= 0x01 << ( i % 8 );
    }
    pduStream << tmp;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    while ( pdu.size() < 12 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( 12 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == 12 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength , rxStartingAddress , rxQuantityOfOutputs;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode
                 >> rxStartingAddress >> rxQuantityOfOutputs;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == 6 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x0F && rxStartingAddress == startingAddress &&
             rxQuantityOfOutputs == outputValues.count() )
        {
            // Ok, done.
            if ( status ) *status = Ok;
            return true;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return false;
}

bool QTcpModbus::writeMultipleRegisters( const quint8 deviceAddress , const quint16 startingAddress ,
                                          const QList<quint16> & registersValues , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus write multiple registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    quint8 txBytes = registersValues.count() * 2;
    pduStream << transactionId << (quint16)0 << (quint16)( txBytes + 7 )
              << deviceAddress << (quint8)0x10 << startingAddress << (quint16)registersValues.count() << txBytes;

    // Encode the register values.
    foreach ( quint16 reg , registersValues )
    {
        pduStream << reg;
    }

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    while ( pdu.size() < 12 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( 12 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == 12 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength , rxStartingAddress , rxQuantityOfRegisters;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode
                 >> rxStartingAddress >> rxQuantityOfRegisters;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == 6 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x10 && rxStartingAddress == startingAddress &&
             rxQuantityOfRegisters == registersValues.count() )
        {
            // Ok, done.
            if ( status ) *status = Ok;
            return true;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return false;
}

bool QTcpModbus::maskWriteRegister( const quint8 deviceAddress , const quint16 referenceAddress ,
                                     const quint16 andMask , const quint16 orMask , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus mask write register pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)8
              << deviceAddress << (quint8)0x16 << referenceAddress << andMask << orMask;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    while ( pdu.size() < 14 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( 14 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == 14 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength , rxReferenceAddress , rxAndMask , rxOrMask;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode
                 >> rxReferenceAddress >> rxAndMask >> rxOrMask;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == 8 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x16 &&
             rxReferenceAddress == referenceAddress && rxAndMask == andMask && rxOrMask == orMask )
        {
            // Ok, done.
            if ( status ) *status = Ok;
            return true;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return false;
}


QList<quint16> QTcpModbus::writeReadMultipleRegisters( const quint8 deviceAddress ,
                                                        const quint16 writeStartingAddress ,
                                                        const QList<quint16> & writeValues ,
                                                        const quint16 readStartingAddress ,
                                                        const quint16 quantityToRead , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus read holding registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)( writeValues.count() * 2 + 12 )
              << deviceAddress << (quint8)0x17 << readStartingAddress << quantityToRead
              << writeStartingAddress << (quint16)writeValues.count() << (quint16)( writeValues.count() * 2 );

    // Add data.
    foreach ( quint16 reg , writeValues )
    {
        pduStream << reg;
    }

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityToRead * 2;
    pdu.clear();
    while ( pdu.size() < neededRxBytes + 9 )
    {
        if ( ( pdu.size() >= 9 && pdu[7] & 0x80 ) || !_socket.waitForReadyRead( _timeout ) ) break;
        pdu += _socket.read( neededRxBytes + 9 - pdu.size() );
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 9 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength;
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == neededRxBytes + 3 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x17 && byteCount == neededRxBytes )
        {
            // Convert data.
            QList<quint16> list;
            quint16 tmp;
            for ( int i = 0 ; i < quantityToRead ; i++ )
            {
                rxStream >> tmp;
                list.append( tmp );
            }
            if ( status ) *status = Ok;
            return list;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QList<quint16>();
}

QList<quint16> QTcpModbus::readFifoQueue( const quint8 deviceAddress , const quint16 fifoPointerAddress ,
                                           quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus read FIFO registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)4
              << deviceAddress << (quint8)0x18 << fifoPointerAddress;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    if ( !_socket.waitForReadyRead( _timeout ) )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }
    pdu = _socket.readAll();

    // Check data and return them on success.
    if ( pdu.size() >= 12 && !( pdu[7] & 0x80 ) )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength , byteCount , fifoCount;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode
                 >> byteCount >> fifoCount;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxLength == byteCount - 2 &&
             rxDeviceAddress == deviceAddress && rxFunctionCode == 0x18 && byteCount == fifoCount * 2 &&
             pdu.size() == rxLength + 6 )
        {
            // Convert data.
            QList<quint16> list;
            quint16 tmp;
            for ( int i = 0 ; i < fifoCount ; i++ )
            {
                rxStream >> tmp;
                list.append( tmp );
            }
            if ( status ) *status = Ok;
            return list;
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QList<quint16>();
}

QByteArray QTcpModbus::executeCustomFunction( const quint8 deviceAddress , const quint8 modbusFunction ,
                                               QByteArray &data , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QByteArray();
    }

    // Create a transaction number.
    quint16 transactionId = qrand();

    // Create modbus pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << transactionId << (quint16)0 << (quint16)( data.size() + 2 )
              << deviceAddress << modbusFunction;
    pdu += data;

    // Send the pdu.
    _socket.write( pdu );

    // Await response.
    pdu.clear();
    if ( !_socket.waitForReadyRead( _timeout ) )
    {
        if ( status ) *status = Timeout;
        return QByteArray();
    }
    pdu = _socket.readAll();

    // Check data and return them on success.
    if ( pdu.size() >= 9 && !( pdu[7] & 0x80 ) )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint16 rxTransactionId , rxProtocolId, rxLength;
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxTransactionId >> rxProtocolId >> rxLength >> rxDeviceAddress >> rxFunctionCode;

        // Control values of the fields.
        if ( rxTransactionId == transactionId && rxProtocolId == 0 && rxDeviceAddress == deviceAddress &&
             rxFunctionCode == modbusFunction && pdu.size() == rxLength + 6 )
        {
            // Convert data.
            return pdu.right( pdu.size() - 8 );
        }
        else
        {
            if ( status ) *status = UnknownError;
        }
    }
    else
    {
        // What was wrong ?
        if ( pdu.size() == 9 )
        {
            if ( status ) *status = pdu[8];
        }
        else
        {
            if ( status ) *status = Timeout;
        }
    }

    return QByteArray();
}

QByteArray QTcpModbus::executeRaw( QByteArray &data , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isConnected() )
    {
        if ( status ) *status = NoConnection;
        return QByteArray();
    }

    // Send the data.
    _socket.write( data );

    // Await response.
    if ( !_socket.waitForReadyRead( _timeout ) )
    {
        if ( status ) *status = Timeout;
        return QByteArray();
    }
    return _socket.readAll();
}

QByteArray QTcpModbus::calculateCheckSum( QByteArray &data ) const
{
    Q_UNUSED( data );
    return QByteArray();
}
