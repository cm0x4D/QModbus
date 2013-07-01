/***********************************************************************************************************************
* QRtuModbus implementation.                                                                                          *
***********************************************************************************************************************/
#include <QRtuModbus>


/*** Qt includes ******************************************************************************************************/
#include <QtCore/QDataStream>
#include <QtEndian>


/*** System includes **************************************************************************************************/
#include <unistd.h>


/*** Definitions ******************************************************************************************************/

# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/
#include <sys/ioctl.h>

# /***/ ifndef Q_OS_MACX /*********************************************************************************************/
#include <linux/serial.h>
# /***/ endif /********************************************************************************************************/
inline void setRts( QFile & f , bool active )
{
    static int status = 0;
    if ( status == 0 ) ioctl( f.handle() , TIOCMGET , &status );
    active ? status |= TIOCM_RTS : status &= ~TIOCM_RTS;
    ioctl( f.handle() , TIOCMSET , &status );
}

#   define _read                _commPort.read
#   define _readAll             _commPort.readAll
#   define _readLine            _commPort.readLine

# /***/ ifndef Q_OS_MACX /*********************************************************************************************/
#   define _write( data )    do {                                                                                   \
                                    if ( _rtsDriveMode == RtsSoftwareActiveOnTx ||                                  \
                                         _rtsDriveMode == RtsSoftwareActiveOnRx )                                   \
                                        setRts( _commPort , _rtsDriveMode == RtsSoftwareActiveOnTx );               \
                                    _commPort.write( (data) );                                                      \
                                    if ( _rtsDriveMode == RtsSoftwareActiveOnTx ||                                  \
                                         _rtsDriveMode == RtsSoftwareActiveOnRx )                                   \
                                    {                                                                               \
                                        unsigned int lsr;                                                           \
                                        do {                                                                        \
                                            ioctl( _commPort.handle() , TIOCSERGETLSR , &lsr );                     \
                                        } while ( ( lsr & TIOCSER_TEMT ) == 0 );                                    \
                                        setRts( _commPort , _rtsDriveMode == RtsSoftwareActiveOnRx );               \
                                    }                                                                               \
                                } while( 0 )
# /***/ else /*********************************************************************************************************/
#   define _write( data )    do {                                                                                   \
                                    if ( _rtsDriveMode == RtsSoftwareActiveOnTx ||                                  \
                                         _rtsDriveMode == RtsSoftwareActiveOnRx )                                   \
                                        setRts( _commPort , _rtsDriveMode == RtsSoftwareActiveOnTx );               \
                                    _commPort.write( (data) );                                                      \
                                    if ( _rtsDriveMode == RtsSoftwareActiveOnTx ||                                  \
                                         _rtsDriveMode == RtsSoftwareActiveOnRx )                                   \
                                    {                                                                               \
                                        tcdrain( _commPort.handle() );                                              \
                                    }                                                                               \
                                } while( 0 )
# /***/ endif  /*******************************************************************************************************/

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/


/*** Abstract interface class destructor implementation ***************************************************************/
QAbstractModbus::~QAbstractModbus()
{}


/*** Class implementation *********************************************************************************************/
QRtuModbus::QRtuModbus() : _timeout( 500 ) , _rtsDriveMode( RtsNotDriven )
{}

QRtuModbus::~QRtuModbus()
{
    close();
}

bool QRtuModbus::open( const QString &device , const BaudRate baudRate , const StopBits stopBits , const Parity parity ,
                       const FlowControl flowControl , RtsDriveMode rtsDriveMode )
{

# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/

    // Setup the filename to use.
    _commPort.setFileName( device );

    // Try to open the serial com port.
    if ( !_commPort.open( QIODevice::ReadWrite | QIODevice::Unbuffered ) )
    {
        return false;
    }

    // Get the termois settings, change them and save them.
    struct termios settings;
    ::tcgetattr( _commPort.handle() , &settings );
    settings.c_cflag |= CREAD | CLOCAL;
    settings.c_lflag &= ~( ECHO | ECHOE | ECHOK | ECHONL | ISIG | ICANON );
    settings.c_iflag &= ~( INPCK | IGNPAR | PARMRK | ISTRIP | ICRNL | IXANY );
    settings.c_oflag &= ~OPOST;
    settings.c_cc[VINTR] = _POSIX_VDISABLE;
    settings.c_cc[VQUIT] = _POSIX_VDISABLE;
    settings.c_cc[VSTART] = _POSIX_VDISABLE;
    settings.c_cc[VSTOP] = _POSIX_VDISABLE;
    settings.c_cc[VSUSP] = _POSIX_VDISABLE;
    ::cfsetispeed( &settings , baudRate );
    ::cfsetospeed( &settings , baudRate );
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    settings.c_cflag &= ~( PARENB | PARODD );
    switch( parity )
    {
        case NoParity:
            break;

        case EvenParity:
            settings.c_cflag |= PARENB;
            break;

        case OddParity:
            settings.c_cflag |= PARENB | PARODD;
            break;
    }
    switch( stopBits )
    {
        case OneStopbit:
            settings.c_cflag &= ~CSTOPB;
            break;

        case TwoStopbits:
            settings.c_cflag |= CSTOPB;
            break;
    }
    switch( flowControl )
    {
        case NoFlowControl:
            settings.c_cflag &= ~CRTSCTS;
            settings.c_iflag &= ~( IXON | IXOFF | IXANY );
            break;

        case HardwareFlowControl:
            settings.c_cflag |= CRTSCTS;
            settings.c_iflag &= ~( IXON | IXOFF | IXANY );
            break;

        case XonXoffFlowControl:
            settings.c_cflag &= ~CRTSCTS;
            settings.c_iflag |= IXON | IXOFF | IXANY;
            break;
    }
    settings.c_cc[VTIME] = _timeout / 100;
    settings.c_cc[VMIN] = 0;
    ::tcsetattr( _commPort.handle() , TCSANOW , &settings );

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

# /***/ if defined( Q_OS_UNIX ) && ! defined( Q_OS_MACX ) /************************************************************/

    // Setup RTS handling:
    _rtsDriveMode = rtsDriveMode;
    switch ( _rtsDriveMode )
    {
        case RtsNotDriven:
            break;

        case RtsSoftwareActiveOnTx:
            setRts( _commPort , false );
            break;

        case RtsSoftwareActiveOnRx:
            setRts( _commPort , true );
            break;

        case RtsAutomaticActiveOnTx:
        {
            struct serial_rs485 rs485conf;
            memset( &rs485conf , 0 , sizeof( rs485conf ) );
            rs485conf.flags |= SER_RS485_ENABLED;
            rs485conf.flags |= SER_RS485_RTS_ON_SEND;

            if ( ioctl( _commPort.handle() , TIOCSRS485 , &rs485conf ) )
            {
                _commPort.close();
                return false;
            }
            break;
        }

        case RtsAutomaticActiveOnRx:
        {
            struct serial_rs485 rs485conf;
            memset( &rs485conf , 0 , sizeof( rs485conf ) );
            rs485conf.flags |= SER_RS485_ENABLED;
            rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;

            if ( ioctl( _commPort.handle() , TIOCSRS485 , &rs485conf ) )
            {
                _commPort.close();
                return false;
            }
            break;
        }
    }

# /***/ endif /* O_OS_UNIX && ! Q_OS_MACX *****************************************************************************/

# /***/ ifdef Q_OS_MACX /**********************************************************************************************/

    // TODO: add mac implementation of the RTS control feature...
    _rtsDriveMode = rtsDriveMode;
    if ( _rtsDriveMode != RtsNotDriven )
    {
        qDebug( "RTS Driving not implemented on Mac OS X!" );
        _commPort.close();
        return false;
    }

# /***/ endif /* Q_OS_MACX ********************************************************************************************/

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

    // Try to open the serial com port.
    _commPort = CreateFileA( device.toAscii().data() , GENERIC_READ | GENERIC_WRITE , 0 , 0 , OPEN_EXISTING ,
                             FILE_ATTRIBUTE_NORMAL , 0 );

    if ( _commPort == INVALID_HANDLE_VALUE )
    {
        return false;
    }

    // Get the DCB settings, change them and save them.
    DCB settings = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    GetCommState( _commPort , &settings );
    GetCommTimeouts( _commPort , &timeouts );
    settings.fBinary = TRUE;
    settings.fInX = FALSE;
    settings.fOutX = FALSE;
    settings.fAbortOnError = FALSE;
    settings.fNull = FALSE;
    settings.BaudRate = baudRate;
    settings.ByteSize = 8;
    switch( parity )
    {
        case NoParity:
            settings.fParity = FALSE;
            settings.Parity = NOPARITY;
            break;

        case EvenParity:
            settings.fParity = TRUE;
            settings.Parity = EVENPARITY;
            break;

        case OddParity:
            settings.fParity = TRUE;
            settings.Parity = ODDPARITY;
            break;
    }
    switch( stopBits )
    {
        case OneStopbit:
            settings.StopBits = ONESTOPBIT;
            break;

        case TwoStopbits:
            settings.StopBits = TWOSTOPBITS;
            break;
    }
    switch( flowControl )
    {
        case NoFlowControl:
            settings.fOutxCtsFlow = FALSE;
            settings.fRtsControl = RTS_CONTROL_DISABLE;
            settings.fInX = FALSE;
            settings.fOutX = FALSE;
            break;

        case HardwareFlowControl:
            settings.fOutxCtsFlow = TRUE;
            settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
            settings.fInX = FALSE;
            settings.fOutX = FALSE;
            break;

        case XonXoffFlowControl:
            settings.fOutxCtsFlow = false;
            settings.fRtsControl = RTS_CONTROL_DISABLE;
            settings.fInX = TRUE;
            settings.fOutX = TRUE;
            break;
    }
    timeouts.ReadIntervalTimeout = _timeout;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = _timeout;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = _timeout;

    SetCommState( _commPort , &settings );
    SetCommTimeouts( _commPort , &timeouts );

    // TODO: add win implementation of the RTS control feature...
    _rtsDriveMode = rtsDriveMode;
    if ( _rtsDriveMode != RtsNotDriven )
    {
        qDebug( "RTS Driving not implemented on Windows!" );
        _commPort.close();
        return false;
    }

# /***/ endif /* Q_OS_WIN *********************************************************************************************/

    // Ok, we are ready.
    return true;
}

bool QRtuModbus::isOpen() const
{

# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/

    // Delegate this to the QFile object.
    return _commPort.isOpen();

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

    // If we have a valid handle, we consider the port to be open...
    return ( _commPort != INVALID_HANDLE_VALUE );

# /***/ endif /* Q_OS_WIN *********************************************************************************************/

}

void QRtuModbus::close()
{

# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/

    // Close the file.
    _commPort.close();

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

    // Close the handle.
    CloseHandle( _commPort );
    _commPort = INVALID_HANDLE_VALUE;

# /***/ endif /* Q_OS_WIN *********************************************************************************************/

}

unsigned int QRtuModbus::timeout( void ) const
{
    return _timeout;
}

void QRtuModbus::setTimeout( const unsigned int timeout )
{
    _timeout = timeout;

    // If the file is open, change the timeout on the fly.
    if ( isOpen() )
    {

# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/

        struct termios settings;
        ::tcgetattr( _commPort.handle() , &settings );
        settings.c_cc[VTIME] = _timeout / 100;
        settings.c_cc[VMIN] = 0;
        ::tcsetattr( _commPort.handle() , TCSANOW , &settings );

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

        COMMTIMEOUTS timeouts = { 0 };
        GetCommTimeouts( _commPort , &timeouts );
        timeouts.ReadIntervalTimeout = _timeout;
        timeouts.ReadTotalTimeoutConstant = _timeout;
        timeouts.WriteTotalTimeoutConstant = _timeout;
        SetCommTimeouts( _commPort , &timeouts );

# /***/ endif /* Q_OS_WIN *********************************************************************************************/

    }
}

QList<bool> QRtuModbus::readCoils( const quint8 deviceAddress , const quint16 startingAddress ,
                                    const quint16 quantityOfCoils , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<bool>();
    }

    // Create modbus read coil status pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x01 << startingAddress << quantityOfCoils;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfCoils / 8;
    if ( quantityOfCoils % 8 ) neededRxBytes++;
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QList<bool>();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QList<bool>();
    }

    // Receive the rest.
    pdu += _read( neededRxBytes );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QList<bool>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 5 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x01 && byteCount == neededRxBytes )
        {
            // It is ok, read and convert the data.
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
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return QList<bool>();
}

QList<bool> QRtuModbus::readDiscreteInputs( const quint8 deviceAddress , const quint16 startingAddress ,
                                             const quint16 quantityOfInputs , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<bool>();
    }

    // Create modbus read coil status pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x02 << startingAddress << quantityOfInputs;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfInputs / 8;
    if ( quantityOfInputs % 8 ) neededRxBytes++;
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QList<bool>();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QList<bool>();
    }

    // Receive the rest of the message.
    pdu += _read( neededRxBytes );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QList<bool>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 5 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x02 && byteCount == neededRxBytes )
        {
            // It is ok, read and convert the data.
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
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return QList<bool>();
}

QList<quint16> QRtuModbus::readHoldingRegisters( const quint8 deviceAddress , const quint16 startingAddress ,
                                                  const quint16 quantityOfRegisters , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create modbus read holding registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x03 << startingAddress << quantityOfRegisters;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfRegisters * 2;
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QList<quint16>();
    }

    // Receive the rest of the message.
    pdu += _read( neededRxBytes );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 5 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x03 && byteCount == neededRxBytes )
        {
            // It is ok, read and convert the data.
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
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return QList<quint16>();
}

QList<quint16> QRtuModbus::readInputRegisters( const quint8 deviceAddress , const quint16 startingAddress ,
                                                const quint16 quantityOfInputRegisters , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create modbus read input status pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x04 << startingAddress << quantityOfInputRegisters;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityOfInputRegisters * 2;
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QList<quint16>();
    }

    // Receive the rest of the message.
    pdu += _read( neededRxBytes );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 5 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x04 && byteCount == neededRxBytes )
        {
            // It is ok, read and convert the data.
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
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return QList<quint16>();
}

bool QRtuModbus::writeSingleCoil( const quint8 deviceAddress , const quint16 outputAddress ,
                                   const bool outputValue , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create modbus write single coil pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x05 << outputAddress << ( outputValue ? (quint16)0xFF00 : (quint16)0x0000 );

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return false;
    }

    // Read the rest of the message.
    pdu += _read( 3 );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 8 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        quint16 rxOutputAddress , rxOutputValue;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> rxOutputAddress >> rxOutputValue;
        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x05 && rxOutputAddress == outputAddress &&
             ( ( outputValue && rxOutputValue == 0xFF00 ) || ( !outputValue && rxOutputValue == 0x0000 ) ) )
        {
            if ( status ) *status = Ok;
            return true;
        }
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return false;
}

bool QRtuModbus::writeSingleRegister( const quint8 deviceAddress , const quint16 outputAddress ,
                                       const quint16 registerValue , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create modbus write single coil pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x06 << outputAddress << registerValue;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return false;
    }

    // Read the rest of the message.
    pdu += _read( 3 );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 8 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        quint16 rxOutputAddress , rxRegisterValue;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> rxOutputAddress >> rxRegisterValue;
        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x06 && rxOutputAddress == outputAddress &&
             rxRegisterValue == registerValue )
        {
            if ( status ) *status = Ok;
            return true;
        }
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return false;
}

bool QRtuModbus::writeMultipleCoils( const quint8 deviceAddress , const quint16 startingAddress ,
                                      const QList<bool> & outputValues , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create modbus write multiple coil pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    quint8 txBytes = outputValues.count() / 8;
    if ( outputValues.count() % 8 != 0 ) txBytes++;
    pduStream << deviceAddress << (quint8)0x0F << startingAddress << (quint16)outputValues.count() << txBytes;

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

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return false;
    }

    // Read the rest of the message.
    pdu += _read( 3 );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 8 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        quint16 rxStartingAddress , rxQuantityOfOutputs;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> rxStartingAddress >> rxQuantityOfOutputs;
        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x0F && rxStartingAddress == startingAddress &&
             rxQuantityOfOutputs == outputValues.count() )
        {
            if ( status ) *status = Ok;
            return true;
        }
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return false;
}

bool QRtuModbus::writeMultipleRegisters( const quint8 deviceAddress , const quint16 startingAddress ,
                                          const QList<quint16> & registersValues , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create modbus write multiple registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    quint8 txBytes = registersValues.count() * 2;
    pduStream << deviceAddress << (quint8)0x10 << startingAddress << (quint16)registersValues.count() << txBytes;

    // Encode the register values.
    foreach ( quint16 reg , registersValues )
    {
        pduStream << reg;
    }

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return false;
    }

    // Read the rest of the message.
    pdu += _read( 3 );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 8 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        quint16 rxStartingAddress , rxQuantityOfRegisters;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> rxStartingAddress >> rxQuantityOfRegisters;
        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x10 && rxStartingAddress == startingAddress &&
             rxQuantityOfRegisters == registersValues.count() )
        {
            if ( status ) *status = Ok;
            return true;
        }
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return false;
}

bool QRtuModbus::maskWriteRegister( const quint8 deviceAddress , const quint16 referenceAddress ,
                                     const quint16 andMask , const quint16 orMask , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return false;
    }

    // Create modbus mask write register pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x16 << referenceAddress << andMask << orMask;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return false;
    }

    // Read the rest of the message.
    pdu += _read( 5 );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 10 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        quint16 rxReferenceAddress , rxAndMask , rxOrMask;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> rxReferenceAddress >> rxAndMask >> rxOrMask;
        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x16 &&
             rxReferenceAddress == referenceAddress && rxAndMask == andMask && rxOrMask == orMask )
        {
            if ( status ) *status = Ok;
            return true;
        }
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return false;
}


QList<quint16> QRtuModbus::writeReadMultipleRegisters( const quint8 deviceAddress ,
                                                        const quint16 writeStartingAddress ,
                                                        const QList<quint16> & writeValues ,
                                                        const quint16 readStartingAddress ,
                                                        const quint16 quantityToRead , quint8 *const status) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create modbus read holding registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x17 << readStartingAddress << quantityToRead
              << writeStartingAddress << (quint16)writeValues.count() << (quint16)( writeValues.count() * 2 );

    // Add data.
    foreach ( quint16 reg , writeValues )
    {
        pduStream << reg;
    }

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    quint16 neededRxBytes = quantityToRead * 2;
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QList<quint16>();
    }

    // Read the rest of the message.
    pdu += _read( neededRxBytes );

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 5 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode , byteCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> byteCount;

        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x17 && byteCount == neededRxBytes )
        {
            // Generate response data.
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
    }

    // If we arrive here, something went wrong.
    if ( status ) *status = UnknownError;
    return QList<quint16>();
}

QList<quint16> QRtuModbus::readFifoQueue( const quint8 deviceAddress , const quint16 fifoPointerAddress ,
                                           quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create modbus read holding registers pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << (quint8)0x18 << fifoPointerAddress;

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    pdu.clear();

    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QList<quint16>();
    }

    // Read the rest of the message.
    pdu += _readAll();

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() >= 10 )
    {
        // Read TCP fields and, device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        quint16 byteCount , fifoCount;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode >> byteCount >> fifoCount;

        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == 0x18 && byteCount == fifoCount * 2 )
        {
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
    }

    // Something went wrong...
    if ( status ) *status = UnknownError;
    return QList<quint16>();
}

QByteArray QRtuModbus::executeCustomFunction( const quint8 deviceAddress , const quint8 modbusFunction ,
                                               QByteArray &data , quint8 *const status ) const
{
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QByteArray();
    }

    // Create modbus pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << modbusFunction;
    pduStream.writeRawData( data.data() , data.size() );

    // Calculate CRC and add it to the PDU.
    quint16 crc = _calculateCrc( pdu );
    pduStream.writeRawData( (const char *)&crc , sizeof( crc ) );

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the pdu.
    _write( pdu );

    // Await response.
    // Even on error we have at least 5 bytes to read.
    pdu = _read( 5 );

    // Handle timeout.
    if ( pdu.size() < 5 )
    {
        if ( status ) *status = Timeout;
        return QByteArray();
    }

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( !_checkCrc( pdu ) )
        {
            if ( status ) *status = CrcError;
        }
        else
        {
            if ( status ) *status = pdu[2];
        }
        return QByteArray();
    }

    // Read the rest of the message.
    pdu += _readAll();

    // Check CRC.
    if ( !_checkCrc( pdu ) )
    {
        if ( status ) *status = CrcError;
        return QByteArray();
    }

    // Check data and return them on success.
    if ( pdu.size() >= 4 )
    {
        // Read device address and command ID and control them.
        quint8 rxDeviceAddress , rxFunctionCode;
        QDataStream rxStream( pdu );
        rxStream.setByteOrder( QDataStream::BigEndian );
        rxStream >> rxDeviceAddress >> rxFunctionCode;

        // Control values of the fields.
        if ( rxDeviceAddress == deviceAddress && rxFunctionCode == modbusFunction )
        {
            // Convert data.
            return pdu.mid( 2 ,  pdu.size() - 4 );
        }
    }

    // Something went wrong...
    if ( status ) *status = UnknownError;
    return QByteArray();
}

QByteArray QRtuModbus::executeRaw( QByteArray &data , quint8 *const status ) const
{
    QByteArray response;

    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QByteArray();
    }

    // Clear the RX buffer before making the request.
    _readAll();

    // Send the data.
    _write( data );

    // Await response.
    // Even on error we have at least 5 bytes to read.
    response = _readAll();

    // Handle timeout.
    if ( response.size() == 0 )
    {
        if ( status ) *status = Timeout;
        return QByteArray();
    }

    return response;
}

QByteArray QRtuModbus::calculateCheckSum( QByteArray &data ) const
{
    quint16 crc = _calculateCrc( data );
    return QByteArray( (char *)&crc , 2 );
}


# /***/ ifdef Q_OS_WIN /***********************************************************************************************/
#include <QtDebug>
QByteArray QRtuModbus::_read( const int numberBytes ) const
{
    QByteArray data( numberBytes , 0 );
    DWORD size = 0;

    if ( numberBytes == 0 ) return data;

    if ( ReadFile( _commPort , data.data() , numberBytes , &size , NULL ) )
    {
        data.resize( size );
    }
    else
    {
        data.resize( 0 );
    }

    return data;
}

QByteArray QRtuModbus::_readAll( void ) const
{
    QByteArray data( 1024 , 0 );
    DWORD size = 0;
    if ( ReadFile( _commPort , data.data() , 1024 , &size , NULL ) )
    {
        data.resize( size );
    }
    else
    {
        data.resize( 0 );
    }

    return data;
}

QByteArray QRtuModbus::_readLine( int maxBytes ) const
{
    QByteArray data;
    DWORD size = 0;

    if ( maxBytes == 0 ) return data;

    while( !data.endsWith( '\n' ) && --maxBytes )
    {
        unsigned char c;
        if ( ReadFile( _commPort , &c , 1 , &size , NULL ) )
        {
           data.append( c );
        }
    }

    return data;
}

bool QRtuModbus::_write( QByteArray &data ) const
{
    DWORD size = 0;

    if ( data.size() == 0 ) return true;

    if ( WriteFile( _commPort , data.constData() , data.size() , &size , NULL ) )
    {
        return ( (int)size == data.size() );
    }
    return false;
}

# /***/ endif /* Q_OS_WIN *********************************************************************************************/


quint16 QRtuModbus::_calculateCrc( const QByteArray &pdu ) const
{
    quint16 crc , carryFlag , tmp;

    crc = 0xFFFF;
    for ( int i = 0 ; i < pdu.size() ; i++ )
    {
        crc = crc ^ (unsigned char)pdu.at( i );
        for (int j = 0 ; j < 8 ; j++ )
        {
            tmp = crc;
            carryFlag = tmp & 0x0001;
            crc = crc >> 1;
            if ( carryFlag == 1 )
            {
                crc = crc ^ 0xA001;
            }
        }
    }

    return crc;
}

bool QRtuModbus::_checkCrc( const QByteArray &pdu ) const
{
    QByteArray msg = pdu.left( pdu.size() - 2 );

    quint16 crc = _calculateCrc( msg );

    return pdu.endsWith( QByteArray( (const char *)&crc , 2 ) );
}
