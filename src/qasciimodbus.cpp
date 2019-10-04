#include <QAsciiModbus>
#include <QtCore/QDataStream>
#include <unistd.h>

#ifdef Q_OS_UNIX
#   define _read        _commPort.read
#   define _readAll     _commPort.readAll
#   define _readLine    _commPort.readLine
#   define _write       _commPort.write
#endif

QAsciiModbus::QAsciiModbus(): _timeout( 500 ) {}

QAsciiModbus::~QAsciiModbus() {
    close();
}

bool QAsciiModbus::open(const QString &device, BaudRate baudRate, BitsPerCharacter bitPerCharacter,
                        StopBits stopBits, Parity parity, FlowControl flowControl) {
#   ifdef Q_OS_UNIX

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
    settings.c_cflag |= bitPerCharacter;
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
    settings.c_cc[VTIME] = static_cast<cc_t>(_timeout / 100);
    settings.c_cc[VMIN] = 0;
    ::tcsetattr( _commPort.handle() , TCSANOW , &settings );

#   endif

#   ifdef Q_OS_WIN

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
    settings.ByteSize = bitPerCharacter;
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

#   endif

    // Ok, we are ready.
    return true;
}

bool QAsciiModbus::isOpen() const {
#   ifdef Q_OS_UNIX

    // Delegate this to the QFile object.
    return _commPort.isOpen();

#   endif

#   ifdef Q_OS_WIN

    // If we have a valid handle, we consider the port to be open...
    return ( _commPort != INVALID_HANDLE_VALUE );

#   endif
}

void QAsciiModbus::close() {
#   ifdef Q_OS_UNIX

    // Close the file.
    _commPort.close();

#   endif

#   ifdef Q_OS_WIN

    // Close the handle.
    CloseHandle( _commPort );
    _commPort = INVALID_HANDLE_VALUE;

#   endif
}

unsigned int QAsciiModbus::timeout() const {
    return _timeout;
}

void QAsciiModbus::setTimeout(unsigned int timeout) {
    _timeout = timeout;

    // If the file is open, change the timeout on the fly.
    if ( isOpen() )
    {
#   ifdef Q_OS_UNIX

        struct termios settings;
        ::tcgetattr( _commPort.handle() , &settings );
        settings.c_cc[VTIME] = static_cast<cc_t>(_timeout / 100);
        settings.c_cc[VMIN] = 0;
        ::tcsetattr( _commPort.handle() , TCSANOW , &settings );

#   endif

#   ifdef Q_OS_WIN

        COMMTIMEOUTS timeouts = { 0 };
        GetCommTimeouts( _commPort , &timeouts );
        timeouts.ReadIntervalTimeout = _timeout;
        timeouts.ReadTotalTimeoutConstant = _timeout;
        timeouts.WriteTotalTimeoutConstant = _timeout;
        SetCommTimeouts( _commPort , &timeouts );

#   endif
    }
}

QList<bool> QAsciiModbus::readCoils(quint8 deviceAddress, quint16 startingAddress, quint16 quantityOfCoils,
                                    quint8* const status) const {
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<bool>();
    }

    // Create modbus read coil quint8 pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << static_cast<quint8>(0x01) << startingAddress << quantityOfCoils;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    quint16 neededRxBytes = quantityOfCoils / 8;
    if ( quantityOfCoils % 8 ) neededRxBytes++;
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QList<bool>();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QList<bool>();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return QList<bool>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 3 )
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
            quint8 tmp = 0;
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

QList<bool> QAsciiModbus::readDiscreteInputs(quint8 deviceAddress, quint16 startingAddress, quint16 quantityOfInputs,
                                             quint8* const status) const {
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<bool>();
    }

    // Create modbus read coil quint8 pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << static_cast<quint8>(0x02) << startingAddress << quantityOfInputs;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    quint16 neededRxBytes = quantityOfInputs / 8;
    if ( quantityOfInputs % 8 ) neededRxBytes++;
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QList<bool>();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QList<bool>();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return QList<bool>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 3 )
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
            quint8 tmp = 0;
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

QList<quint16> QAsciiModbus::readHoldingRegisters(quint8 deviceAddress, quint16 startingAddress,
                                                  quint16 quantityOfRegisters, quint8* const status) const {
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
    pduStream << deviceAddress << static_cast<quint8>(0x03) << startingAddress << quantityOfRegisters;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    quint16 neededRxBytes = quantityOfRegisters * 2;
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 3 )
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

QList<quint16> QAsciiModbus::readInputRegisters(quint8 deviceAddress, quint16 startingAddress,
                                                quint16 quantityOfInputRegisters, quint8* const status) const {
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QList<quint16>();
    }

    // Create modbus read input quint8 pdu (Modbus uses Big Endian).
    QByteArray pdu;
    QDataStream pduStream( &pdu , QIODevice::WriteOnly );
    pduStream.setByteOrder( QDataStream::BigEndian );
    pduStream << deviceAddress << static_cast<quint8>(0x04) << startingAddress << quantityOfInputRegisters;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    quint16 neededRxBytes = quantityOfInputRegisters * 2;
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 3 )
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

bool QAsciiModbus::writeSingleCoil(quint8 deviceAddress, quint16 outputAddress, bool outputValue,
                                   quint8* const status) const {
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
    pduStream << deviceAddress << static_cast<quint8>(0x05) << outputAddress
              << static_cast<quint16>( outputValue ? 0xFF00 : 0x0000 );

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 6 )
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

bool QAsciiModbus::writeSingleRegister(quint8 deviceAddress, quint16 outputAddress, quint16 registerValue,
                                       quint8* const status) const {
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
    pduStream << deviceAddress << static_cast<quint8>(0x06) << outputAddress << registerValue;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 6 )
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

bool QAsciiModbus::writeMultipleCoils(quint8 deviceAddress, quint16 startingAddress, const QList<bool>& outputValues,
                                      quint8* const status) const {
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
    quint8 txBytes = static_cast<quint8>(outputValues.count() / 8);
    if ( outputValues.count() % 8 != 0 ) txBytes++;
    pduStream << deviceAddress << static_cast<quint8>(0x0F) << startingAddress
              << static_cast<quint16>(outputValues.count()) << txBytes;

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

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 6 )
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

bool QAsciiModbus::writeMultipleRegisters(quint8 deviceAddress, quint16 startingAddress,
                                          const QList<quint16>& registersValues, quint8* const status) const {
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
    quint8 txBytes = static_cast<quint8>(registersValues.count() * 2);
    pduStream << deviceAddress << static_cast<quint8>(0x10) << startingAddress
              << static_cast<quint16>(registersValues.count()) << txBytes;

    // Encode the register values.
    foreach ( quint16 reg , registersValues )
    {
        pduStream << reg;
    }

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 6 )
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

bool QAsciiModbus::maskWriteRegister(quint8 deviceAddress, quint16 referenceAddress, quint16 andMask, quint16 orMask,
                                     quint8* const status) const {
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
    pduStream << deviceAddress << static_cast<quint8>(0x16) << referenceAddress << andMask << orMask;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return false;
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return false;
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return false;
    }

    // Check data and return them on success.
    if ( pdu.size() == 8 )
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

QList<quint16> QAsciiModbus::writeReadMultipleRegisters(quint8 deviceAddress, quint16 writeStartingAddress,
                                                        const QList<quint16>& writeValues, quint16 readStartingAddress,
                                                        quint16 quantityToRead, quint8* const status) const {
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
    pduStream << deviceAddress << static_cast<quint8>(0x17) << readStartingAddress << quantityToRead
              << writeStartingAddress << static_cast<quint16>(writeValues.count())
              << static_cast<quint16>( writeValues.count() * 2 );

    // Add data.
    foreach ( quint16 reg , writeValues )
    {
        pduStream << reg;
    }

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    quint16 neededRxBytes = quantityToRead * 2;
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() == neededRxBytes + 3 )
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

QList<quint16> QAsciiModbus::readFifoQueue(quint8 deviceAddress, quint16 fifoPointerAddress,
                                           quint8* const status ) const {
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
    pduStream << deviceAddress << static_cast<quint8>(0x18) << fifoPointerAddress;

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 200 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QList<quint16>();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QList<quint16>();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
        return QList<quint16>();
    }

    // Check data and return them on success.
    if ( pdu.size() >= 8 )
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

QByteArray QAsciiModbus::executeCustomFunction(quint8 deviceAddress, quint8 modbusFunction, const QByteArray& data,
                                               quint8* const status) const {
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

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += pdu.toHex().toUpper();

    // Calculate LRC and add it to the PDU.
    hexEncoded += QString( "%1" ).arg( _calculateLrc( pdu ) , 2 , 16 , QChar( '0' ) ).toUpper();
    hexEncoded += 0x0D;
    hexEncoded += 0x0A;

    // Send the pdu.
    _write( hexEncoded );

    // Await response.
    pdu.clear();
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 2000 );

    // Handle timeout.
    if ( hexEncoded.size() < 9 )
    {
        if ( status ) *status = Timeout;
        return QByteArray();
    }

    // Check LRC.
    if ( !_checkLrc( hexEncoded ) )
    {
        if ( status ) *status = CrcError;
        return QByteArray();
    }

    // Get the hex decoded form.
    pdu = QByteArray::fromHex( hexEncoded.mid( 1 , hexEncoded.size() -5 ) );

    // Was it a Modbus error?
    if ( pdu[1] & 0x80 )
    {
        if ( status ) *status = static_cast<quint8>(pdu[2]);
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

QByteArray QAsciiModbus::executeRaw(const QByteArray& data, quint8* const status) const {
    // Are we connected ?
    if ( !isOpen() )
    {
        if ( status ) *status = NoConnection;
        return QByteArray();
    }

    // Encode to hex.
    QByteArray hexEncoded( ":" );
    hexEncoded += data.toHex().toUpper();
    // Send the data.
    _write( hexEncoded );

    // Await response.
    hexEncoded.clear();

    // Try to read line.
    hexEncoded = _readLine( 2000 );

    // Handle timeout.
    if ( hexEncoded.size() == 0 )
    {
        if ( status ) *status = Timeout;
        return QByteArray();
    }

    // Get the hex decoded form.
    return QByteArray::fromHex( hexEncoded );
}

QByteArray QAsciiModbus::calculateCheckSum(const QByteArray& data) const {
    quint8 lrc = _calculateLrc( data );
    return QByteArray( reinterpret_cast<char *>(&lrc) , 1 );
}

#ifdef Q_OS_WIN

QByteArray QAsciiModbus::_read(int numberBytes) const {
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

QByteArray QAsciiModbus::_readAll() const {
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

QByteArray QAsciiModbus::_readLine(int maxBytes) const {
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

bool QAsciiModbus::_write(const QByteArray& data) const {
    DWORD size = 0;

    if ( data.size() == 0 ) return true;

    if ( WriteFile( _commPort , data.constData() , data.size() , &size , NULL ) )
    {
        return ( (int)size == data.size() );
    }
    return false;
}

#endif

quint8 QAsciiModbus::_calculateLrc(const QByteArray& pdu) const {
    qint8 nLRC = 0 ;

    for ( int i = 0 ; i < pdu.size() ; i++ )
        nLRC += pdu[i];

    return static_cast<quint8>(-nLRC);
}

bool QAsciiModbus::_checkLrc(const QByteArray& pdu) const {
    if ( pdu.size() < 6 ) return false;

    QByteArray msg = QByteArray::fromHex( pdu.mid( 1 , pdu.size() - 5 ) );
    quint8 oLrc = static_cast<quint8>(QByteArray::fromHex( pdu.mid( pdu.size() - 4 , 2 ) )[0]);
    quint8 lrc = _calculateLrc( msg );

    return oLrc == lrc;
}
