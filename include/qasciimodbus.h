/***********************************************************************************************************************
* QiRtuModbus : Local (serial port) support for RTU Modbus devices                                                     *
************************************************************************************************************************
* Author : Michael Clausen (michael.clausen@hevs.ch)                                                                   *
* Date   : 2009/07/14                                                                                                  *
* Changes: -                                                                                                           *
***********************************************************************************************************************/
#pragma once


/*** Base class *******************************************************************************************************/
#include <QAbstractModbus>


/*** Qt includes & prototypes *****************************************************************************************/
#include <QtCore/QFile>
#include <QtCore/QString>
#include <QtCore/QList>
#include <QtCore/QByteArray>


/*** System includes **************************************************************************************************/

#ifdef Q_OS_UNIX /*****************************************************************************************************/

#   include <termios.h>

#endif /* Q_OS_UNIX ***************************************************************************************************/

#ifdef Q_OS_WIN /******************************************************************************************************/

#   include <windows.h>

#endif /* Q_OS_WIN ****************************************************************************************************/


/*** QiAsciiModbus class declaration and help *************************************************************************/
/*!
* The QiAsciiModbus class talks to local attached (serial port) modbus slave devices using the modbus protocol in ASCII
* mode.
* \headerfile qiasciimodbus.h QiAsciiModbus
*/
class QAsciiModbus : public QAbstractModbus
{
public:
    /*!
    * Specifies the baudrate.
    */
    enum BaudRate
    {

# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/

#           ifdef B50
                BR50        = B50 ,     //!< 50 baud.
#           endif
#           ifdef B75
                BR75        = B75 ,     //!< 75 baud.
#           endif
#           ifdef B110
                BR110       = B110 ,    //!< 110 baud.
#           endif
#           ifdef B134
                BR134       = B134 ,    //!< 134 baud.
#           endif
#           ifdef B150
                BR150       = B150 ,    //!< 150 baud.
#           endif
#           ifdef B200
                BR200       = B200 ,    //!< 200 baud.
#           endif
#           ifdef B300
                BR300       = B300 ,    //!< 300 baud.
#           endif
#           ifdef B600
                BR600       = B600 ,    //!< 600 baud.
#           endif
#           ifdef B1200
                BR1200      = B1200 ,   //!< 1200 baud.
#           endif
#           ifdef B1800
                BR1800      = B1800 ,   //!< 1800 baud.
#           endif
#           ifdef B2400
                BR2400      = B2400 ,   //!< 2400 baud.
#           endif
#           ifdef B4800
                BR4500      = B4800 ,   //!< 4800 baud.
#           endif
#           ifdef B9600
                BR9600      = B9600 ,   //!< 9600 baud.
#           endif
#           ifdef B19200
                BR19200     = B19200 ,  //!< 19200 baud.
#           endif
#           ifdef B38400
                BR38400     = B38400 ,  //!< 38400 baud.
#           endif
#           ifdef B57600
                BR57600     = B57600 ,  //!< 57600 baud.
#           endif
#           ifdef B115200
                BR115200    = B115200 , //!< 115200 baud.
#           endif
#           ifdef B230400
                BR230400    = B230400 , //!< 230400 baud (if supported by the OS).
#           endif
#           ifdef B460800
                BR460800    = B460800 , //!< 460800 baud (if supported by the OS).
#           endif
#           ifdef B500000
                BR500000    = B500000 , //!< 500 kbaud (if supported by the OS).
#           endif
#           ifdef B576000
                BR576000    = B576000 , //!< 576000 baud (if supported by the OS).
#           endif
#           ifdef B921600
                BR921600    = B921600 , //!< 921600 baud (if supported by the OS).
#           endif
#           ifdef B1000000
                BR1000000   = B1000000 ,//!< 1Mbaud baud (if supported by the OS).
#           endif
#           ifdef B1152000
                BR1152000   = B1152000 ,//!< 1152000baud (if supported by the OS).
#           endif
#           ifdef B1500000
                BR1500000   = B1500000 ,//!< 1.5 Mbaud (if supported by the OS).
#           endif
#           ifdef B2000000
                BR2000000   = B2000000 ,//!< 1 Mbaud (if supported by the OS).
#           endif
#           ifdef B2500000
                BR2500000   = B2500000 ,//!< 2.5 Mbaud (if supported by the OS).
#           endif
#           ifdef B3000000
                BR3000000   = B3000000 ,//!< 3 Mbaud (if supported by the OS).
#           endif
#           ifdef B3500000
                BR3500000   = B3500000 ,//!< 3.5 Mbaud (if supported by the OS).
#           endif
#           ifdef B4000000
                BR4000000   = B4000000  //!< 4 Mbaud (if supported by the OS).
#           endif

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

#           ifdef CBR_50
                BR50        = CBR_50 ,     //!< 50 baud.
#           endif
#           ifdef CBR_75
                BR75        = CBR_75 ,     //!< 75 baud.
#           endif
#           ifdef CBR_110
                BR110       = CBR_110 ,    //!< 110 baud.
#           endif
#           ifdef CBR_134
                BR134       = CBR_134 ,    //!< 134 baud.
#           endif
#           ifdef CBR_150
                BR150       = CBR_150 ,    //!< 150 baud.
#           endif
#           ifdef CBR_200
                BR200       = CBR_200 ,    //!< 200 baud.
#           endif
#           ifdef CBR_300
                BR300       = CBR_300 ,    //!< 300 baud.
#           endif
#           ifdef CBR_600
                BR600       = CBR_600 ,    //!< 600 baud.
#           endif
#           ifdef CBR_1200
                BR1200      = CBR_1200 ,   //!< 1200 baud.
#           endif
#           ifdef CBR_1800
                BR1800      = CBR_1800 ,   //!< 1800 baud.
#           endif
#           ifdef CBR_2400
                BR2400      = CBR_2400 ,   //!< 2400 baud.
#           endif
#           ifdef CBR_4800
                BR4500      = CBR_4800 ,   //!< 4800 baud.
#           endif
#           ifdef CBR_9600
                BR9600      = CBR_9600 ,   //!< 9600 baud.
#           endif
#           ifdef CBR_19200
                BR19200     = CBR_19200 ,  //!< 19200 baud.
#           endif
#           ifdef CBR_38400
                BR38400     = CBR_38400 ,  //!< 38400 baud.
#           endif
#           ifdef CBR_57600
                BR57600     = CBR_57600 ,  //!< 57600 baud.
#           endif
#           ifdef CBR_115200
                BR115200    = CBR_115200 , //!< 115200 baud.
#           endif
#           ifdef CBR_230400
                BR230400    = CBR_230400 , //!< 230400 baud (if supported by the OS).
#           endif
#           ifdef CBR_460800
                BR460800    = CBR_460800 , //!< 460800 baud (if supported by the OS).
#           endif
#           ifdef CBR_500000
                BR500000    = CBR_500000 , //!< 500 kbaud (if supported by the OS).
#           endif
#           ifdef CBR_576000
                BR576000    = CBR_576000 , //!< 576000 baud (if supported by the OS).
#           endif
#           ifdef CBR_921600
                BR921600    = CBR_921600 , //!< 921600 baud (if supported by the OS).
#           endif
#           ifdef CBR_1000000
                BR1000000   = CBR_1000000 ,//!< 1Mbaud baud (if supported by the OS).
#           endif
#           ifdef CBR_1152000
                BR1152000   = CBR_1152000 ,//!< 1152000baud (if supported by the OS).
#           endif
#           ifdef CBR_1500000
                BR1500000   = CBR_1500000 ,//!< 1.5 Mbaud (if supported by the OS).
#           endif
#           ifdef CBR_2000000
                BR2000000   = CBR_2000000 ,//!< 1 Mbaud (if supported by the OS).
#           endif
#           ifdef CBR_2500000
                BR2500000   = CBR_2500000 ,//!< 2.5 Mbaud (if supported by the OS).
#           endif
#           ifdef CBR_3000000
                BR3000000   = CBR_3000000 ,//!< 3 Mbaud (if supported by the OS).
#           endif
#           ifdef CBR_3500000
                BR3500000   = CBR_3500000 ,//!< 3.5 Mbaud (if supported by the OS).
#           endif
#           ifdef CBR_4000000
                BR4000000   = CBR_4000000  //!< 4 Mbaud (if supported by the OS).
#           endif

# /***/ endif /* Q_OS_WIN *********************************************************************************************/

    };

    /*!
    * Specifies the used number of bits for a character. Modbus supports 7 and 8.
    */
    enum BitsPerCharacter
    {
# /***/ ifdef Q_OS_UNIX /**********************************************************************************************/

#           ifdef CS7
                BPC7 = CS7 ,    //!< 7 bits per character.
#           endif
#           ifdef CS8
                BPC8 = CS8 ,    //!< 8 bits per character.
#           endif

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

                BPC7 = 7 ,      //!< 7 bits per character.
                BPC8 = 8 ,      //!< 8 bits per character.

# /***/ endif /* Q_OS_UNIX ********************************************************************************************/

    };

    /*!
    * Specifies the used number of stop bits in communication with the slave.
    */
    enum StopBits
    {
        OneStopbit  = 1 ,   //!< One stop bit.
        TwoStopbits = 2     //!< Two stop bits.
    };

    /*!
    * Specifies the used parity mechanism in communication with the slave.
    */
    enum Parity
    {
        NoParity    = 0x00 ,    //!< No parity at all.
        EvenParity  = 0x01 ,    //!< Even parity.
        OddParity   = 0x02      //!< Odd parity.
    };

    /*!
    * Specifies the used flow control in communication with the slave.
    */
    enum FlowControl
    {
        NoFlowControl       = 0x00 ,    //!< No flow control.
        HardwareFlowControl = 0x01 ,    //!< Hardware CRT/RTS flow control.
        XonXoffFlowControl  = 0x02      //!< Xon/Xoff Software flow control.
    };

private:

#   ifdef Q_OS_UNIX /**************************************************************************************************/

        mutable QFile _commPort;            // File used to speach with the serial port.

#   endif /* Q_OS_UNIX ************************************************************************************************/

#   ifdef Q_OS_WIN /***************************************************************************************************/

        mutable HANDLE _commPort;           // File used to speach with the serial port.

#   endif /* Q_OS_WIN *************************************************************************************************/

    unsigned int _timeout;                  // Timeout to use in serial communication.

public:
    /*!
    * Constructor.
    */
    QAsciiModbus();

    /*!
    * Destructor.
    */
    virtual ~QAsciiModbus();

    /*!
    * Tries to open a local serial port.
    * \param device The device to use for communication, on Linux this will be something like "dev/ttySx" and on
    *               window this is some sort of "COMx" where the x stands for the number.
    * \param baudRate The baudrate to use. Default is 9600 baud.
    * \param bitPerCharacter Number of bits per character to use in serial communication with Modbus slave.
    * \param stopBits The number of stopbits to use. Default is one.
    * \param parity Parity mechanism to use. Default is none.
    * \param flowControl Flow control mechanism to use. Default is none.
    * \return True if the port could be openend, false otherwise.
    */
    bool open( const QString &device , const BaudRate baudRate = BR9600 ,
               const BitsPerCharacter bitPerCharacter = BPC7 , const StopBits stopBits = OneStopbit ,
               const Parity parity = NoParity , const FlowControl flowControl = NoFlowControl );

    /*!
    * Returns the state of the serial port handle.
    * \return True if the port is open and ready to use, false otherwise.
    */
    bool isOpen() const;

    /*!
    * Closes the connection to the slave modbus device.
    */
    void close();

    // Interface implementation (QiAbstractModbus).
    unsigned int timeout( void ) const;

    // Interface implementation (QiAbstractModbus).
    void setTimeout( const unsigned int timeout );

    // Interface implementation (QiAbstractModbus).
    QList<bool> readCoils( const quint8 deviceAddress ,
                           const quint16 startingAddress ,
                           const quint16 quantityOfCoils ,
                           quint8 *const quint8 = NULL
                         ) const;

    // Interface implementation (QiAbstractModbus).
    QList<bool> readDiscreteInputs( const quint8 deviceAddress ,
                                    const quint16 startingAddress ,
                                    const quint16 quantityOfInputs ,
                                    quint8 *const quint8 = NULL
                                  ) const;

    // Interface implementation (QiAbstractModbus).
    QList<quint16> readHoldingRegisters( const quint8 deviceAddress ,
                                         const quint16 startingAddress ,
                                         const quint16 quantityOfRegisters ,
                                         quint8 *const quint8 = NULL
                                       ) const;

    // Interface implementation (QiAbstractModbus).
    QList<quint16> readInputRegisters( const quint8 deviceAddress ,
                                       const quint16 startingAddress ,
                                       const quint16 quantityOfInputRegisters ,
                                       quint8 *const quint8 = NULL
                                     ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeSingleCoil( const quint8 deviceAddress ,
                          const quint16 outputAddress ,
                          const bool outputValue ,
                          quint8 *const quint8 = NULL
                        ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeSingleRegister( const quint8 deviceAddress ,
                              const quint16 registerAddress ,
                              const quint16 registerValue ,
                              quint8 *const quint8 = NULL
                            ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeMultipleCoils( const quint8 deviceAddress ,
                             const quint16 startingAddress ,
                             const QList<bool> & outputValues ,
                             quint8 *const quint8 = NULL
                           ) const;

    // Interface implementation (QiAbstractModbus).
    bool writeMultipleRegisters( const quint8 deviceAddress ,
                                 const quint16 startingAddress ,
                                 const QList<quint16> & registersValues ,
                                 quint8 *const quint8 = NULL
                               ) const;

    // Interface implementation (QiAbstractModbus).
    bool maskWriteRegister( const quint8 deviceAddress ,
                            const quint16 referenceAddress ,
                            const quint16 andMask ,
                            const quint16 orMask ,
                            quint8 *const quint8 = NULL
                          ) const;

    // Interface implementation (QiAbstractModbus).
    QList<quint16> writeReadMultipleRegisters( const quint8 deviceAddress ,
                                               const quint16 writeStartingAddress ,
                                               const QList<quint16> & writeValues ,
                                               const quint16 readStartingAddress ,
                                               const quint16 quantityToRead ,
                                               quint8 *const quint8 = NULL
                                             ) const;


    // Interface implementation (QiAbstractModbus).
    QList<quint16> readFifoQueue( const quint8 deviceAddress ,
                                  const quint16 fifoPointerAddress ,
                                  quint8 *const quint8 = NULL
                                ) const;

    // Interface implementation (QiAbstractModbus).
    QByteArray executeCustomFunction( const quint8 deviceAddress , const quint8 modbusFunction ,
                                      QByteArray &data , quint8 *const status = NULL ) const;

    // Interface implementation (QiAbstractModbus).
    QByteArray executeRaw( QByteArray &data , quint8 *const status = NULL ) const;

    // Interface implementation (QiAbstractModbus).
    QByteArray calculateCheckSum( QByteArray &data ) const;

private:

# /***/ ifdef Q_OS_WIN /***********************************************************************************************/

    QByteArray _read( const int numberBytes ) const;
    QByteArray _readAll( void ) const;
    QByteArray _readLine( int maxBytes ) const;
    bool _write( QByteArray &data ) const;

# /***/ endif /* Q_OS_WIN *********************************************************************************************/

    // Used to perform LRC on outgoing modbus messages.
    quint8 _calculateLrc( const QByteArray &pdu ) const;

    // Used to check LRC on incomming modbus messages.
    bool _checkLrc( const QByteArray &pdu ) const;
};

