#pragma once
#include <QtCore/QList>

/*!
* Base class for all modbus protocol specific classes, unites all common methods that a modbus class implementation has
* to support. With this it is possible to access all modbus related methods using a base class pointer which enables
* modbus application programming independent of the actually used modbus protocol variant.
*
* \headerfile qiabstractmodbus.h QiAbstractModbus
*/
class QAbstractModbus {
  public:
    /*!
    * This enumeration defines the different status possible after the execution of a momdbus access.
    */
    enum Status {
        Ok                                  = 0x00 ,    //!< Success.
        IllegalFunction                     = 0x01 ,    //!< Modbus function not supported by the device.
        IllegalDataAddress                  = 0x02 ,    //!< Address is out the devices address range.
        IllegalDataValue                    = 0x03 ,    //!< A given parameter is invalid.
        SlaveDeviceFailure                  = 0x04 ,    //!< The slave device has made a failure.
        Acknowledge                         = 0x05 ,    //!< Basic acknowledge.
        SlaveDeviceBusy                     = 0x06 ,    //!< Slave is busy.
        MemoryParityError                   = 0x07 ,    //!< Fatal parity error in modbus slave.
        GatewayPathUnavailable              = 0x0A ,    //!< Gateway has no path to device (TcpModbus).
        GatewayTargetDeviceFailedToRespond  = 0x0B ,    //!< Gateway has no response received from slave (TcpModbus).
        CrcError                            = 0x10 ,    //!< The received modbus message is corrupted.
        Timeout                             = 0x11 ,    //!< There was an timeout (slave did not respond in time).
        NoConnection                        = 0x12 ,    //!< No connection to the slave possible.
        UnknownError                        = 0xFF      //!< An unknown error happened.
    };

    virtual ~QAbstractModbus();

    /*!
     * Returns true if the ASCII or RTU modbus ports are open or the TCP modbus is connected, false otherwise.
     *
     * \return True if open/connected, false otherwise.
     */
    virtual bool isOpen() const = 0;

    /*!
    * Returns the currently used timeout in milliseconds.
    *
    * \return Timeout in milliseconds.
    */
    virtual unsigned int timeout() const = 0;

    /*!
    * Changes the timeout setting.
    *
    * \param timeout Timeout in milliseconds.
    */
    virtual void setTimeout(unsigned int timeout) = 0;

    /*!
    * This method is used to read from 1 to 2000 contiguous status of coils (Outputs) in a remote device. The
    * parameters specify the starting address, i.e. the address of the first coil specified, and the number of coils.
    * Coils are addressed starting at zero. Therefore coils numbered 1-16 are addressed as 0-15. The coils in the
    * result of the method are packed as one coil entry in the QList. Status is indicated as true = ON and false = OFF.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param startingAddress Starting address [0..65535].
    * \param quantityOfCoils Number of coils [1..2000].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return A list of all coil states bits coded as bools in a QList (true=ON,false=OFF).
    */
    virtual QList<bool> readCoils(quint8 deviceAddress, quint16 startingAddress,
                                  quint16 quantityOfCoils, quint8* const status = NULL) const = 0;

    /*!
    * This method is used to read from 1 to 2000 contiguous status of discrete inputs in a remote device. The
    * parameters specify the starting address, i.e. the address of the first input specified, and the number of inputs.
    * Discrete Inputs are addressed starting at zero. Therefore Discrete inputs numbered 1-16 are addressed as 0-15.
    * The discrete inputs in the result of the method are packed as one input per entry in the QList. Status is
    * indicated as true = ON and false = OFF.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param startingAddress Starting address [0..65535].
    * \param quantityOfInputs Number of inputs [1..2000].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return A list of all input states bits coded as bools in a QList (true=ON,false=OFF).
    */
    virtual QList<bool> readDiscreteInputs(quint8 deviceAddress, quint16 startingAddress,
                                           quint16 quantityOfInputs, quint8* const status = NULL) const = 0;

    /*!
    * This method is used to read the contents of a contiguous block of holding registers in a remote device. The
    * parameters specify the starting register address and the number of registers. Registers are addressed starting
    * at zero. Therefore registers numbered 1-16 are addressed as 0-15. The register data in the method's result are
    * packed as one list entry per register inside a QList.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param startingAddress Starting address [0..65535].
    * \param quantityOfRegisters Number of registers [1..125].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return The contents of the holding registers inside a QList.
    */
    virtual QList<quint16> readHoldingRegisters(quint8 deviceAddress, quint16 startingAddress,
                                                quint16 quantityOfRegisters, quint8* const status = NULL) const = 0;

    /*!
    * This method is used to read the contents of a contiguous block of input registers in a remote device. The
    * parameters specify the starting register address and the number of registers. Registers are addressed starting
    * at zero. Therefore registers numbered 1-16 are addressed as 0-15. The register data in the method's result are
    * packed as one list entry per register inside a QList.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param startingAddress Starting address [0..65535].
    * \param quantityOfInputRegisters Number of registers [1..125].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return The contents of the input registers inside a QList.
    */
    virtual QList<quint16> readInputRegisters(quint8 deviceAddress, quint16 startingAddress,
                                              quint16 quantityOfInputRegisters,
                                              quint8* const status = NULL) const = 0;

    /*!
    * This method is used to write a single output to either ON or OFF in a remote device. The requested ON/OFF state
    * is specified by a constant in the request data field. A value of true requests the output to be ON. A value of
    * false requests it to be OFF. Coils are addressed starting at zero. Therefore coil numbered 1 is addressed as 0.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param outputAddress Output (coil) address [0..65535].
    * \param outputValue true for ON and false for OFF.
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return True on success, false otherwise (Error number can be retrieved using the status pointer).
    */
    virtual bool writeSingleCoil(quint8 deviceAddress, quint16 outputAddress, bool outputValue,
                                 quint8* const status = NULL) const = 0;

    /*!
    * This function code is used to write a single holding register in a remote device. The parameters specify the
    * address of the register to be written. Registers are addressed starting at zero. Therefore register numbered 1
    * is addressed as 0.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param registerAddress Register address [0..65535].
    * \param registerValue Value to write to the register [0..65535].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return True on success, false otherwise (Error number can be retrieved using the status pointer).
    */
    virtual bool writeSingleRegister(quint8 deviceAddress, quint16 registerAddress, quint16 registerValue,
                                     quint8* const status = NULL) const = 0;

    /*!
    * This fmethod is used to force each coil in a sequence of coils to either ON or OFF in a remote device. The
    * parameters specify the coil references to be forced. Coils are addressed starting at zero. Therefore coil
    * numbered 1 is addressed as 0. The requested ON/OFF states are specified by a list of boolean values. True
    * requests the corresponding output to be ON. False requests it to be OFF.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param startingAddress The address of the first output (coil) [0..65535].
    * \param outputValues A list containing all desired output values for the coils. Note that the number of coils to
    *                     write to is derived from the list. Maximal 1968 values in list are supported by modbus.
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return True on success, false otherwise (Error number can be retrieved using the status pointer).
    */
    virtual bool writeMultipleCoils(quint8 deviceAddress, quint16 startingAddress, const QList<bool>& outputValues,
                                    quint8* const status = NULL) const = 0;

    /*!
    * This method is used to write a block of contiguous registers (1 to 123 registers) in a remote device. The
    * requested written values are specified in the given list of 16 bit unsigned int values.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param startingAddress The address of the first register [0..65535].
    * \param registersValues A list containing all desired output values for the registers. Note that the number of
    *                        registers is derived from the list and the miximal amount of registers is 123.
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return True on success, false otherwise (Error number can be retrieved using the status pointer).
    */
    virtual bool writeMultipleRegisters(quint8 deviceAddress, quint16 startingAddress,
                                        const QList<quint16>& registersValues,
                                        quint8* const status = NULL) const = 0;

    /*!
    * This method is used to modify the contents of a specified holding register using a combination of an AND mask,
    * an OR mask, and the register's current contents. This functionality can be used to set or clear individual bits
    * in the register. The parameters specify the holding register to be written, the data to be used as the AND
    * mask, and the data to be used as the OR mask. Registers are addressed starting at zero. Therefore register 1 is
    * addressed as 0.
    * The function’s algorithm is: Result = ( Current Contents AND And_Mask ) OR ( Or_Mask AND (NOT And_Mask) )
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param referenceAddress Address of the holding register to be modified [0..65535].
    * \param andMask The mask to use for the AND function.
    * \param orMask The mask to use for the OR function.
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return True on success, false otherwise (Error number can be retrieved using the status pointer).
    */
    virtual bool maskWriteRegister(quint8 deviceAddress, quint16 referenceAddress, quint16 andMask,
                                   quint16 orMask, quint8* const status = NULL) const = 0;

    /*!
    * This method performs a combination of one read operation and one write operation in a single MODBUS transaction.
    * The write operation is performed before the read. Holding registers are addressed starting at zero. Therefore
    * holding registers 1-16 are addressed as 0-15. The parameters specify the starting address and number of holding
    * registers to be read as well as the starting address and the data to be written.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param writeStartingAddress Starting register address for write operation [0..65535].
    * \param writeValues List with all values to be written. Note that up to 121 registers can be written.
    * \param readStartingAddress Starting register address for read operation [0..65535].
    * \param quantityToRead Number of registers to read [0..125].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return A list containing all the read register values organized inside a QList.
    */
    virtual QList<quint16> writeReadMultipleRegisters(quint8 deviceAddress, quint16 writeStartingAddress,
                                                      const QList<quint16>& writeValues,  quint16 readStartingAddress,
                                                      quint16 quantityToRead, quint8* const status = NULL) const = 0;

    /*!
    * This method allows to read the contents of a First-In-First-Out (FIFO) queue of register in a remote device.
    * It returns the queued data. Up to 31 registers can be read.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param fifoPointerAddress Address of the FIFO [0..65535].
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return A list containing all the available FIFO values organized inside a QList.
    */
    virtual QList<quint16> readFifoQueue(quint8 deviceAddress, quint16 fifoPointerAddress,
                                         quint8* const status = NULL) const = 0;

    /*!
    * This method executes a "special" modbus function, where you can mostly free decide how the data of the message
    * will look like. The only fields that are automatically generated by the class are the device id (parameter), the
    * function code (parameter) and if needed a CRC/LRC is automatically generated for you. All the other data should
    * be passed to the method using the data byte array. The result of the method is the received data WITHOUT device
    * address, function code and even the CRC/LRC is automatically removed from the message. If the received message is
    * an exception/error response, the status variable is set according to this error.
    *
    * \param deviceAddress Address of the slave device [1..247].
    * \param modbusFunction Modbus function to execute [0..255].
    * \param data Data to append to the common modbus header.
    * \param status Pointer to a variable that will contain the transaction status after method execution. If NULL
    *               status will not be reported at all.
    * \return Byte array of the received data section or an empty array if the modbus function failed.
    */
    virtual QByteArray executeCustomFunction(quint8 deviceAddress, quint8 modbusFunction,
                                             const QByteArray& data, quint8* const status = NULL) const = 0;

    /*!
    * This method speakes raw to the device.
    *
    * \param data Data to send.
    * \param status Pointer to a variable that will contain the transaction status after transfer. If NULL
    *               status will not be reported at all.
    * \return Byte array of the whole received data (including checksum) or empty array if the modbus function failed.
    */
    virtual QByteArray executeRaw(const QByteArray& data, quint8* const status = NULL) const = 0;

    /*!
     * Calculates the checksum for the given data.
     * \param data Data to use to generate the checksum.
     *
     * \return The checksum.
     */
    virtual QByteArray calculateCheckSum(const QByteArray& data) const = 0;
};

