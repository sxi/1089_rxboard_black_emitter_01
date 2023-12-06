/*------------------------------------------------------------------------------
 *
 * $Workfile:   DeviceDefines.h  $ $Modtime:   Aug 22 2018 14:17:00  $
 *
 * Copyright (C) 2021 Banner Engineering Corp. All rights reserved.
 * Developed by Sensonix Incorporated
 *------------------------------------------------------------------------------
 */


 #ifndef DEVICEDEFINES_H_
 #define DEVICEDEFINES_H_

#define FALSE                             ( 0 )
#define TRUE                              (!FALSE)

#define NUMBER_OF_ADJUSTMENTS             255

//Index Definitions
#define INDEX_START            0

//Sleep Defines
#define LO_NYBBLE_MASK         0x0F



//Modbus Defines
#define MODBUS_READ_MULTIPLE_REGISTERS         0x03
#define MODBUS_READ_MULTIPLE_MAX_REGISTERS     125
#define READ_MULTIPLE_REGISTERS_DATA_SIZE_IN   7    // Fuction Code        (1 byte)
                                                    // Statring Address    (2 bytes)
                                                    // Number Of Registers (2 bytes)
                                                    // CRC                 (2 bytes)
                                                    // Note: Do NOT include modbus address

#define READ_MULTIPLE_REGISTERS_DATA_SIZE_OUT  4    // Fuction Code        (1 byte)
                                                    // Byte Count          (1 bytes)
                                                    // CRC                 (2 bytes)
                                                    // Note: Do NOT include modbus address
                                                    //       Do Not include Register Values


#define MODBUS_WRITE_SINGLE_REGISTER           0x06
#define WRITE_SINGLE_REGISTER_DATA_SIZE_IN_OUT 7    // Fuction Code        (1 byte)
                                                    // Statring Address    (2 bytes)
                                                    // Registers Value     (2 bytes)
                                                    // CRC                 (2 bytes)
                                                    // Note: Do NOT include modbus address

#define MODBUS_WRITE_MULTIPLE_REGISTERS        0x10
#define MODBUS_WRITE_MULTIPLE_MAX_REGISTERS    120
#define WRITE_MULTIPLE_REGISTERS_DATA_SIZE_IN  8    // Fuction Code        (1 byte)
                                                    // Statring Address    (2 bytes)
                                                    // Number Of Registers (2 bytes)
                                                    // Byte Count          (1 bytes)
                                                    // CRC                 (2 bytes)
                                                    // Note: Do NOT include modbus address
                                                    //       Do Not include Register Values

#define WRITE_MULTIPLE_REGISTERS_DATA_SIZE_OUT 7    // Fuction Code        (1 byte)
                                                    // Statring Address    (2 bytes)
                                                    // Number Of Registers (2 bytes)
                                                    // CRC                 (2 bytes)
                                                    // Note: Do NOT include modbus address

#define MODBUS_NACK_DATA_SIZE_OUT              4    // Fuction Code        (1 byte)
                                                    // Error Code          (1 bytes)
                                                    // CRC                 (2 bytes)
                                                    // Note: Do NOT include modbus address

#define MODBUS_NACK_BIT_MASK                                0x80
#define MODBUS_NACK_NO_ERROR                                0x00
#define MODBUS_NACK_DEVICE_NOT_IN_NETWORK                   0x81
#define MODBUS_NACK_DEVICE_NOT_HEALTHY                      0x82

#define MODBUS_REGISTER_SIZE                                2
#define EXTENTED_DEVICE_NUMBER_MODBUS_ADDRESS               0xFA
#define DEVICE_NUMBER_MODBUS_ADDRESS_SIZE                   3
#define MASTER_PING_MODBUS_ADDRESS                          0xFB
#define MASTER_PING_MODBUS_ADDRESS_SIZE                     2
#define MUX_WITH_32BIT_LABEL_MODBUS_ADDRESS                 0xFC
        // NOTE: see MuxAddressIndexType
#define SERIAL_NUMBER_MODBUS_ADDRESS                        0xFD
#define SERIAL_NUMBER_MODBUS_ADDRESS_SIZE                   5
#define BROADCAST_MODBUS_ADDRESS                            0x00
#define STANDARD_MODBUS_ADDRESS_SIZE                        1
#define MODBUS_CRC_SIZE                                     2
// Maximum modbus message size in bytes
#define MODBUS_MESSAGE_SIZE_MAX                             ( 250 )

// Modbus Table Fuction Definitions
#define NO_EEPROM_WRITE 0xFFFF
#define READ_ONLY       0xFFFE

#define MAXIMUM_DATA_PACKET_IN_FRAMELET_SIZE          25
#define MODBUS_WRITE_MULTIPLE_PROCESS_TIME_FROM_BYTES (MAXIMUM_DATA_PACKET_IN_FRAMELET_SIZE*2)
#define DATA_FRAMELETS_PER_SLOT           2

#define REX_NET_INVERT_MASK_BYTE            0x47

#endif

