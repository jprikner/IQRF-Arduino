// *********************************************************************
//   Sketch to cooperate with CustomDpaHandler-Bridge-UART.c           *
// *********************************************************************
// Copyright (c) IQRF Tech s.r.o.
//
// File:    $RCSfile: CustomDpaHandler-Bridge-UART.ino,v $
// Version: $Revision: 1.7 $
// Date:    $Date: 2021/02/22 17:55:18 $
//
// Revision history:
//   2019/03/01  Release for DPA 4.01
//   2018/10/25  Release for DPA 3.03
//
// *********************************************************************

// Please see CustomDpaHandler-Bridge-UART.c for implementation details.

// This Sketch implements one standard IQRF sensor
// Type of the sensor is Binary7

// include necessary core libraries
#include "stdint.h"
#include "Arduino.h"

// Include IQRF DPA headers
#include "DPA.h"
#include "IQRFstandard.h"
#include "IQRF_HWPID.h"

// include own created libraries
#include "IQRFservo.h"
#include "IQRFbattery.h"

// number of sensors currently in use (for future use - to be implemented)
#define SENSORS_NR 2

//############################################################################################
// servo - specific declarations
//############################################################################################

// IQRF-specific type of servo object
IQRFservo servo ;

// Declare a pin which the servo should be connected to (currently pin D9)
int servoPin = 9 ;
// Servo position as demanded from coordinator
byte servoPos = 0;
// Delay in [ms] that affects the servo turning speed. 
// Higher value means slower speed.
// Please choose value from the interval of <10,100>.
int delayTime = 10;

//############################################################################################
// battery - specific declarations
//############################################################################################

// IQRF-specific type of battery object
IQRFbattery battery ;

//############################################################################################
void setup()
//############################################################################################
{
  Serial.begin ( 115200 );

  // Setup UART to connect to IQRF TR module
  Serial1.begin ( 115200 );

  // initialise servo object with the selected pin to attach
  servo.init ( servoPin );

  //  perform a battery voltage check after boot
  battery.checkBatteryLevel () ;
}

//############################################################################################

// Returns sensor value
byte GetSensor0Value()
{
  return 0;
}

//############################################################################################

// HDLC byte stuffing bytes
// Flag Sequence
#define   HDLC_FRM_FLAG_SEQUENCE    0x7e
// Asynchronous Control Escape
#define   HDLC_FRM_CONTROL_ESCAPE   0x7d
// Asynchronous transparency modifier
#define   HDLC_FRM_ESCAPE_BIT       0x20

// Flag to DpaEvent_DpaRequest event value to indicate return TRUE not FALSE
#define EVENT_RETURN_TRUE           0x80
// Flag to DpaEvent_DpaRequest event value to report error, error value is the 1st data byte
#define EVENT_RESPONSE_ERROR        0x40

// Received data from IQRF
byte RxBuffer[2 * sizeof( byte ) + sizeof( TDpaIFaceHeader ) + sizeof( TDpaMessage )];

// Data from IQRF length limits
#define MIN_RX_PACKET_DATA_LENGTH  2
#define MAX_RX_PACKET_DATA_LENGTH  sizeof( RxBuffer )

//############################################################################################
// Sends one byte to IQRF
void TxByte( byte data )
//############################################################################################
{
  Serial1.write( data );
}

//############################################################################################
// Sends one HDLC byte to IQRF
void TxHdlcByte( byte data )
//############################################################################################
{
  switch ( data )
  {
    default:
      TxByte( data );
      return;

    case HDLC_FRM_FLAG_SEQUENCE:
    case HDLC_FRM_CONTROL_ESCAPE:
    {
      TxByte( HDLC_FRM_CONTROL_ESCAPE );
      TxByte( data ^ HDLC_FRM_ESCAPE_BIT );
      return;
    }
  }
}

//############################################################################################
// Returns FRC value back to IQRF
void ResponseFRCvalue( unsigned long frcValue )
//############################################################################################
{
  // Start packet
  TxByte( HDLC_FRM_FLAG_SEQUENCE );
  // Send event value
  TxHdlcByte( DpaEvent_FrcValue );
  // Send FRC value up to 4 bytes
  TxHdlcByte( frcValue & 0xFF );
  TxHdlcByte( ( frcValue >> 8 ) & 0xFF );
  TxHdlcByte( ( frcValue >> 16 ) & 0xFF );
  TxHdlcByte( ( frcValue >> 24 ) & 0xFF );
  // Stop packet
  TxByte( HDLC_FRM_FLAG_SEQUENCE );
}

//############################################################################################
// Return DPA response back to IQRF
void ResponseCommand( byte returnFlags, byte _DpaDataLength, byte dataLength, byte *pData )
//############################################################################################
{
  // Start packet
  TxByte( HDLC_FRM_FLAG_SEQUENCE );
  // Send event value
  TxHdlcByte( DpaEvent_DpaRequest | returnFlags );
  // Send DPA variable data length (must not equal to the actual data length sent)
  TxHdlcByte( _DpaDataLength );
  // Send DPA response data
  for ( ; dataLength != 0; dataLength-- )
    TxHdlcByte( *pData++ );
  // Stop packet
  TxByte( HDLC_FRM_FLAG_SEQUENCE );
}

//############################################################################################
// Packet from Custom DPA Handler was received
void CustomDpaHandler( byte dataLength )
//############################################################################################
{
  // if true, then set the servo to requested position after performing DPA response 
  // otherwise, delays during servo operation could cause DPA General Fail (timeout)
  bool turnServo = false ;  

  // Which Custom DPA Handler event to handle?
  switch ( RxBuffer[0] )
  {
    // Prepare DPA response to DPA request
    case DpaEvent_DpaRequest:
      if ( dataLength >= ( 2 * sizeof( byte ) + sizeof( TDpaIFaceHeader ) ) )
      {
        // Fake important DPA variables for the DPA Request/Response so the Custom DPA handler code will can be written almost same way on both platforms
#define _DpaDataLength  (RxBuffer[1])
#define _NADR           (RxBuffer[2])
#define _NADRhigh       (RxBuffer[3])
#define _PNUM           (RxBuffer[4])
#define _PCMD           (RxBuffer[5])
#define _HWPIDlow       (RxBuffer[6])
#define _HWPIDhigh      (RxBuffer[7])
#define _DpaMessage     (*((TDpaMessage*)(RxBuffer+8)))

      // Fake Custom DPA Handler macro to return DPA error (this macro does not do return the same way the DPA original macro)
#define DpaApiReturnPeripheralError(error) do { \
  _DpaMessage.ErrorAnswer.ErrN = error; \
  returnDataLength = _DpaDataLength = sizeof( _DpaMessage.ErrorAnswer.ErrN ); \
  returnFlags = EVENT_RESPONSE_ERROR | EVENT_RETURN_TRUE; \
  } while( 0 )

        // Value or error flag to return from Custom DPA handler
        byte returnFlags = 0;
        // Length data to return (may not equal to _DpaDataLength)
        byte returnDataLength = 0;
        // Device enumeration?
        if ( IsDpaEnumPeripheralsRequest() )
        {
          // We implement 1 user peripheral
          _DpaMessage.EnumPeripheralsAnswer.UserPerNr = 1;
          FlagUserPer( _DpaMessage.EnumPeripheralsAnswer.UserPer, PNUM_STD_SENSORS );
          _DpaMessage.EnumPeripheralsAnswer.HWPID = 0x123F;
          _DpaMessage.EnumPeripheralsAnswer.HWPIDver = 0xABCD;

          // Return the enumeration structure but do not modify _DpaDataLength
          returnDataLength = sizeof( _DpaMessage.EnumPeripheralsAnswer );
          // Return TRUE
          returnFlags = EVENT_RETURN_TRUE;
        }
        // Get information about peripherals?
        else if ( IsDpaPeripheralInfoRequest() )
        {
          if ( _PNUM == PNUM_STD_SENSORS )
          {
            _DpaMessage.PeripheralInfoAnswer.PerT = PERIPHERAL_TYPE_STD_SENSORS;
            _DpaMessage.PeripheralInfoAnswer.PerTE = PERIPHERAL_TYPE_EXTENDED_READ_WRITE;
            // Set standard version
            _DpaMessage.PeripheralInfoAnswer.Par1 = STD_SENSORS_VERSION;

            // Return the information structure but do not modify _DpaDataLength
            returnDataLength = sizeof( _DpaMessage.PeripheralInfoAnswer );
            // Return TRUE
            returnFlags = EVENT_RETURN_TRUE;
          }
        }
        else
        {
          // Handle peripheral command

          // Supported peripheral number?
          if ( _PNUM == PNUM_STD_SENSORS )
          {
            // Supported commands?
            switch ( _PCMD )
            {
              // Invalid command
              default:
                // Return error
                DpaApiReturnPeripheralError( ERROR_PCMD );
                break;

                // Sensor enumeration
              case PCMD_STD_ENUMERATE:
                // Check data request length
                if ( _DpaDataLength != 0 )
                {
                  DpaApiReturnPeripheralError( ERROR_DATA_LEN );
                  break;
                }

                // 1st byte is sensor type
                _DpaMessage.Response.PData[0] = STD_SENSOR_TYPE_HUMIDITY;
                _DpaMessage.Response.PData[1] = STD_SENSOR_TYPE_LOW_VOLTAGE;
                // _DpaMessage.Response.PData[2] = STD_SENSOR_TYPE_BINARYDATA7;

                // Return just one sensor type
                returnDataLength = _DpaDataLength = sizeof ( _DpaMessage.Response.PData[0] ) + sizeof ( _DpaMessage.Response.PData[1] ) ;
                // Return TRUE
                returnFlags = EVENT_RETURN_TRUE;
                break;

                // Supported commands. They are handled almost the same way
              case PCMD_STD_SENSORS_READ_VALUES:
              case PCMD_STD_SENSORS_READ_TYPES_AND_VALUES:
              {
                
                Serial.println ();
                Serial.print ("_DpaDataLength: ");
                Serial.println (_DpaDataLength);
                // No sensor bitmap specified?
                if ( _DpaDataLength == 0 )
                {
                  // Bitmap is 32 bits long = 4
                  _DpaDataLength = 4;
                  // Simulate 1st sensor in the bitmap (states of the other unimplemented sensors do not care)
                  _DpaMessage.Request.PData[0] = 1; // Note: must not modify W
                }
                // Valid bitmap length?
                else if ( _DpaDataLength != 4 && _DpaDataLength != 9 && _DpaDataLength != 14 )
                {
                  // Return error
                  DpaApiReturnPeripheralError( ERROR_DATA_LEN );
                  break;
                }
                
                // write request for sensor #1 (servo control)
                if ( ( _DpaDataLength == 9 || _DpaDataLength == 14 ) && _DpaMessage.Request.PData[4] == 1 )
                {
                  // sensor type == [0x80] Relative Humidity 
                  // hence we are interested only in the first data byte [6]
                  servoPos = _DpaMessage.Request.PData[5] ;
                  
                  // turn servo to the requested position
                  turnServo = true ;
                }

                // Pointer to the response data
                byte *pResponseData = _DpaMessage.Response.PData;
                // Is my only sensor selected?
                if ( _DpaMessage.Request.PData[0] == 1 )
                {
                  // Return also sensor type?
                  if ( _PCMD == PCMD_STD_SENSORS_READ_TYPES_AND_VALUES )
                    *pResponseData++ = STD_SENSOR_TYPE_HUMIDITY ;

                  if ( ( _DpaDataLength == 9 || _DpaDataLength == 14 ) && _DpaMessage.Request.PData[4] == 1 )
                    *pResponseData++ = servoPos ;
                  else
                  *pResponseData++ = servo.getAngle () ;
                }

                if ( _DpaMessage.Request.PData[0] == 2 )
                { 
                  if ( _PCMD == PCMD_STD_SENSORS_READ_TYPES_AND_VALUES )
                    *pResponseData++ = STD_SENSOR_TYPE_LOW_VOLTAGE;
                  // Return sensor data
                  battery.checkBatteryLevel ();
                  *pResponseData++ = battery.battLow;  // little endian => low goes first
                  *pResponseData++ = battery.battHigh;
                }

                if ( _DpaMessage.Request.PData[0] == 3 || _DpaMessage.Request.PData[0] == 255 )
                {
                  // Return also sensor type?
                  if ( _PCMD == PCMD_STD_SENSORS_READ_TYPES_AND_VALUES )
                    *pResponseData++ = STD_SENSOR_TYPE_HUMIDITY ;
                  
                  if ( ( _DpaDataLength == 9 || _DpaDataLength == 14 ) && _DpaMessage.Request.PData[4] == 1 )
                    *pResponseData++ = servoPos ;
                  else
                    *pResponseData++ = servo.getAngle () ;

                  if ( _PCMD == PCMD_STD_SENSORS_READ_TYPES_AND_VALUES )
                    *pResponseData++ = STD_SENSOR_TYPE_LOW_VOLTAGE;
                  // Return sensor data
                  battery.checkBatteryLevel ();
                  *pResponseData++ = battery.battLow;  // little endian => low goes first
                  *pResponseData++ = battery.battHigh;
                }

                // Returned data length
                returnDataLength = _DpaDataLength = ( pResponseData - _DpaMessage.Response.PData );
                // Return TRUE
                returnFlags = EVENT_RETURN_TRUE;
                break;
              }
            }
          }
        }

        // Return DPA response
        ResponseCommand( returnFlags, _DpaDataLength, returnDataLength, (byte*)&_DpaMessage );
        
        if ( turnServo ) servo.setAngle ( servoPos, delayTime ) ; // finally, turn the servo
      }
      break;

      // Return FRC Value
    case DpaEvent_FrcValue:
      // Check for the minimum length (FRC command and at least 2 bytes of data)
      if ( dataLength >= ( 2 + 1 + 2 ) )
      {
        // Fake important DPA variables for the DPA FRC handling
#define FrcCommand               (RxBuffer[1])
#define DataOutBeforeResponseFRC ((byte*)( &RxBuffer[2] ))

      // Check the correct FRC request
        if ( DataOutBeforeResponseFRC[0] == PNUM_STD_SENSORS &&
          ( DataOutBeforeResponseFRC[1] == 0x00 || DataOutBeforeResponseFRC[1] == STD_SENSOR_TYPE_LOW_VOLTAGE ) &&
             ( DataOutBeforeResponseFRC[2] & 0x1f ) == 0 )
        {
          // Which FRC command to handle?
          switch ( FrcCommand )
          {
            case FRC_STD_SENSORS_1B:
              ResponseFRCvalue( GetSensor0Value() + 4 );
              break;

            case FRC_STD_SENSORS_BIT:
              ResponseFRCvalue( ( GetSensor0Value() & ( 0x01 << ( DataOutBeforeResponseFRC[2] >> 5 ) ) ) != 0 ? 0x03 : 0x01 );
              break;
          }
        }
      }
      break;
  }
}

//############################################################################################
void loop()
//############################################################################################
{
  // Byte received from the IQRF?
  while ( Serial1.available() )
  {
    // HDLC machine states
    typedef enum { RXstateWaitHead, RXstatePacket, RXstateEscape } TState;

    // HDLC state
    static byte state = RXstateWaitHead;
    // Length of the already received data
    static byte rxLength;
    // Pointer to the received data
    static byte *pRxBuffer;

    // Read the byte from IQRF
    byte oneByte = Serial1.read();
    Serial.print(oneByte);
    switch ( state )
    {
      // Waiting for the HDLC header
      case RXstateWaitHead:
      {
        if ( oneByte == HDLC_FRM_FLAG_SEQUENCE )
        {
_SetRXstatePacket:
          rxLength = 0;
          pRxBuffer = RxBuffer;
          state = RXstatePacket;
        }
        break;
      }

      // Handling packet data byte
      case RXstatePacket:
      {
        switch ( oneByte )
        {
          case HDLC_FRM_CONTROL_ESCAPE:
            // RXstateEscape
            state++;
            goto _ExitMachine;

          case HDLC_FRM_FLAG_SEQUENCE:
          {
            if ( rxLength >= MIN_RX_PACKET_DATA_LENGTH )
              // Packet received, handle it
              CustomDpaHandler( rxLength );

            goto _SetRXstatePacket;
          }
        }

_StoreByte:
        if ( rxLength == ( MAX_RX_PACKET_DATA_LENGTH + 2 ) )
          goto _SetRXstateWaitHead;

        *pRxBuffer++ = oneByte;
        rxLength++;

_ExitMachine:
        break;
      }

      // Handle escaped byte
      case RXstateEscape:
      {
        switch ( oneByte )
        {
          case HDLC_FRM_FLAG_SEQUENCE:
            goto _SetRXstatePacket;

          case HDLC_FRM_CONTROL_ESCAPE:
_SetRXstateWaitHead:
            state = RXstateWaitHead;
            break;

          default:
            // RXstatePacket
            state--;
            oneByte ^= HDLC_FRM_ESCAPE_BIT;
            goto _StoreByte;
        }
        break;
      }
    }
  }
  
}

//############################################################################################
