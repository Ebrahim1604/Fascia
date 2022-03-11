/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_FASCIA_V1_
#define _VARIANT_FASCIA_V1_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC   (32768ul)

/** Master clock frequency */
#define VARIANT_MCK       (120000000ul)

#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (41u)
#define NUM_DIGITAL_PINS     (38u)
#define NUM_ANALOG_INPUTS    (6u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)


/*
 * Analog pins
 */
#define PIN_A0               (19ul)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_A4               (PIN_A0 + 4)
#define PIN_A5               (PIN_A0 + 5)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;

#define ADC_RESOLUTION    12

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_RX       SERCOM_RX_PAD_0
#define PAD_SERIAL1_TX       UART_TX_PAD_1

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 3

//SPi
#define PIN_SPI_MISO         (5u)
#define PIN_SPI_MOSI         (6u)
#define PIN_SPI_SCK          (4u)
#define PERIPH_SPI           sercom0
#define PAD_SPI_TX           SPI_PAD_3_SCK_1 //mosi pad
#define PAD_SPI_RX           SERCOM_RX_PAD_2 //miso pad

static const uint8_t SS   = 3; // SERCOM4 last PAD is present on d9 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;  

//SPi1
#define PIN_SPI1_MISO         (10u)
#define PIN_SPI1_MOSI         (8u)
#define PIN_SPI1_SCK          (9u)
#define PERIPH_SPI1           sercom1
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3

static const uint8_t SS1   = 7; // SERCOM4 last PAD is present on d9 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ; 

//SPi2
#define PIN_SPI2_MISO         (14u)
#define PIN_SPI2_MOSI         (13u)
#define PIN_SPI2_SCK          (16u)
#define PERIPH_SPI2           sercom4
#define PAD_SPI2_TX           SPI_PAD_0_SCK_1
#define PAD_SPI2_RX           SERCOM_RX_PAD_3

static const uint8_t SS2   = 15; // SERCOM4 last PAD is present on d9 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI2 = PIN_SPI2_MOSI ;
static const uint8_t MISO2 = PIN_SPI2_MISO ;
static const uint8_t SCK2  = PIN_SPI2_SCK ; 

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (34u)
#define PIN_WIRE_SCL         (35u)
#define PERIPH_WIRE          sercom3
#define WIRE_IT_HANDLER      SERCOM3_Handler
#define WIRE_IT_HANDLER_0    SERCOM3_0_Handler
#define WIRE_IT_HANDLER_1    SERCOM3_1_Handler
#define WIRE_IT_HANDLER_2    SERCOM3_2_Handler
#define WIRE_IT_HANDLER_3    SERCOM3_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (38ul)
#define PIN_USB_DM          (36ul)
#define PIN_USB_DP          (37ul)


//QSPI Pins
#define PIN_QSPI_SCK    (25u)
#define PIN_QSPI_CS     (26u)
#define PIN_QSPI_IO0    (27u)
#define PIN_QSPI_IO1    (28u)
#define PIN_QSPI_IO2    (29u)
#define PIN_QSPI_IO3    (30u)

//TODO: meaningful value for this
#define VARIANT_QSPI_BAUD_DEFAULT 5000000

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*  =========================
 *  ===== SERCOM DEFINITION
 *  =========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_FASCIA_V1_ */

