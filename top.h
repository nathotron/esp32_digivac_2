#ifndef top_h
#include "Arduino.h" // so, this does not quite make sense, but I found I have to include this to get access to the
// usually-always-available Serial object from "tabs" meaning e.g. ADT7311.cpp
#include <SPI.h>
#include <Wire.h> // for LCD & button control panel, also heater boards
#include <stdlib.h> // for dtostrf
#include <stdint.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/dedic_gpio.h"
// #include "logic_analyzer_ws_server.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include <mutex>

// we only have one use_SPI so use a MUTEX to guarantee exclusive access to it
extern portMUX_TYPE gSPI_MUX;
#define SPI_ENTER_CRITICAL()      // portENTER_CRITICAL(&gSPI_MUX)
#define SPI_EXIT_CRITICAL()      // portEXIT_CRITICAL(&gSPI_MUX)

// same for the Wire bus
extern portMUX_TYPE gWire_MUX;
#define WIRE_ENTER_CRITICAL()      //portENTER_CRITICAL(&gWire_MUX) ******************** disabled for debug
#define WIRE_EXIT_CRITICAL()      //portEXIT_CRITICAL(&gWire_MUX)

// inter-task communication, handles both serial input commands and e.g. encoder wheel
extern QueueHandle_t gSerialReceiveQueue;

#define BIT(x)  (1<<x)

#define kTFT_backlight_pin  0 // must be output only
#define kTFT_CS0_pin  1
#define kTFT_DC_pin 2
#define kTFT_reset0_pin  3 // each TFT needs its own reset line

#define kDIGIVAC1_RX_pin  15
#define kSCL_pin  16
#define kDIGIVAC0_TX_pin  17
#define kSDA_pin  18

#define kRGB_LED_pin  21 // this is hard wired on the ESP32-S3-ETH board and it is unclear why there is
// no standard definition

// oddly there was a problem with 
// #define kDIGIVAC0_RX_pin  33
// it seemed to be an output & certainly was not reading the RX0 signal
// had to move it to 36

#define kTFT_reset1_pin  34

#define kDIGIVAC0_RX_pin  36

// *** note pin 37 possibly cannot be used as an output

#define kTURBO_START_IN_pin   38 // the controller drives this HIGH during startup, then it goes LOW when full speed is achieved
#define kTURBO_FAULT_pin      39  // if this is HIGH it means a fault condition, either controller is off (!) or the front panel
// will show a fault such as over temperature
#define kTURBO_START_OUT_pin  40 // we drive this high to start the turbo

#define kDIGIVAC1_TX_pin  43
#define kTFT_CS1_pin  44

#define kTFT_MOSI_pin 45 //
#define kTFT_CLK_pin  46 // default 12 // note 46 is a "strapping" pin which means it is sampled on boot to determine the boot mode. If they are left floating on
// boot a weak pull up or pull down gives them the desired state. It means, don't let any external device drive this during startup.
// #define kSPI_M95P32_CS_pin 2 // Initially I thought 45 was not working, but actually it does.
#define kTFT_MISO_pin 47 // default 13

// ESP32-S3 has 4 hardware SPIs. SPI0 and SPI1 are used for private ESP stuff like memory. SPI2 and SPI3 are available for our use.
// https://forum.arduino.cc/t/using-vspi-with-esp32/621775
// Mentions that when Arduino sees plain "SPI" it understands VSPI... maybe meaning SPI3?
// In this application I used plain SPI for the W5500 ethernet because I don't know how to tell the ethernet layer to use anything else.
// therefore we use HSPI for e.g. the M95P32
extern SPIClass use_SPI;

void printHEX(uint8_t hex);

// log by producing output to usual Serial terminal but also to active telnet connection
// so that in principle we see same thing on each. The D is for "dual"
void DSerialPrintln();
void DSerialPrintln(char *s);
void DSerialPrint(char *s);
void DSerialPrint(char c);
void DSerialPrint(String s);
void DSerialPrint(float f, int dec);
void DSerialPrint(int32_t i);
void DSerialPrint(uint32_t i);
void DSerialPrint(int i);

// #define Wire1_lock() Wire1.end(); digitalWrite(SDA1, HIGH); pinMode(SDA1, OUTPUT); digitalWrite(SCL1, HIGH); pinMode(SCL1, OUTPUT)
#define top_h
#endif // wrapper condition so only included once