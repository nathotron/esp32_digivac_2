// code for embedded system on vacuum tank which reads the DPP Vacuum sensor & displays pressure
// version 2 with ESP32S3-POE module & dual SPI TFT displays

// Use Arduino setting "ESP32S3 Dev Module"

// Dr Nathan Scott 20241003

// Code for mySerial1 from https://forum.arduino.cc/t/esp32-serial-ports/1203861/3
// SPI TFT LCD 128x160 ST7735 https://www.amazon.com/dp/B0DFWLFN35?ref=cm_sw_r_cp_ud_dp_PH7GW4VP3ZN9TWN9CS2C&ref_=cm_sw_r_cp_ud_dp_PH7GW4VP3ZN9TWN9CS2C&social_share=cm_sw_r_cp_ud_dp_PH7GW4VP3ZN9TWN9CS2C

// ESP32-S3-ETH Power over ethernet version
// schematic at https://files.waveshare.com/wiki/ESP32-S3-ETH/ESP32-S3-ETH-Schematic.pdf
// choose Arduino compile setting ESP32S3 Dev Module
// check that the Tools/settings match image at https://www.waveshare.com/wiki/ESP32-S3-ETH, watch out because Arduino enviroment resets them
// when the board library files are updated
// the main ones are
// USB CDC on boot: ENABLED (-> serial monitor works!)
// Flash size 16MB (128 Mb)
// Partition Scheme: "Huge App (3MB No OTA/1MB SPIFFS)"
// PSRAM: "OPI PSRAM"

#include "top.h"
#include <WS2812B.h> // RGB LED on ESP32-S3-ETH board, connected to GPIO21

// honor externs in top.h
portMUX_TYPE gSPI_MUX = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE gWire_MUX = portMUX_INITIALIZER_UNLOCKED;

//#include <SPI.h>
//#include <Wire.h>
#include <floatToString.h>  // https://github.com/tedtoal/floatToString

// 20260410 mySerial2 seems to work fine with custom pin assignment
// mySerial1 however seemed to TX correctly but not RX. So far as I can tell the assigned RX pin is not set to input mode.
// In frustration, try a different library
// Note these are just for communication to the DigiVac sensors.
#define kUseHardwareSerial 1
#if kUseHardwareSerial
HardwareSerial hSerial1(1);
HardwareSerial hSerial2(2);
#define mySerial1 hSerial1
#define mySerial2 hSerial2
#else // use Arduino provided Serial objects
#define mySerial1 Serial1
#define mySerial2 Serial2
#endif

#define kHasLCD 0
#if kHasLCD // WaveShare LCD1602I2C with AIP31068 controller
// https://github.com/HendrikVE/Arduino-LiquidCrystalWired
#include "LiquidCrystalWired.h"
#define LCD_ADDRESS (0x7c >> 1)
#define ROW_COUNT   2
#define COL_COUNT   16
LiquidCrystalWired lcd = LiquidCrystalWired(ROW_COUNT, COL_COUNT, FONT_SIZE_5x8, BITMODE_8_BIT);
#endif

// extern in top.h: queue handle for inter-task communication
QueueHandle_t gSerialReceiveQueue;

// honor extern in header
SPIClass use_SPI(HSPI);

#define kHasTFT 1
#if kHasTFT
// https://github.com/adafruit/Adafruit-GFX-Library/blob/master/examples/GFXcanvas/GFXcanvas.ino
// https://learn.adafruit.com/adafruit-gfx-graphics-library?view=all
#include "GFXcanvasSerialDemo.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#define kBigFont FreeSerifBold24pt7b
#include "FreeSerifBold24pt7b.h"
#define kSmallFont FreeSerifBold12pt7b
#include "FreeSerifBold12pt7b.h"

// TWO RGB display panels
Adafruit_ST7735 tft0 = Adafruit_ST7735(&use_SPI, kTFT_CS0_pin, kTFT_DC_pin, kTFT_reset0_pin);
#define kHasTFT1  1
#if kHasTFT1
Adafruit_ST7735 tft1 = Adafruit_ST7735(&use_SPI, kTFT_CS1_pin, kTFT_DC_pin, kTFT_reset1_pin);
#endif

#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF


volatile SemaphoreHandle_t tftSemaphore;
#define UpdateTFT()   xSemaphoreGive(tftSemaphore)
#define kTFTSPISettings SPISettings(200000, MSBFIRST, SPI_MODE0)
#define SPI_begin_TFT() // SPI.beginTransaction(kTFTSPISettings) // seems to be unnecessary

#endif // kHasTFT

#define has_W5500 1
#if has_W5500
#include <Ethernet.h>

// Define the W5500 Ethernet module pins with the new GPIO assignments

// https://github.com/adafruit/circuitpython/issues/9884
// GP9 : RST
// GP10 : INT
// GP11 : MOSI
// GP12 : MISO
// GP13 : SCLK
// GP14: SCSn

#define W5500_CS_PIN  14
#define W5500_RST_PIN 9
#define W5500_INT_PIN 10
#define W5500_CLK_PIN 13
#define W5500_MOSI_PIN 11
#define W5500_MISO_PIN 12

// MAC address for your Ethernet shield (must be unique on your network)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };

// Static IP address configuration
// 169.254.186.245 autoconfig address
IPAddress ip(192, 168, 192, 34);       // Static IP address
IPAddress subnet(255, 255, 255, 0);   // Subnet mask
IPAddress gateway(192, 168, 3, 1);    // Default gateway
IPAddress dns(192, 168, 3, 1);        // DNS server address

// Create an EthernetServer object to handle TCP connections
uint16_t gPort = 5535;
EthernetServer *gServer = NULL; // once set up, this will be non-null
// EthernetClient* gClient; // when a telnet client is connected, this will be non-null
#endif // has_W5500

//Variables for Serial Read Input
#define kStringBufLen 40                                          //for reserving bytes for input
String gInputString = "";                                         //a String to hold incoming data
boolean gStringComplete = false;                                  //whether the string is complete

//String gDPPString0 = "";
String gDPPString1 = "";
//String gDisp0 = "Not detected"; // this is all so clunky at the moment
String gDisp1 = "Not detected";
//String gUnits0 = "";
String gUnits1 = "";

// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TimerHandle_t xSoftwareTimer;

uint32_t  gTicks;
#define kTickFrequency  25 // Hz, may be a good idea if 1000 can be evenly divided by this number

// Vacuum sensor response is something like @253ACK7.6207E+02[terminator]
#define kDPPTerminator '\\'
// values parsed from vacuum sensor response
//uint8_t gDPPAddress0, gDPPAddress1;
float gDPPPressure1; // gDPPPressure0, 
uint8_t gWaitingForPressureData;

uint8_t gVerbose = 0; // set to non-zero to get a pressure reading every second on the terminal

// Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, kRGB_LED_pin, NEO_GRB + NEO_KHZ800);
WS2812B gWS2812B_LED;

#if kHasTFT
void show_TFT_time()
{
  // String tstr = get_date_time_str(0); // use gEpoch which is now updating nicely

  // tstr.remove(0, 4); // strip off "[202" so more of the string fits on LCD

  String tstr = "";
  tstr += millis();

  SPI_ENTER_CRITICAL();
  SPI_begin_TFT();
  tft0.setCursor(10,ST7735_TFTHEIGHT_160 - 10);
  tft0.setTextSize(1); // "setTextSize(size) will multiply the scale of the text by a given integer factor"
  tft0.print(tstr);

#if kHasTFT1
  tft1.setCursor(10,ST7735_TFTHEIGHT_160 - 10);
  tft1.setTextSize(2); // "setTextSize(size) will multiply the scale of the text by a given integer factor"
  tft1.print(tstr);
#endif

  SPI_EXIT_CRITICAL();
}

String gPrevDisp1, gPrevUnits1; // hack to do text overwrite
void show_TFT_pressure()
{
  // String tstr = gDisp0 + " " + gUnits0;

  SPI_ENTER_CRITICAL();
  SPI_begin_TFT();
  //tft0.setCursor(10,ST7735_TFTHEIGHT_160 - 40);
  //tft0.setTextSize(1); // "setTextSize(size) will multiply the scale of the text by a given integer factor"
  //tft0.print(tstr);

#if kHasTFT1
  // tstr = gDisp1 + " " + gUnits1;
  //tft1.setTextColor(BLACK);
  //tft1.setCursor(0, 50); // ST7735_TFTHEIGHT_160
  //tft1.setTextSize(1); // "setTextSize(size) will multiply the scale of the text by a given integer factor"
  //tft1.print(gPrevDisp1); // erase previous pressure by overwriting with black
  tft1.setTextColor(WHITE);
  tft1.setCursor(0, 50);
  tft1.print(gDisp1);
  gPrevDisp1 = gDisp1;

  //tft1.setTextColor(BLACK);
  //tft1.setCursor(0,ST7735_TFTWIDTH_128 - 20);
  //tft1.setTextSize(1); // "setTextSize(size) will multiply the scale of the text by a given integer factor"
  //tft1.print(gPrevUnits1); // erase previous units by overwriting with black
  tft1.setTextColor(WHITE);
  tft1.setCursor(0,ST7735_TFTWIDTH_128 - 20);
  tft1.print(gUnits1);
  gPrevUnits1 = gUnits1;

#endif

  SPI_EXIT_CRITICAL();
}

void updateTFTTask(void *pvParameters) // because something may have changed
{
  String tstr, tempStr, humStr;
  SPI_ENTER_CRITICAL();
  SPI_begin_TFT();
  tft0.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  // seems to leave noise field i.e. not black?
  // tft.init(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160, 0, 0, ST7796S_RGB);

  tft0.setRotation(-45); // bizarrely we have to input the angle in degrees but divided by 2! Positive clockwise
  tft0.fillScreen(0); // Clear screen

#if kHasTFT1
  tft1.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft1.setRotation(-45);
  tft1.fillScreen(0); // Clear screen
#endif

  // https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST77xx.h
  tft0.setFont(&kSmallFont);
  // tft0.setCursor(10,ST7735_TFTHEIGHT_160 - 10); // top left is (0, 0)
  tft0.setCursor(0,30); // top left is (0, 0)
  tft0.setTextColor(WHITE);
  // tft.setTextSize(48); // keep as-installed size since it will probably look smoothest
  tft0.setTextWrap(true);
  tft0.print("DigiVac DPP\r\n& turbo intf.");

    // show IP address
  tft0.setTextColor(WHITE);
  //tft0.setRotation(0);
  tft0.setFont(&kSmallFont);
  tft0.setCursor(0,ST7735_TFTWIDTH_128 - 5);
  String ipString;
  for (int i = 0; i < 4; i++)
  {
    ipString += String(ip[i]);
    if (i < 3) ipString += ".";
  }
  tft0.print(ipString);

#if kHasTFT1
  tft1.setFont(&kSmallFont);
  tft1.setCursor(0, 30); // top left is (0, 0)
  tft1.setTextColor(WHITE);
  // tft.setTextSize(48); // keep as-installed size since it will probably look smoothest
  tft1.setTextWrap(true);
  tft1.print("STScI Makidon Lab\r\n20260410\r\nDr Nathan Scott");
#endif

  SPI_EXIT_CRITICAL();

  vTaskDelay(2000); // delay so intro screen is visible for a short time

  SPI_ENTER_CRITICAL();
  SPI_begin_TFT();
  tft0.setFont(&kBigFont);
  tft0.setTextWrap(false);
  tft0.fillScreen(0); // fill screen with black
  #if kHasTFT1
    tft1.setFont(&kBigFont);
    tft1.setTextWrap(false);
    tft1.fillScreen(0); // fill screen with black
  #endif
  SPI_EXIT_CRITICAL();

  while (1)
  {
    SPI_ENTER_CRITICAL();
    SPI_begin_TFT();
    tft0.fillScreen(0); // fill screen with black

    tft0.setFont(&kBigFont);
    //tft0.setRotation(45);
    // show state of turbo controller on tft0
    // First line: are we requesting turbo start or not?
    tft0.setTextColor(MAGENTA);
    tft0.setCursor(0, ST7735_TFTWIDTH_128 / 3 - 5);
    if (digitalRead(kTURBO_START_OUT_pin))
      tft0.print("ON");
    else
      tft0.print("OFF");

    // second line, show state of the START pin on J2
    tft0.setTextColor(BLUE);
    tft0.setCursor(0, (2*ST7735_TFTWIDTH_128)/3 - 5);
    if (digitalRead(kTURBO_START_OUT_pin)) // we are trying to start the turbo
    {
      if (digitalRead(kTURBO_START_IN_pin)) // high while running up to speed
        tft0.print("Starting");
      else // low again when full operating speed is attained
        tft0.print("MaxRPM"); // if the controller is disconnected, we will show this which is kind of confusing
    }
    else
      tft0.print("OFF"); // if START_OUT is low, the state of START_IN is kind of meaningless so mask it

    // 3rd line: fault condition
    tft0.setCursor(0,ST7735_TFTWIDTH_128 - 5);
    if (digitalRead(kTURBO_FAULT_pin))
    {
      tft0.setTextColor(RED);
      tft0.print("FAULT");
    }
    else
    {
      tft0.setTextColor(GREEN);
      tft0.print("OK");
    }

  #if kHasTFT1
    tft1.fillScreen(0); // fill screen with black
  #endif
    SPI_EXIT_CRITICAL();

    // show_TFT_time();
    show_TFT_pressure();
    //#endif // has_RTC

    xSemaphoreTake(tftSemaphore, portMAX_DELAY); // block indefinitely until some task calls UpdateLCD()
  } // while(1)
} // UpdateTFT()
#endif // kHasTFT


static void ARDUINO_ISR_ATTR my_tick( TimerHandle_t xTimer )
{
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  gTicks++;
  // lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

#define kSampleTicks  kTickFrequency
void tick_task(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(timerSemaphore, portMAX_DELAY); // block indefinitely until the timer ISR fires
    // so frustrated with crazy behavior of gTicks which seems to count very fast, thousands of times per second, even though
    // it should be incremented only 25 times per second in my_tick
    // vTaskDelay(configTICK_RATE_HZ/kTickFrequency); // this is crude & will cause "time slip" because it does not
    // account for time spent actually doing work in this thread

    uint32_t isrCount = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    
    // gTicks++;
    isrCount = gTicks;
    portEXIT_CRITICAL(&timerMux);

    //if (isrCount > gBacklightOffTicks)
    //{
      //lcd.noBacklight();
      //gBacklightOffTicks = isrCount; // so we don't constantly "turn off" already off backlight
    //}

#ifdef RGB_BUILTIN
  if (isrCount % kTickFrequency == 0)
    rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // red
  else if (isrCount % kTickFrequency == 1)
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);  // Off / black
#endif

  if (isrCount % kTickFrequency == 0)
    gWS2812B_LED.set("red", 255);
  else 
    gWS2812B_LED.set("red", 0);


    // having a strange problem where isrCount % kSampleTicks seems to always evaluate to zero
    uint32_t modulo = isrCount;
    modulo %= kSampleTicks;
    //DSerialPrintln();
    //DSerialPrint(isrCount); DSerialPrint("\t"); DSerialPrint(modulo);

    if (modulo == 0)
    {
#if kHasTFT
      PressureToText(gDPPPressure1);
      UpdateTFT();
#endif
      if (gVerbose)
      {
        DPrintStatus(); // this will show the LAST pressure but can only be one second out of date
      }
      
        // send a message to the DPP sensor
      // mySerial1.print("@254P?"); // note this serial port cannot receive data, unclear why not
      // mySerial1.print(kDPPTerminator);

      mySerial2.print("@254P?");
      mySerial2.print(kDPPTerminator);
      gWaitingForPressureData = 1; // so we can display an error if it does not arrive
    }
  } // while 1
}

void setup_timer()
{
  DSerialPrintln();
  DSerialPrint("timer setup: FreeRTOS configTICK_RATE_HZ = "); DSerialPrint(configTICK_RATE_HZ); Serial.flush();

    // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // create a task to handle events from the timer ISR
  xTaskCreatePinnedToCore(tick_task, "tick_task", 4096, NULL, 2, NULL, 1);
  // xTaskCreate(TaskWriteToSerial, "WriteSerial", 256, NULL, 1, NULL);

  xSoftwareTimer = xTimerCreate( /* A text name, purely to help debugging. */
                                          ( const char * ) "tick_timer",
                                          /* The timer period, in this case
                                             1000ms (1s). */
                                          configTICK_RATE_HZ/kTickFrequency,
                                          /* This is a periodic timer, so
                                             xAutoReload is set to pdTRUE. */
                                          pdTRUE,
                                          /* The ID is not used, so can be set
                                             to anything. */
                                          ( void * ) 0,
                                          /* The callback function  */
                                          my_tick
                                        );

    /* Start the created timer. A block time of zero is used as the timer
       command queue cannot possibly be full here (this is the first timer to
       be created, and it is not yet running). */
    xTimerStart( xSoftwareTimer, 0 );

}

#if has_W5500

String gReceivedMessage = "";

void ethernetClientTask(void *pvParameters)
{
  EthernetClient* client = (EthernetClient*) pvParameters;

  // DSerialPrintln("New client connected");


  // To get started we duplicate the way serial terminal input is processed
  // Read data from the client and echo it back
  while (client->connected())
  {
    if (client->available())
    {
      char c = client->read();
      if (c == '\n')
      { // Check for newline character (Enter key)
        // Send the complete message to the queue for other tasks to process
        DSerialPrintln();
        DSerialPrint("> ");
        // for (int i = 0; i < gReceivedMessage.length(); i++)
        // {
          // putHEX(gReceivedMessage.charAt(i)); Serial.print(' ');
          // Mystery here. Not every character's HEX value printed to the Arduino terminal, but the string itself seems OK
        // }
        DSerialPrint(gReceivedMessage); DSerialPrint(" "); // separate command from response on same line
        if (xQueueSend(gSerialReceiveQueue, &gReceivedMessage, portMAX_DELAY) != pdPASS)
        {
          DSerialPrintln("Failed to send message to queue.");
        }
        gReceivedMessage = ""; // Clear the message buffer for the next input
      }
      else if (c >= 32) // everything below 32 is "non printing"
      {
        gReceivedMessage += c; // Append the character to the message string
      }

      // server.write(c);

    }
    vTaskDelay(1); // to prevent watchdog reset
  }

  // Close the connection when done
  client->stop();
  // DSerialPrintln("Client disconnected");

  vTaskDelete(NULL); // delete this task
}

// listens for connecting clients, then starts a client thread for each
void ethernetServerTask(void *pvParameters)
{
  // Initialize the W5500 module
  pinMode(W5500_RST_PIN, OUTPUT);
  pinMode(W5500_INT_PIN, INPUT);
  digitalWrite(W5500_RST_PIN, LOW);  // Reset the W5500 module
  delay(100);                       // Wait for reset to complete
  digitalWrite(W5500_RST_PIN, HIGH); // Release reset

  // Initialize SPI with the correct pin definitions
  SPI.begin(W5500_CLK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN); // ************************* note plain SPI is dedicated to the W5500 & Core 0

  // Set up the Ethernet library with W5500-specific pins
  Ethernet.init(W5500_CS_PIN);

  // Start the Ethernet connection with static IP configuration
  Ethernet.begin(mac, ip, dns, gateway, subnet);

  // Print the IP address to the serial monitor
  DSerialPrint("IP Address: ");
  DSerialPrint(Ethernet.localIP()); Serial.flush();

  // It seems we can't change the port after instantiating the object
  // so at this moment we require restart of application to get a new
  // port number
  EthernetServer server(gPort);

  DSerialPrint(" port: "); DSerialPrint(gPort); Serial.flush();

  // Start listening for incoming TCP connections
  server.begin();

  while (1)
  {
    // Check for incoming client connections
    EthernetClient client = server.available(); // seems to not block, so we are polling, ugh
    // apparently "the connection persists when the returned client object goes out of scope"
    if (client)
    {
        // up till now gServer may have been NULL
        // this is messy *************************************************************
        gServer = (EthernetServer*) &server; // global ptr

        xTaskCreatePinnedToCore(
          ethernetClientTask,     // Task function
          "Ethernet Client",       // Name of the task
          4096,                   // Stack size (bytes)
          &client,                   // Parameter to pass to the task
          1,                      // Task priority (higher is more urgent)
          NULL,                   // Task handle (not used here)
          0                       // Core to run on (0 or 1) // ********************* note 0 here!
        );
    }
    vTaskDelay(10); // FreeRTOS needs this to prevent a "watchdog time reset"
  } // while(1)
} // ethernetServerTask

void setup_ethernet_server()
{

    xTaskCreatePinnedToCore(
    ethernetServerTask,     // Task function
    "Ethernet Server",       // Name of the task
    4096,                   // Stack size (bytes)
    NULL,                   // Parameter to pass to the task
    1,                      // Task priority (higher is more urgent)
    NULL,                   // Task handle (not used here)
    0                       // Core to run on (0 or 1) // ********************* note 0 here!
    // else is on Core1
  );
}
#endif // has_W5500

void setup()
{
  gWS2812B_LED.begin(kRGB_LED_pin);
  gWS2812B_LED.set("white", 255);

  pinMode(kTURBO_START_OUT_pin, OUTPUT); // we drive this high to start the turbo
  digitalWrite(kTURBO_START_OUT_pin, LOW); // off initially

  pinMode(kTURBO_START_IN_pin, INPUT); // _PULLDOWN the controller drives this low during startup, then it goes high when full speed is achieved
  pinMode(kTURBO_FAULT_pin, INPUT); // _PULLDOWN if this is LOW it means a fault condition, either controller is off (!) or the front panel
// will show a fault such as over temperature

  // Serial.begin(115200);
  setup_uart();

  // Two ports to communicate with Digivac0 and Digivac1 sensors
  mySerial1.begin(9600, SERIAL_8N1, kDIGIVAC0_RX_pin, kDIGIVAC0_TX_pin); // RX does not work. Does not seem to matter which RX pin I choose.
  mySerial2.begin(9600, SERIAL_8N1, kDIGIVAC1_RX_pin, kDIGIVAC1_TX_pin);

  // Initialise SPI.
  // SPI.begin(kSPI_CLK_pin, kSPI_MISO_pin, kSPI_MOSI_pin, kSPI_CS_pin);  // CLK , MISO, MOSI, CS
  // SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));

  // Initialise I2C. Note we can choose which pins!
  // https://community.platformio.org/t/how-to-assign-i2c-pins-on-esp32-s3-on-platformio/38965/3
  Wire.begin(kSDA_pin, kSCL_pin); // use default frequency

#if kHasLCD
  lcd.begin(LCD_ADDRESS, &Wire);
  lcd.turnOn();
  // lcd.setCustomSymbol(CUSTOM_SYMBOL_1, customCharHeart);
  lcd.setCursorPosition(0,0);
  lcd.print("STScI Makidon Lb");
  lcd.setCursorPosition(1, 0);
  lcd.print("NWS 20241003");
#endif

#if kHasTFT // **** note this is not compatible with other uses of SPI at the moment
  // SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI); // note it has no MISO output
  SPI_begin_TFT();
  use_SPI.begin(kTFT_CLK_pin, kTFT_MISO_pin, kTFT_MOSI_pin);  // CLK , MISO, MOSI, CS // , kSPI_CS_pin
  
  pinMode(kTFT_backlight_pin, OUTPUT); digitalWrite(kTFT_backlight_pin, 1);

  tftSemaphore = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(
    updateTFTTask,     // Task function
    "Update TFT",       // Name of the task
    4096,                   // Stack size (bytes)
    NULL,                   // Parameter to pass to the task
    1,                      // Task priority (higher is more urgent)
    NULL,                   // Task handle (not used here)
    1                       // Core to run on (0 or 1)
  );
  
#endif

  DSerialPrintln();
#if has_W5500
  setup_ethernet_server();
#else
  DSerialPrint("no W5500"); Serial.flush();
#endif // has_W5500

  setup_timer();

  DSerialPrintln("ESP32 ready");
}

void printActiveState()
{

}

#define kDefaultDelta 1000
void ObeyCommand(String cmd)
{
   if (cmd.length() == 0)
   {
      DPrintStatus();
      return;
   } // end empty command

  char first = cmd.charAt(0);                         //reads only the first character of cmd
  cmd.remove(0,1);                                        //deletes the first character of cmd
  cmd.trim();
  long s = -1;
  if (cmd.length() > 0) s = cmd.toInt(); // s will be 0 if the format is not as expected
  
  DSerialPrintln();

  int i;
  long scratch;
  switch (first)
  {
    case 't': // turbo on or off
      if (s)
      {
        digitalWrite(kTURBO_START_OUT_pin, HIGH);
        DSerialPrint("turbo ON");
#if kHasTFT
        UpdateTFT();
#endif
      }
      else
      {
        digitalWrite(kTURBO_START_OUT_pin, LOW);
        DSerialPrint("turbo OFF");
#if kHasTFT
        UpdateTFT();
#endif 
      }
    return;

    case 'v': // verbose mode on or off
    // in verbose mode the vacuum pressure is displayed once per second
      if (s)
      {
        gVerbose = 1;
        DSerialPrint("verbose on");
      }
      else
      {
        gVerbose = 0;
        DSerialPrint("verbose off");       
      }
    return;
  } // switch on first character

  // if we reach here the command was not understood
  DSerialPrint("?"); DSerialPrintln();
}//end ObeyCommand Function

// handler for characters that come from the vacuum sensor. We send it something like @254P?[terminator]
// and the response is e.g. @253ACK7.6207E+02[terminator]
uint8_t ProcessDPPString(String s)
// returns 1 when address & pressure values have been parsed
{
  // Serial.println(s);

  String sub = s.substring(1); // skip the @
  //if (which)
    //gDPPAddress1 = sub.toInt();
  //else
    //gDPPAddress0 = sub.toInt();

  //Serial.println();
  //Serial.print(gDPPAddress);
  //Serial.print('\t');

  sub = s.substring(7); // skip everything up to start of pressure
  //if (which)
    gDPPPressure1 = sub.toFloat();
  //else
    //gDPPPressure0 = sub.toFloat();
  //Serial.print(gDPPPressure);
  return 1;
}

#define kScratchStrLen  30
void DPrintStatus()
{
  char scratch[kScratchStrLen];
  floatToString(gDPPPressure1, scratch, kScratchStrLen, 6);
  // crudely format to produce two columns, more or less. Note either or both sensors might be absent.
  DSerialPrint("\r\n");
  DSerialPrint(millis()); // show "up time"
  DSerialPrint('\t');
  if (gWaitingForPressureData)
    DSerialPrint("N/A"); // sensor disconnected maybe?
  else
    DSerialPrint(scratch); // no unit conversion here, just raw number, for data logging

  // show turbo state
  if (digitalRead(kTURBO_START_IN_pin)) // high while running up to speed
    DSerialPrint("\tStarting");
  else // low again when full operating speed is attained
    DSerialPrint("\tMaxRPM"); // if the controller is disconnected, we will show this which is kind of confusing

  if (digitalRead(kTURBO_FAULT_pin))
    DSerialPrint("\tFAULT");
  else
    DSerialPrint("\tOK");
}

// process pressure measurement to produce reader-friendly text gDisp1, gUnits1
void PressureToText(float p)
{
  // https://github.com/tedtoal/floatToString
  // char* floatToString(float f, char* S, size_t n, int digitsAfterDP);
  char scratch[kScratchStrLen];

  String disp = "";
  String units = "";
  if (gWaitingForPressureData)
  {
    disp = "N/A";
  }
  else if (p < 1)
  {
    floatToString(p * 1000, scratch, kScratchStrLen, 3);
    disp = scratch;
    units = "mTorr";
  }
  else  // pressure in Torr range
  {
    floatToString(p, scratch, kScratchStrLen, 3);
    disp = scratch;
    units = "Torr";
  }

  //if (which)
  //{
    gDisp1 = disp;
    gUnits1 = units; 
  //}
  //else
  //{
    //gDisp0 = disp;
    //gUnits0 = units; 
  //}

// we will update once per second on a timed heartbeat so no need to also update here
//#if kHasLCD  
  //lcd.clear();
  //lcd.print(disp); lcd.print(" "); lcd.print(units);
//#endif

//#if kHasTFT
  //UpdateTFT();
//#endif
}

void loop()
{

  /*
   // behind the scenes an interrupt gathers up serial events to build the gInputString.
  // when the user pressed RETURN the flag gStringComplete is set.
  if (gStringComplete)
  {
    Serial.print("\t"); // separate command from response
    ObeyCommand(gInputString);
    // clear the string:
    gInputString = "";
    gStringComplete = false;
    Serial.println();
    // Serial.print("\r\n> "); // show command prompt
  } // end if gStringComplete */

  // process characters that come from the DPP sensors
  char c;
/*  if (mySerial1.available()) // note we don't use while() here because if both mySerial1 and mySerial2 are receiving,
  // we want to service both with about equal priority, or characters might be lost
  {
      // represent as hex so we can see what is going on
      // Serial.write(' ');
      c = mySerial1.read();
      // Serial.write(c); // pass through for now
      if (c == kDPPTerminator)
      {
        if (ProcessDPPString(gDPPString0, 0)) // note we did not append the terminator
        {
          PressureToText(gDPPPressure0, 0);
        }
        gDPPString0 = ""; // reset ready for next
      }
      else
        gDPPString0 += c;
  } */

  if (mySerial2.available())
  {
      // represent as hex so we can see what is going on
      // Serial.write(' ');
      c = mySerial2.read();
      // Serial.write(c); // pass through for now
      if (c == kDPPTerminator)
      {
        if (ProcessDPPString(gDPPString1)) // note we did not append the terminator
        {
          // although we could update displays here, I think I prefer to just update once per second, driven by
          // tick count

          // PressureToText(gDPPPressure1, 1);
//#if kHasTFT
          //UpdateTFT();
//#endif
          gWaitingForPressureData = 0;
        }
        gDPPString1 = ""; // reset ready for next
      }
      else
        gDPPString1 += c;
  }

}

// utility to represent one byte as two ASCII HEX digits e.g 0F
void putHEX(char c)
{
  char high = c >> 4;
  if (high < 10)
    Serial.write('0' + high);
  else
    Serial.write('A' + high - 10);
  char low = c & 0x0F;
    if (low < 10)
    Serial.write('0' + low);
  else
    Serial.write('A' + low - 10);
}


// call these instead of direct e.g. Serial.print
void DSerialPrintln()
{
  Serial.println();
  if (gServer) gServer->write("\r\n", 2); // note this sends to all connected clients
}

void DSerialPrintln(char *s)
{
  Serial.println(s);
  if (gServer) gServer->write((const char*)s, strlen(s));
  if (gServer) gServer->write("\r\n", 2);
}

void DSerialPrint(char *s)
{
  Serial.print(s);
  if (gServer) gServer->write((const char*)s, strlen(s));
}

void DSerialPrint(char c)
{
  Serial.print(c);
  if (gServer) gServer->write(c);
}

void DSerialPrint(String s)
{
  char *str = (char*) s.c_str();
  // DSerialPrint(str);
  Serial.print(s);
  if (gServer) gServer->write(str, strlen(str));
}

void DSerialPrint(float f, int dec)
{
  String s = String(f, dec);
  char *str = (char*) s.c_str();
  // DSerialPrint(str);
  Serial.print(f, dec);
  if (gServer) gServer->write(str, strlen(str));
}

void DSerialPrint(int32_t i)
{
  String s = String(i);
  char *str = (char*) s.c_str();
  // DSerialPrint(str);
  Serial.print(i);
  if (gServer) gServer->write(str, strlen(str));
}

void DSerialPrint(uint32_t i)
{
  String s = String(i);
  char *str = (char*) s.c_str();
  // DSerialPrint(str);
  Serial.print(i);
  if (gServer) gServer->write(str, strlen(str));
}

void DSerialPrint(int i)
{
  String s = String(i);
  char *str = (char*) s.c_str();
  // DSerialPrint(str);
  Serial.print(i);
  if (gServer) gServer->write(str, strlen(str));
}


// code proposed by AI response to
// https://www.google.com/search?q=esp32s3+arduino+serial+example+freertos+receiver+example&sca_esv=4dcf95639e6c46b2&rlz=1C1CHBD_enUS912US912&sxsrf=AE3TifM165TW38bA6zjim4ZHX67dSAl3sA%3A1761078784641&ei=AO73aPPwJoqqw8cP4peyqQ4&oq=ESP32S3+Arduino+serial+example+freertos+receive&gs_lp=Egxnd3Mtd2l6LXNlcnAiL0VTUDMyUzMgQXJkdWlubyBzZXJpYWwgZXhhbXBsZSBmcmVlcnRvcyByZWNlaXZlKgIIATIFECEYoAEyBRAhGKABMgUQIRigAUjSI1CrBlisD3ABeACQAQCYAe4BoAGvCqoBBTAuNy4yuAEByAEA-AEBmAIJoAKGCsICChAAGLADGNYEGEfCAgUQIRirApgDAIgGAZAGCJIHBTEuNi4yoAfHIbIHBTAuNi4yuAf8CcIHBTAuMS44yAcm&sclient=gws-wiz-serp

void setup_uart()
{
  Serial.begin(115200);

    // Create the queue for serial data
  gSerialReceiveQueue = xQueueCreate(10, sizeof(String)); // Queue can hold 10 String objects

  if (gSerialReceiveQueue == NULL) {
    DSerialPrintln("Failed to create serial receive queue.");
    while (1); // Halt if queue creation fails
  }

  // Create the serial receiver task
  xTaskCreatePinnedToCore(
    serialReceiverTask,     // Task function
    "SerialReceiver",       // Name of the task
    4096,                   // Stack size (bytes)
    NULL,                   // Parameter to pass to the task
    1,                      // Task priority (higher is more urgent)
    NULL,                   // Task handle (not used here)
    1                       // Core to run on (0 or 1)
  );

  // Create the message processor task
  xTaskCreatePinnedToCore(
    messageProcessorTask,   // Task function
    "MessageProcessor",     // Name of the task
    4096,                   // Stack size (bytes)
    NULL,                   // Parameter to pass to the task
    1,                      // Task priority
    NULL,                   // Task handle
    1                       // Core to run on
  );
}

// Task to receive data from the serial port
// now also used to handle button presses and encoder wheel clicks from the button panel
void serialReceiverTask(void *pvParameters)
{
  char incomingChar;
  String receivedMessage = "";

  for (;;) {
    // Check if data is available in the Serial buffer
    while (Serial.available())
    {
      incomingChar = Serial.read(); // Read each character from the buffer

      if (incomingChar == '\n')
      { // Check for newline character (Enter key)
        // Send the complete message to the queue for other tasks to process
        if (xQueueSend(gSerialReceiveQueue, &receivedMessage, portMAX_DELAY) != pdPASS)
        {
          DSerialPrintln("Failed to send message to queue.");
        }
        receivedMessage = ""; // Clear the message buffer for the next input
      }
      else if (incomingChar >= 32) // everything below 32 is "non printing"
      {
        receivedMessage += incomingChar; // Append the character to the message string
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield to other tasks
    // There is something here I do not get. If Serial.read() blocks, why would we need a "yield delay" here?
    // Yet if I take out the delay I get watchdog timer timeout
  }
}

// Task to process received serial messages
void messageProcessorTask(void *pvParameters)
{
  String messageToProcess;
  for (;;) {
    // Wait for a message to be available in the queue
    if (xQueueReceive(gSerialReceiveQueue, &messageToProcess, portMAX_DELAY) == pdPASS) {
      //DSerialPrint("Received message: ");
      //DSerialPrintln(messageToProcess);

      ObeyCommand(messageToProcess);
    }
  }
}