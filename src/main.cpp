/**
 * @file main.cpp
 * @brief Main source file for Wall-E firmware
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 */


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Includes
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "version.h"            /* Versioning macros, generated in the build_version.py script */
#include <Arduino.h>            /* The Arduino framework */
/* ~~~~~ Generic libraries for ESP8266: */
// #include <GDBStub.h>            /* Enable debugging */
#include <Esp.h>                /* Get infos about ESP reset, stack/heap, etc. */
#include <ESP8266WiFi.h>        /* WiFi functions */
#include <ESPAsyncTCP.h>        /* Asynchronous web and websocket servers  */
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>

#include <LittleFS.h>           /* LittleFS flash filesystem (replaces deprecated SPIFFS)*/
#include <ArduinoJson.h>        /* Parse JSON commands */


/* ~~~~~  Project-local dependencies */
#include <credentials.h>        /* Wifi access point credentials, IP configuration macros */

#include <ArduinoLogger.h>      /* Logging library */
#include <SerialLogger.h>       /* Log to Serial */
#include <UDPLogger.h>          /* Send logs to a UDP server */
#include <DupLogger.h>          /* Send logs to multiple channels */

#include "webServer.h"          /* web server and file system */
#include "webSocketServer.h"    /* websocket server */

/* Actuators */
#include "led.h"                /* built-in LED */

/* I2C OLED screen */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include "images.h"

/* Servos */
#include <pca9685_servo.h>


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Function declarations / prototypes
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void setup();
void loop();
void heartBeat();
void actuatorsLoop();
void reportState();

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Global variables & object instances
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Configure a logger that uses both Serial and UDP channels */
/* start your server with: nc -ulk 8888 |tee -i log_file.txt */
WiFiUDP Udp;
UdpLogger loggerUdp(Udp, IPAddress(192,168,1,105), 8888, "UDP", LOG_ALL);
DupLogger logger(SerialLogger::getDefault(), loggerUdp); // log to both Serial and UDP

/* I2C OLED screen */
Adafruit_SSD1306 oledDisplay(0);

/* Servos: limited to 60° per second speed */
SlowServo servos[] = 
{
  SlowServo(12, 60.0),
  SlowServo(13, 60.0),
  SlowServo(14, 60.0),
  SlowServo(15, 60.0)
};
#define NUM_SERVOS ((int)(sizeof(servos) / sizeof(servos[0])))


/* last message from remote controller page */
char userMessage[128] = "(user message)";

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Function implementation
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ~~~~~ Platform initialization ~~~~~ */

/**
 * @brief Set up software runtime
 */
void setup()
{
  ledOff();
  Serial.begin(115200);
  while (!Serial) continue;
//  gdbstub_init();

  // let power rails stabilize, and let PlatformIO monitor time to start
  delay(1000); 

  logger.info("\n\n\n");
  logger.info("Application " __FILE__ " compiled " BUILD_DATE);


  /* Start up OLED display screen */
  Wire.begin();
  Wire.setClock(400 * 1000); // according to implementation, supports 1 kHz to 400 kHz clock frequency
  oledDisplay.setRotation(2);
  oledDisplay.clearDisplay();
  oledDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oledDisplay.drawBitmap(0, 0, walle_splash, walle_splash_width, walle_splash_height, 1);
  oledDisplay.display();
  logger.info("Display is up");

  /* Setup WiFi */
  WiFi.mode(WIFI_STA);
  WiFi.hostname("Wall-E");
  WiFi.config(
      IPAddress(192,168,1,182),     /* my IP address */
      IPAddress(255,255,255,0),     /* subnet */
      IPAddress(192,168,1,178),    /* gateway */
      IPAddress(8,8,8,8));         /* DNS */

  logger.info("Connecting to " WIFI_STASSID);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STASSID, WIFI_STAPSK);
  while (WiFi.status() != WL_CONNECTED) 
  {
    ledBlink(200, 50);
    delay(10);
  }

  logger.info(BUILD_DETAILS);
  logger.info("Reset reason: %s", ESP.getResetReason().c_str());
  logger.info("Reset info: %s", ESP.getResetInfo().c_str());

  logger.info("Wifi is up. I'm %s", WiFi.localIP().toString().c_str());

  /* Start up file system */
  LittleFS.begin();
  logger.info("File system is up:");
  logger.info(filesJSON().c_str());

  /* Start up webserver */
  webServerSetup();
  logger.info("Web server is up");

  /* Start up Websocket server */
  webSocketServerSetup();

  logger.info("WebSocket server is up");

  /* Setup servos */
  for (int i = 0; i < NUM_SERVOS; i++)
  {
    servos[i].begin();
    servos[i].blockingMoveTo(5);
    servos[i].blockingMoveTo(0);
  }
  logger.info("Servos are up");

  logger.info("\n\nSetup done!\n\n");
}

/**
 * @brief Main loop
 */
void loop()
{
  uint32_t t0 = micros();

  heartBeat();

  uint32_t t1 = micros();

  webSocketServerLoop();

  uint32_t t2 = micros();

  actuatorsLoop();

  uint32_t t3 = micros();

  reportState();

  uint32_t t4 = micros();

  if ((t4 - t0) > 10000) 
  {
    logger.info("Loop %lu µs: heartbeat %lu, WS %lu, act %lu, report %lu", 
      t4 - t0, t1 - t0, t2 - t1, t3 - t2, t4 - t3);
  }
}

/**
 * @brief Periodically output liveness info
 */
void heartBeat()
{
  static uint32_t nextMs = 0;
  const uint32_t periodMs = 1000;

  ledBlink(10, 990);

  if ((int32_t)(millis() - nextMs) >= 0)
  {
    while ((int32_t)(millis() - nextMs) >= 0)
    {
      nextMs += periodMs;
    }
  
    char msg[256];
    snprintf(msg, sizeof(msg), "At %3u: %u clients, %ukB free. %s                                          ",
        (unsigned int)millis()/1000, wsServer.count(), (unsigned int)(ESP.getFreeHeap()/1024), userMessage);

    logger.info(msg);

    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(1);
    oledDisplay.setTextColor(WHITE);
    oledDisplay.setCursor(0,0);
    oledDisplay.print(msg);
    oledDisplay.display();
  }
}

/**
 * @brief Update servo position
 */
void actuatorsLoop()
{
  static uint32_t nextMs = 5;
  const uint32_t periodMs = 10;

  if (1) //(int32_t)(millis() - nextMs) >= 0)
  {
    while ((int32_t)(millis() - nextMs) >= 0)
    {
      nextMs += periodMs;
    }

    for (int i = 0; i < NUM_SERVOS; i++)
    {
      servos[i].update();
    }
  }
}

/**
 * @brief Periodically send current actuators state to connected controller(s)
 */
void reportState()
{
  static uint32_t nextMs = 8;
  const uint32_t periodMs = 200;

  char msg[256];

  if ((int32_t)(millis() - nextMs) >= 0)
  {
    while ((int32_t)(millis() - nextMs) >= 0)
    {
      nextMs += periodMs;
    }

    // Build up JSON representation of current state
    snprintf(msg, sizeof(msg), "{\"ms\":%lu,\"servos\":[%d,%d,%d,%d]}",
        millis(), 
        servos[0].getPos(), servos[1].getPos(), servos[2].getPos(), servos[3].getPos());
    
    // Send to all connected clients
    wsServer.textAll(msg);
//    logger.info(msg);
  }
}

/**
 * @brief This handler is called whenever a valid JSON is received from client
 */
void webSocketJsonFrameHandler(AsyncWebSocket * server, AsyncWebSocketClient * client, StaticJsonDocument<JSON_MEMORY_SIZE> &jsonDoc)
{
  const char *message = jsonDoc["message"];
  if (message) 
  {
    // copy message from JSON payload to global variable 
    snprintf(userMessage, sizeof(userMessage), message);
    logger.info("[WS] <-- message: %s", userMessage);
  }

  JsonArray servoArray = jsonDoc["servos"].as<JsonArray>();
  if (servoArray.size() == NUM_SERVOS)
  {
    for (int i = 0; i < NUM_SERVOS; ++i)
    {
      int pos = servoArray[i].as<int>();
//      logger.info("[WS] <-- servo[%d].moveTo(%d)", i, pos);
      pos = constrain(pos, -90, 90);
      servos[i].moveTo(pos);
    }
  }
}