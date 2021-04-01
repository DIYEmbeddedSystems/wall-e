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
#include <ESP8266mDNS.h>        /* multicast DNS responder */
#include <ESPAsyncTCP.h>        /* Asynchronous web and websocket servers  */
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>

#include <LittleFS.h>           /* LittleFS flash filesystem (replaces deprecated SPIFFS)*/
#include <ArduinoJson.h>        /* Parse JSON commands */


/* ~~~~~  Project-local dependencies */
#include <credentials.h>        /* Wifi access point credentials, IP configuration macros */
#include "trigger.h"            /* Utility for periodic tasks */

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
#include "pca9685_servo.h"

/* Motors */
#include "MotorShield.h"
/* Note: motor shield should be programmed, see: https://hackaday.io/project/18439-motor-shield-reprogramming
 */

/* GY-80 IMU (Inertial measurement unit) */
#include "GY-80.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Function declarations / prototypes
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void setup();
void loop();
void heartBeat();
void actuatorsLoop();
void reportState();
void safetyCheck();
void updateAhrs();
uint32_t i2c_check();

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Global variables & object instances
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Configure a logger that uses both Serial and UDP channels */
/* start your server with: nc -ulk 8888 |tee -i log_file.txt */
WiFiUDP Udp;
UdpLogger loggerUdp(Udp, IPAddress(192,168,1,105), 8888, "UDP", LOG_ALL);
DupLogger logger(SerialLogger::getDefault(), loggerUdp); // log to both Serial and UDP

/* I2C devices addresses 
  Wemos Motor shield: 0x2F
  SSD1306 OLED shield: 0x3C
  PCA9685 PWM Servo shield: 0x40
  GY-80 IMU: 
  * HMC5883 magnetometer: 0x1E,
  * ADXL345 3-axis accelerometer: 0x53
  * L3G4200D - 3-axis Gyro: 0x69
  * BMP085 Barometer & temperature: 0x77 
*/
static const uint8_t i2c_addresses[] = {0x2F, 0x3C, 0x40, 0x1E, 0x53, 0x69, 0x77};
static const char *i2c_device_names[] = {"motor", "oled", "pwm", "mag", "accel", "gyro", "baro"};

/* I2C OLED screen */
Adafruit_SSD1306 oledDisplay(0);

/* Servos: an array of servos for position-commanded joints (head, arms) */
//    "HdUp", "HdLR",         /* Head: up/down, left/right */
//    "LAUp", "LALR", "LH",   /* Left arm: up/down, left/right, hand open/close */
//    "RAUp", "RALR", "RH"    /* Right arm: up/down, left/right, hand open/close */
SlowServo servos[] = 
{
  SlowServo( 8, 180.0),  /* head up/down */
  SlowServo( 9, 180.0),  /* head left/right */
  SlowServo(10, 120.0),  /* left arm up/down */
  SlowServo(11, 120.0),  /* left arm left/right */
  SlowServo(12, 300.0),  /* left hand open/close */
  SlowServo(13, 120.0),  /* right arm up/down */
  SlowServo(14, 120.0),  /* right arm left/right */
  SlowServo(15, 300.0)   /* right hand open/close */
};
#define NUM_SERVOS ((int)(sizeof(servos) / sizeof(servos[0])))

/* Motors: an array of motors for velocity-commanded joints (tank treads DC motors) */
Motor motors[] = 
{
  Motor(0x30, Motor::motor_a),
  Motor(0x30, Motor::motor_b)
};
#define NUM_MOTORS ((int)(sizeof(motors)/sizeof(motors[0])))

/* Remember last 'move' command */
int32_t move_command[2] = {0};

/* Remember attitude & orientaion (yaw/pitch/roll) */
float attitude[3] = {0.0f, 0.0f, 0.0f};

/* last message from remote controller page */
char userMessage[128] = "(user message)";

/* Last time we have received a proper command */
uint32_t lastCommandMs = 0;

/* Am I being updated right now? (i.e. stop other communications) */
bool updating = false;

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
  SlowServo::outputDisable();

  Serial.begin(115200);
  while (!Serial) continue;
//  gdbstub_init();

  // let power rails stabilize, and let PlatformIO monitor time to start
  delay(1000); 

  // This log line appears only on Serial link because WiFi is not up yet
  logger.info("\n\n\n");
  logger.info("Application " __FILE__ " compiled " BUILD_DATE);

  /* Start up OLED display screen */
  Wire.begin();
  Wire.setClock(400 * 1000); // according to implementation, supports 1 kHz to 400 kHz clock frequency

  oledDisplay.setRotation(2);
  oledDisplay.clearDisplay();
  oledDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //oledDisplay.drawBitmap(0, 0, walle_splash, walle_splash_width, walle_splash_height, 1);
  oledDisplay.drawBitmap(0, 0, splash_solar_charge, splash_solar_charge_width, splash_solar_charge_height, 1);
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

  logger.info("\n\n");
  logger.info("Application " __FILE__ " compiled " BUILD_DATE);
  logger.info(BUILD_DETAILS);
  logger.info("Reset reason: %s", ESP.getResetReason().c_str());
  logger.info("Reset info: %s", ESP.getResetInfo().c_str());
  logger.info("Wifi is up. I'm %s", WiFi.localIP().toString().c_str());

  /* Start up multicast DNS */
  if (!MDNS.begin("wall-e"))
  { 
    logger.warn("Could not start MDNS");
  }
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
  // FIXIT: I could not test MDNS to actually respond to wall-e.local

  /* Start up file system */
  LittleFS.begin();
  logger.info("File system is up. Files: %s", filesJSON().c_str());

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
    servos[i].moveTo(0);
  }
  logger.info("Servos are up");

  /* Setup motors */
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].begin();
    motors[i].setSpeed(10);
    delay(200);
    motors[i].setSpeed(0);
  }

  i2c_check();

  logger.info("\n\nSetup done!\n\n");
}

/**
 * @brief Main loop
 */
void loop()
{
  if (updating)
  {
    ledBlink(150, 50);

    static uint32_t nextMs = 0;
    if (periodicTrigger(&nextMs, 1000))
    {
      Serial.printf("Web update ongoing...");
    }
  }
  else
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
    safetyCheck();
    uint32_t t5 = micros();
    updateAhrs();
    if ((t5 - t0) > 10000) 
    {
      logger.info("Loop %lu µs: heartbeat %lu, WS %lu, act %lu, report %lu, safetyCheck %lu", 
        t4 - t0, t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4);
    }
  }
}

/**
 * @brief Periodically output liveness info
 */
void heartBeat()
{
  static uint32_t nextMs = 0;
  const uint32_t periodMs = 1000;

  if (wsServer.count() > 0)
  {
    ledBlink(10, 490);
  }
  else
  {
    ledBlink(10, 990);
  }
  
  if (periodicTrigger(&nextMs, periodMs))
  {  
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

  if (periodicTrigger(&nextMs, periodMs))
  {
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

  if (periodicTrigger(&nextMs, periodMs))
  {
    // Build up JSON representation of current state
    snprintf(msg, sizeof(msg), "{\"ms\":%lu,\"servos\":[%d,%d,%d,%d,%d,%d,%d,%d],\"move\":[%d,%d],\"attitude\":[%3.2f,%3.2f,%3.2f]}",
        millis(), 
        servos[0].getPos(), servos[1].getPos(), servos[2].getPos(), servos[3].getPos(), 
        servos[4].getPos(), servos[5].getPos(), servos[6].getPos(), servos[7].getPos(),
        move_command[0], move_command[1], 
        attitude[0], attitude[1], attitude[2]);
    
    // Send to all connected clients
    wsServer.textAll(msg);
//    logger.info(msg);
  }
}

/**
 * @brief Check system safety; switch to failsafe mode if necessary
 */
void safetyCheck()
{
  static uint32_t nextMs = 7;
  const uint32_t periodMs = 500;
  static bool isSafe = false;

  if (periodicTrigger(&nextMs, periodMs))
  {
    if (isSafe)
    {
      // System was safe. Is it still?
      if ((int32_t)(millis() - lastCommandMs - 1000) >= 0)
      {
        // No communication for 1 second --> switch to failsafe
        isSafe = false;

        motors[0].setSpeed(0);
        motors[1].setSpeed(0);

        SlowServo::outputDisable();
        for (int i = 0; i < NUM_SERVOS; i++)
        {
          servos[i].stop();
        }
        logger.warn("No active communication: stopped");
      }
    }
    else
    {
      // System was in failsafe. Can we switch back to safe?
      if ((int32_t)(millis() - lastCommandMs - 1000) < 0)
      {
        // A command was received during last second: OK.
        isSafe = true;
        SlowServo::outputEnable();
      }
    }
  }
}


void updateAhrs()
{
  static uint32_t checkSensorMs = 0;
  const uint32_t checkSensorPeriodMs = 15000;

  static uint32_t updateNextMs = 0;
  const uint32_t updatePeriodMs = 100;

  static uint32_t outputNextMs = 0;
  const uint32_t outputPeriodMs = 1000;

  static bool imuReady = false;
  static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  if (periodicTrigger(&checkSensorMs, checkSensorPeriodMs))
  {
    if (!imuReady)
    {
      imuReady = initGY80();
    }
  }

  if (periodicTrigger(&updateNextMs, updatePeriodMs) && imuReady)
  {
    updateGY80();
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my,  mz, q);
  }

  if (periodicTrigger(&outputNextMs, outputPeriodMs) && imuReady)
  {
    float roll, pitch, yaw;
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;

    attitude[0] = yaw;
    attitude[1] = pitch;
    attitude[2] = roll;

    char msg[256];
    snprintf(msg, sizeof(msg), 
        "{\"ms\":%ld,\"yaw\":%3.2f,\"pitch\":%3.2f,\"roll\":%3.2f}",
        millis(), yaw, pitch, roll);
    logger.info(msg);
    wsServer.textAll(msg);
  }
}

/**
 * @brief This handler is called whenever a valid JSON is received from client
 */
void webSocketJsonFrameHandler(AsyncWebSocket * server, AsyncWebSocketClient * client, StaticJsonDocument<JSON_MEMORY_SIZE> &jsonDoc)
{
  uint32_t nowMs = millis();

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
      pos = constrain(pos, -90, 90);
      servos[i].moveTo(pos);
    }
    lastCommandMs = nowMs;
  }

  JsonArray motorArray = jsonDoc["move"].as<JsonArray>();
  if (motorArray.size() == 2)
  {
    move_command[0] = constrain(motorArray[0].as<int>(), -100, 100); // forward / backward
    move_command[1] = constrain(motorArray[1].as<int>(), -100, 100); // left / right
    float speedFactor = constrain(2. * fabs(move_command[0]) / 100., 0, 1);
    int leftSpeed = constrain(speedFactor * (move_command[0] + move_command[1]), -100, 100);
    int rightSpeed = constrain(speedFactor * (move_command[0] - move_command[1]), -100, 100);
    motors[0].setSpeed(leftSpeed);
    motors[1].setSpeed(-rightSpeed);
    lastCommandMs = nowMs;
  }
}

/**
 * @brief Peek at all i2c devices that should be connected to the wemos
 * @return 0 if all devices acknowledged, i-th bit set if i-th device fails
 */
uint32_t i2c_check()
{
  uint32_t res = 0;

  for (uint32_t i = 0; i < sizeof(i2c_addresses)/sizeof(i2c_addresses[0]); i++)
  {
    // peek I2C address (do not send data), device should just ACK
    Wire.beginTransmission(i2c_addresses[i]);
    uint8_t error = Wire.endTransmission();

    if (error != 0)
    {
      res |= (1 << i);
    }
    logger.info("I2C devices %s at %02x: %s", 
        i2c_device_names[i], i2c_addresses[i], error ? "FAIL" : "OK");
  }
  return res;
}
