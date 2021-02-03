/**
 * @file webServer.cpp
 * @brief Implementation of the Web server module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 */

#include "webServer.h"

#include <Arduino.h>
#include <ESP8266WiFi.h>        /* WiFi functions */
#include <ESPAsyncTCP.h>        /* Asynchronous web and websocket servers  */
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>           /* LittleFS flash filesystem (replaces deprecated SPIFFS)*/

#include <DupLogger.h>          /* Logging library */

#include "version.h"

extern DupLogger logger;        /* logger to be used (defined in main) */
extern bool updating;


AsyncWebServer httpServer(80);  /* HTTP server instance */

void ota_update_progress(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

/**
 * @brief Configure a web server
 */
void webServerSetup()
{
  /* Configure HTTP server handlers */
  /* Serve any file stored in flash filesystem */
  httpServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  httpServer.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "application/json", filesJSON());
    });  
 
  httpServer.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "application/json", versionJSON());
    });

  httpServer.on("/fw_ota", HTTP_POST, [](AsyncWebServerRequest *request){
      logger.warn("Firmware OTA update started !");
      updating = true;
      request->send(200);
    }, ota_update_progress);

  httpServer.onNotFound([](AsyncWebServerRequest *request) {
      logger.info("http: %s %s%s not found", 
          (request->method() == HTTP_GET) ? "GET" : (request->method() == HTTP_POST) ? "POST" : "METHOD?",
          request->host().c_str(),
          request->url().c_str());
      request->send(404, "text/plain", "404: Not Found");
    });

  httpServer.begin();
}


/**
 * @brief Callback for firmware upload request
 */
void ota_update_progress(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if (!index)
  {
    logger.info("Update ! Free space %u B", free_space);
    Update.runAsync(true);
    if (!Update.begin(free_space)) 
    {
      Update.printError(Serial);
      logger.error("Update: not enough space");
      updating = false;
    }
  }

  if (Update.write(data, len) != len) 
  {
    Update.printError(Serial);
    logger.error("Update: write error");
    updating = false;
  }
  else
  {
    logger.info("Progress: %d%%\n", (Update.progress()*100)/Update.size());
    updating = true;
  }

  if (final) 
  {
    if (!Update.end(true))
    {
      Update.printError(Serial);
      logger.error("Update: end failed");
      updating = false;
    } 
    else
     {
      Serial.println("Update complete");
      logger.error("Restarting. Bye !");
      updating = false;
      delay(1000);
      ESP.restart();
    }
  }
}


/**
 * @brief Describe filesystem in JSON format
 */
String filesJSON()
{
  String s = "";
  FSInfo fs_info;
  LittleFS.info(fs_info);
  s += "{\"totalSize\":\"";
  s += getSizeFormat(fs_info.totalBytes);
  s += "\", \"usedSize\":\"";
  s += getSizeFormat(fs_info.usedBytes);
  s += "\", \"files\":[";

  Dir dir = LittleFS.openDir("/");
  int numFiles = 0;
  while (dir.next()) 
  {
    if (numFiles > 0) 
    { 
      s += ",";
    }
    s += "{\"name\":\"";
    s += dir.fileName();
    s += "\", \"size\":\"";
    s += getSizeFormat(dir.fileSize());
    s += "\"}";
    numFiles++;
  }
  s += "]}";
  return s;
}

/**
 * @brief Describe build version in JSON format
 */
String versionJSON()
{
  String s = "";
  s += "{\"build-date\":\"" BUILD_DATE "\"";
  s += ",\"build-nb\":\"" + String(BUILD_NUMBER) + "\"";
  s += ",\"hash\":\"" GIT_DESCRIPTION "\"";
  s += ",\"branch\":\"" GIT_BRANCH "\"";
  s += "}";
  return s;
}

const char *getSizeFormat(int size) 
{
  static char buffer[128];
  if (size > 1024*1024) 
  {
    snprintf(buffer, sizeof(buffer)-1, "%.1fMB", size*1.0/1024/1024);
  }
  else if (size > 1024) 
  {
    snprintf(buffer, sizeof(buffer)-1, "%.1fkB", size*1.0/1024);
  } 
  else 
  {
    snprintf(buffer, sizeof(buffer)-1, "%uB", size);
  }
  return buffer;
}
