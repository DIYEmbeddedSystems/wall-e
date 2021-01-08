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

extern DupLogger logger;

AsyncWebServer httpServer(80);

/**
 * @brief Configure a web server
 */
void setupWebServer()
{
  /* Start up file system */
  LittleFS.begin();
  logger.info("File system is up");

  /* Configure HTTP server handlers */
  httpServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  httpServer.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "application/json", filesJSON());
  });  
 
  httpServer.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "application/json", versionJSON());
  });

  httpServer.onNotFound([](AsyncWebServerRequest *request) {
      logger.info("http: %s %s%s not found", 
          (request->method() == HTTP_GET) ? "GET" : (request->method() == HTTP_POST) ? "POST" : "METHOD?",
          request->host().c_str(),
          request->url().c_str());
      request->send(404, "text/plain", "404: Not Found");
  });

  httpServer.begin();
}


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

String getContentType(String filename) 
{
  if (filename.endsWith(".htm")) return "text/html";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/x-pdf";
  else if (filename.endsWith(".zip")) return "application/x-zip";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}