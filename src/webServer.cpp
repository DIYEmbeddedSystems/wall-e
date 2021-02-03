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

void fw_upload_progress(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void file_upload_progress(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);


/**
 * @brief Configure a web server
 */
void webServerSetup()
{
  /* Configure HTTP server handlers */

  /* Serve files stored in LittleFS flash filesystem */
  httpServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  /* Serve JSON apis */
  httpServer.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      request->send(200, "application/json", filesJSON());
    });  
 
  httpServer.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      request->send(200, "application/json", versionJSON());
    });

  httpServer.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      char msg[256];
      snprintf(msg, sizeof(msg), "{\"heap\":%u}", ESP.getFreeHeap());
      request->send(200, "application/json", msg);
    });


  httpServer.on("/erase", HTTP_ANY, [](AsyncWebServerRequest *request)
    {
       if(request->hasParam("path"))
       {
        String path = request->getParam("path")->value();
        if (LittleFS.exists(path))
        {
          logger.warn("Removing file `%s`", path.c_str());
          LittleFS.remove(path);
          request->send(200, "text/plain", "Removed");
        }
        else
        {
          logger.warn("File `%s` not found", path.c_str());
          request->send(200, "text/plain", "Not found");
        }
       }
       else
       {
         logger.warn("File erase: path not provided");
         request->send(200, "text/plain", "Usage: erase?path=/example.txt");
       }
    });

  /* Serve firmware update endpoint */
  httpServer.on("/fw_ota", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      if (!Update.hasError())
      {
        logger.warn("Firmware OTA update successful");
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
        response->addHeader("Connection", "close");
        request->send(response);
        delay(5000);
        ESP.restart();
      }
      else
      {
        logger.warn("Firmware OTA update successful");
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "FW update failed");
        request->send(response);
      }
      
      updating = true;
      request->send(200, "text/plain", "update finished");
    }, fw_upload_progress);

  /* Serve file update endpoint */
  httpServer.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final)
    {
      if(!index) 
      {
        logger.warn("UploadStart: %s", filename.c_str());
        request->_tempFile = LittleFS.open(filename, "w");
      }

      if (len && request->_tempFile)
      {
        request->_tempFile.write(data,len);
      }

      if(final)
      {
        if (request->_tempFile)
        {
          request->_tempFile.close();
        }
        logger.warn("UploadEnd: %s (%s)\n", filename.c_str(), getSizeFormat(index+len));
        request->send(200, "text/plain", "Upload complete");
      }
    });

  httpServer.onNotFound([](AsyncWebServerRequest *request) 
    {
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
void fw_upload_progress(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  static uint32_t reportedProgress = 0;
  uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

  if (!index)
  {
    logger.info("FW upload ! Filename %s, Free space %u B", filename.c_str(), free_space);
    Update.runAsync(true);
    if (!Update.begin(free_space)) 
    {
      Update.printError(Serial);
      logger.error("FW upload: not enough space");
      updating = false;
    }
    reportedProgress = 0;
  }

  if (Update.write(data, len) != len) 
  {
    Update.printError(Serial);
    logger.error("FW upload write error");
    updating = false;
  }
  else
  {
    uint32_t progress = (Update.progress()*100)/Update.size();
    if (progress - reportedProgress >= 5)
    {
      logger.info("FW upload %d%%", progress);
      reportedProgress = progress;
    }
    updating = true;
  }

  if (final) 
  {
    if (!Update.end(true))
    {
      int err = Update.getError();
      Update.printError(Serial);
      logger.error("FW upload failed");
      logger.error("Error: %d, %s",
        err,
        (err == UPDATE_ERROR_WRITE) ? "write" :
        (err == UPDATE_ERROR_ERASE) ? "erase" : 
        (err == UPDATE_ERROR_SPACE) ? "space" :
        (err == UPDATE_ERROR_SIZE) ? "size" :
        (err == UPDATE_ERROR_MD5) ? "md5" : 
        (err == UPDATE_ERROR_SIGN) ? "sign" :
        (err == UPDATE_ERROR_FLASH_CONFIG) ? "flash_config" :
        (err == UPDATE_ERROR_MAGIC_BYTE) ? "magic_byte" :
        (err == UPDATE_ERROR_BOOTSTRAP) ? "bootstrap" :
        "other"
      );
    }
    updating = false;
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
