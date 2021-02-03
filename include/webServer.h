/**
 * @file webServer.h
 * @brief Interface of the Web server module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 * 
 * - Upload a new firmware image:
 *     curl -v -F "image=@firmware.bin" ${IP}/update
 *  - Upload all files in /data/ folder:
*      for f in ./data/ * ; do curl -F "file=@$f" 192.168.1.167/upload ; done
 * - List files on file system:
 *    curl ${IP_ADDRESS}/list
 */

#ifndef WEBSERVER_H__
#define WEBSERVER_H__

#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

extern AsyncWebServer httpServer;

void webServerSetup();
String filesJSON();
String versionJSON();
const char *getSizeFormat(int size);

#endif // WEBSERVER_H