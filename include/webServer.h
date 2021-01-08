/**
 * @file webServer.h
 * @brief Interface of the Web server module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
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