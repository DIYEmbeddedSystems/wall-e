/**
 * @file webSocketServer.h
 * @brief Interface of the WebSocket server module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 */
#ifndef WEBSOCKETSERVER_H_
#define WEBSOCKETSERVER_H_

#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <AsyncWebSocket.h>

#include <ArduinoJson.h>

extern AsyncWebSocket wsServer;

void webSocketServerSetup();
void webSocketServerLoop();

void websocketEventHandler(AsyncWebSocket * server, AsyncWebSocketClient * client, 
    AwsEventType eventType, void * arg, uint8_t *payload, size_t len);


void webSocketClientConnectHandler(AsyncWebSocket * server, AsyncWebSocketClient * client);
void webSocketClientDisconnectHandler(AsyncWebSocket * server, AsyncWebSocketClient * client);
void webSocketTextFrameHandler(AsyncWebSocket * server, AsyncWebSocketClient * client, const uint8_t *payload, size_t len);
#endif