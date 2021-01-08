/**
 * @file webSocketServer.cpp
 * @brief Implementation of the WebSocket server module
 * @date 2021-01-08
 * @author DIY Embedded Systems (diy.embeddedsytems@gmail.com)
 * 
 */


#include "webSocketServer.h"

#include <Arduino.h>
#include <ESP8266WiFi.h>        /* WiFi functions */
#include <ESPAsyncTCP.h>        /* Asynchronous web and websocket servers  */
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>           /* LittleFS flash filesystem (replaces deprecated SPIFFS)*/

#include <DupLogger.h>          /* Logging library */

extern DupLogger logger;        /* logger to be used (defined in main) */
extern AsyncWebServer httpServer; /* main AsyncTCP server */

AsyncWebSocket wsServer("/ws");  /* Websocket server instance */


/**
 * @brief Configure websocket server
 */
void webSocketServerSetup()
{
  wsServer.onEvent(websocketEventHandler);
  httpServer.addHandler(&wsServer);
}


/**
 * @brief Websocket server housekeeping
 */
void webSocketServerLoop()
{
  static uint32_t nextMs = 0;
  const uint32_t periodMs = 500;

  if ((int32_t)(millis() - nextMs) >= 0)
  {
    nextMs += periodMs;
    wsServer.cleanupClients();
  }
}


/**
 * @brief Callback that handles generic websocket events
 * @param server
 * @param client
 * @param eventType
 * @param arg
 * @param payload
 * @param len
 */
void websocketEventHandler(AsyncWebSocket * server, AsyncWebSocketClient * client, 
    AwsEventType eventType, void * arg, uint8_t *payload, size_t len)
{
  AwsFrameInfo *frameInfo;

  switch (eventType)
  {
  case WS_EVT_CONNECT:
    logger.info("[WS] New client #%u (%s)", client->id(), client->remoteIP().toString().c_str());
    client->text("Welcome");
    break;

  case WS_EVT_DISCONNECT:
    logger.info("[WS] Client #%u disconnected", client->id());
    break;

  case WS_EVT_PONG:
    break;

  case WS_EVT_DATA:
    frameInfo = (AwsFrameInfo *)arg;
    // is this frame a full unfragmented websocket TEXT frame?
    if (!frameInfo->final || frameInfo->index != 0 || frameInfo->len != len)
    {
      logger.warn("[WS] frame fragmentation not supported");
    }
    else if (frameInfo->opcode != WS_TEXT)
    {
      logger.warn("[WS] binary frame not supported");
    }
    else
    {
      logger.info("[WS] Client #%u: has message '%.*s'", 
          client->id(), len, payload);
      // Simply echo
      wsServer.textAll(payload, len);
    }
    break;

  case WS_EVT_ERROR:
    logger.warn("[WS] client #%u error #%u `%.*s`", client->id(), *(uint16_t*)arg, len, (char *)payload);
    break;

  default:
    break;
  }
}
