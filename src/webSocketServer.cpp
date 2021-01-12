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
#include <AsyncWebSocket.h>
#include <LittleFS.h>           /* LittleFS flash filesystem (replaces deprecated SPIFFS)*/
#include <ArduinoJson.h>        /* JSON parsing */

#include <DupLogger.h>          /* Logging library */


extern DupLogger logger;        /* logger to be used (defined in main) */
extern AsyncWebServer httpServer; /* main AsyncTCP server */

AsyncWebSocket wsServer("/ws");  /* Websocket server instance */

extern char userMessage[128];

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
    webSocketClientConnectHandler(server, client);
    break;

  case WS_EVT_DISCONNECT:
    webSocketClientDisconnectHandler(server, client);
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
      logger.warn("[WS] BIN frame not supported");
    }
    else
    {
      webSocketTextFrameHandler(server, client, payload, len);
    }
    break;

  case WS_EVT_ERROR:
    logger.warn("[WS] client #%u error #%u `%.*s`", client->id(), *(uint16_t*)arg, len, (char *)payload);
    break;

  default:
    logger.error("[WS] Event type %u not supported", eventType);
    break;
  }
}

/**
 * @brief This handler is called whenever a client connects
 */
void webSocketClientConnectHandler(AsyncWebSocket * server, AsyncWebSocketClient * client)
{
  char msg[256];
  snprintf(msg, sizeof(msg), "New client #%u (%s)", client->id(), client->remoteIP().toString().c_str());
  logger.info("[WS] %s", msg);
  
  client->text(msg);
}

/**
 * @brief This handler is called whenever a client discconnects
 */
void webSocketClientDisconnectHandler(AsyncWebSocket * server, AsyncWebSocketClient * client)
{
  logger.info("[WS] Client #%u disconnected", client->id());
}

/**
 * @brief This handler is called whenever a text frame is received for client
 */
void webSocketTextFrameHandler(AsyncWebSocket * server, AsyncWebSocketClient * client, const uint8_t *payload, size_t len)
{
    static StaticJsonDocument<JSON_MEMORY_SIZE> jsonDoc;

    logger.info("[WS] #%u <- `%.*s`", 
          client->id(), len, payload);
    // Is message a JSON?
    DeserializationError error = deserializeJson(jsonDoc, (const char *)payload);
    if (!error)
    {
      // message payload is valid JSON
      webSocketJsonFrameHandler(server, client, jsonDoc);
    }
    else
    {
      // message is not valid JSON
      // Default behavior: broadcast message to all connected clients
      server->textAll((const char*)payload, len);
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
  else
  {
    logger.info("[WS] message field not found");
  }
}