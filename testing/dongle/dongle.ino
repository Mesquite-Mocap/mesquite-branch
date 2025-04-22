// esp32 websocket server

#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "finger-tracking"; // SSID for the access point
const char* password = "12345678";       // Password for the access point

WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("%s\n", payload);
      webSocket.sendTXT(num, "reply: " + String((char *)payload));
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Set up the ESP32 as an access point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP()); // Print the IP address of the access point

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
}