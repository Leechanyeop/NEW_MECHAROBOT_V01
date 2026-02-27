#include <WiFi.h>

const char* ssid = "hhme";
const char* password = "hme*12345";
WiFiServer server(502);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nConnected!");
    Serial.println(WiFi.localIP());
    
    server.begin();
    Serial.println("Server started on 502");
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        Serial.println("Client connected!");
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                Serial.print(c);
            }
        }
        client.stop();
        Serial.println("Client disconnected");
    }
}