/*
 * Project: Bait Boat Control System (ESP32) - WebServer Module
 * Description: Handles WiFi and web server setup
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#include "webserver.h"
#include "telemetry.h"

extern WebServer server;
extern Preferences preferences;
extern portMUX_TYPE nvsMutex;
extern char ssid[33];

void setupWifi() {
  portENTER_CRITICAL(&nvsMutex);
  preferences.begin("baitboat", true);
  String ap_ssid = preferences.getString("wifi_ssid", "Navigator-Pro");
  String ap_pass = preferences.getString("wifi_pass", "Nav@1234!");
  preferences.end();
  portEXIT_CRITICAL(&nvsMutex);

  if (ap_pass.length() > 0 && ap_pass.length() < 8) {
      ap_pass = "Nav@1234!";
  }

  strncpy(ssid, ap_ssid.c_str(), sizeof(ssid)-1);
  ssid[sizeof(ssid)-1] = '\0';

  WiFi.softAP(ap_ssid.c_str(), ap_pass.c_str());
}

void webserver_task(void *pvParameters) {
    setupWifi();
    setupWebServerRoutes();
    while(true) {
        server.handleClient();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
