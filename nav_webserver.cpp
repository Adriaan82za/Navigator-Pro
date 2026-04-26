/*
 * Project: Bait Boat Control System (ESP32) - WebServer Module
 * Description: Handles WiFi and web server setup
 * Author: [Adriaan v.d.Westhuizen]
 * Date: April 12, 2026
 * Version: 14.5.1 (WDT Integration & Logger)
 */

#include <Arduino.h>
#include <WebServer.h>
#include <esp_task_wdt.h>
#include <SPIFFS.h>
#include "nav_webserver.h"
#include "telemetry.h"

extern WebServer server;
extern Preferences preferences;
extern SemaphoreHandle_t nvsMutex;
extern char ssid[33];
void initLogger();

void setupWifi() {
  if (xSemaphoreTake(nvsMutex, 500 / portTICK_PERIOD_MS) == pdTRUE) {
      preferences.begin("baitboat", true);
      String ap_ssid = preferences.getString("wifi_ssid", "Navigator-Pro");
      String ap_pass = preferences.getString("wifi_pass", "Nav@1234!");
      preferences.end();
      xSemaphoreGive(nvsMutex);

      if (ap_pass.length() > 0 && ap_pass.length() < 8) {
          ap_pass = "Nav@1234!";
      }

      strncpy(ssid, ap_ssid.c_str(), sizeof(ssid)-1);
      ssid[sizeof(ssid)-1] = '\0';

      WiFi.softAP(ap_ssid.c_str(), ap_pass.c_str());
  } else {
      strncpy(ssid, "Navigator-Pro-Safe", sizeof(ssid)-1);
      WiFi.softAP("Navigator-Pro-Safe", "Nav@1234!");
  }
}

void setupLoggerRoutes() {
    server.on("/log", HTTP_GET, []() {
        extern SemaphoreHandle_t logMutex;
        if (xSemaphoreTake(logMutex, 100) == pdTRUE) {
            if (SPIFFS.exists("/telemetry.csv")) {
                File file = SPIFFS.open("/telemetry.csv", "r");
                server.streamFile(file, "text/csv");
                file.close();
            } else {
                server.send(404, "text/plain", "Log file not found.");
            }
            xSemaphoreGive(logMutex);
        } else {
            server.send(503, "text/plain", "System Busy");
        }
    });

    server.on("/clear_log", HTTP_GET, []() {
        extern SemaphoreHandle_t logMutex;
        if (xSemaphoreTake(logMutex, 100) == pdTRUE) {
            SPIFFS.remove("/telemetry.csv");
            server.send(200, "text/plain", "Log Cleared.");
            xSemaphoreGive(logMutex);
            initLogger(); 
        } else {
            server.send(503, "text/plain", "System Busy");
        }
    });
}

void webserver_task(void *pvParameters) {
    esp_task_wdt_add(NULL); 
    
    setupWifi();
    setupLoggerRoutes();
    setupWebServerRoutes();
    
    while(true) {
        esp_task_wdt_reset(); 
        
        server.handleClient();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}