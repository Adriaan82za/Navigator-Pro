/*
 * Project: Bait Boat Control System (ESP32) - WebServer Module
 * Description: Handles WiFi and web server setup
 * Author: [Adriaan v.d.Westhuizen]
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#ifndef NAV_WEBSERVER_H
#define NAV_WEBSERVER_H

#include <WiFi.h>
#include <Preferences.h>

// Forward declare WebServer class
class WebServer;

// ==================================
// Function Declarations
// ==================================
void setupWifi();
void webserver_task(void *pvParameters);
void setupWebServerRoutes(); 

// ==================================
// External declarations
// ==================================
extern WebServer server;
extern Preferences preferences;
extern SemaphoreHandle_t nvsMutex; // FIXED: Changed to SemaphoreHandle_t
extern char ssid[33];

#endif