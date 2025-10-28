/*
 * Project: Bait Boat Control System (ESP32) - WebServer Module
 * Description: Handles WiFi and web server setup
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#ifndef NAV_WEBSERVER_H // <-- CHANGED GUARD
#define NAV_WEBSERVER_H // <-- CHANGED GUARD

#include <WiFi.h>
// #include <WebServer.h> // <-- DELETED THIS LINE
#include <Preferences.h>

// Forward declare WebServer class instead of including the header
class WebServer;

// ==================================
// Function Declarations
// ==================================
void setupWifi();
void webserver_task(void *pvParameters);

// ==================================
// External declarations
// ==================================
extern WebServer server; // Declaration is okay here
extern Preferences preferences;
extern portMUX_TYPE nvsMutex;
extern char ssid[33];

#endif