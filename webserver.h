/*
 * Project: Bait Boat Control System (ESP32) - WebServer Module
 * Description: Handles WiFi and web server setup
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized)
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ==================================
// Function Declarations
// ==================================
void setupWifi();
void webserver_task(void *pvParameters);

// ==================================
// External declarations
// ==================================
extern WebServer server;
extern Preferences preferences;
extern portMUX_TYPE nvsMutex;
extern char ssid[33];

#endif
