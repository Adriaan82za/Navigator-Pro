/*
 * Project: Bait Boat Control System (ESP32) - Telemetry Module
 * Description: Handles the web server and provides a web-based UI.
 */

#include "telemetry.h"
#include "autopilot.h"
#include <WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "imu.h"
#include <TinyGPSPlus.h> 

extern SemaphoreHandle_t boatStatusMutex;
extern SemaphoreHandle_t nvsMutex;
extern SemaphoreHandle_t pidMutex;
extern SemaphoreHandle_t routeMutex;
extern SemaphoreHandle_t dataMutex;
extern SemaphoreHandle_t i2cMutex;

extern int currentOrientationIndex; 
extern TinyGPSPlus gps; 

enum RestoreStatus { RESTORE_IDLE, RESTORE_SUCCESS, RESTORE_FAILED };
static volatile RestoreStatus restoreUploadStatus = RESTORE_IDLE;

void flagSettingsChanged() {
    if (xSemaphoreTake(boatStatusMutex, 10) == pdTRUE) {
        boatStatus.persistence.settings_dirty = true;
        boatStatus.persistence.last_change_ms = millis();
        xSemaphoreGive(boatStatusMutex);
    }
}

void sendJsonResponse(int code, bool success, const char* message) {
    DynamicJsonDocument doc(256); 
    doc["success"] = success;
    doc["message"] = message;
    String response;
    serializeJson(doc, response);
    server.send(code, "application/json", response);
}

String getContentType(String filename) {
  if (server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".json")) return "application/json";
  else if (filename.endsWith(".csv")) return "text/csv";
  return "text/plain";
}

bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleRestoreUpload(); 

void setupWebServerRoutes() {
  if (MDNS.begin("baitboat")) {
    MDNS.addService("http", "tcp", 80);
  }

  server.on("/telemetry", HTTP_GET, [](){
    DynamicJsonDocument doc(4096); 
    
    double localLat = 0.0;
    double localLng = 0.0;
    bool localHasFix = false;

    if (xSemaphoreTake(boatStatusMutex, 20) == pdTRUE) {
        doc["ip_address"] = WiFi.softAPIP().toString();

        switch (boatStatus.mode.current) {
          case MANUAL_MODE: doc["mode"] = "MANUAL"; break;
          case AUTOPILOT_MODE: doc["mode"] = "AUTOPILOT"; break;
          case LOCATION_SAVE_MODE: doc["mode"] = "LOCATION SAVE"; break;
          case ANCHOR_MODE: doc["mode"] = "ANCHOR"; break;
          case AUTOTUNE_MODE: doc["mode"] = "AUTO-TUNING"; break; // <--- UPDATE
        }

        doc["armed"] = boatStatus.mode.is_armed;
        doc["autopilot_status"] = boatStatus.autopilot.engaged ? "YES" : "NO";
        doc["rth"] = boatStatus.autopilot.rth_active ? "YES" : "NO";
        
        if (boatStatus.autopilot.engaged && boatStatus.autopilot.target_waypoint_index != -1) {
          if (boatStatus.autopilot.target_waypoint_index >= 0 && boatStatus.autopilot.target_waypoint_index < 5) {
             doc["waypoint_idx"] = boatStatus.autopilot.target_waypoint_index; 
          } else {
             doc["waypoint"] = "INVALID";
          }
        } else {
          doc["waypoint"] = "NONE";
        }

        doc["heading"] = String(boatStatus.nav.heading, 1);
        doc["pitch"] = String(boatStatus.nav.pitch, 1);
        doc["roll"] = String(boatStatus.nav.roll, 1);
        doc["imu_confidence"] = boatStatus.nav.imu_accuracy;
        doc["imu_orientation"] = currentOrientationIndex;

        doc["low_batt"] = boatStatus.mode.low_battery_threshold;
        doc["low_batt_rth"] = boatStatus.autopilot.low_battery_rth_enabled;
        doc["brake_dist"] = boatStatus.autopilot.braking_distance;
        doc["declination"] = String(boatStatus.nav.magnetic_declination, 2);
        doc["battery"] = String(boatStatus.vitals.battery_voltage, 2);
        
        localHasFix = boatStatus.nav.has_gps_fix;
        doc["gps_fix"] = localHasFix ? "YES" : "NO";
        
        doc["sbas_active"] = boatStatus.nav.sbas_active;
        
        if (localHasFix) {
            localLat = boatStatus.nav.latitude;
            localLng = boatStatus.nav.longitude;
            doc["latitude"] = String(localLat, 6);
            doc["longitude"] = String(localLng, 6);
            doc["speed"] = String(boatStatus.nav.speed_mps, 2);
        } else {
            doc["latitude"] = "N/A";
            doc["longitude"] = "N/A";
            doc["speed"] = "N/A";
        }
        
        xSemaphoreGive(boatStatusMutex);
    }
    
    doc["wifi_ssid"] = String(ssid);
    doc["satellites"] = gps.satellites.isValid() ? String(gps.satellites.value()) : "N/A";

    if (gps.date.isValid() && gps.time.isValid()) {
        char dateTimeStr[32];
        snprintf(dateTimeStr, sizeof(dateTimeStr), "%04d-%02d-%02d %02d:%02d:%02d",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
        doc["gps_time"] = dateTimeStr;
    } else {
        doc["gps_time"] = "Waiting for Sync...";
    }

    if (xSemaphoreTake(pidMutex, 20) == pdTRUE) {
        JsonObject pid_obj = doc.createNestedObject("pid");
        pid_obj["p"] = steeringPID.Kp;
        pid_obj["i"] = steeringPID.Ki;
        pid_obj["d"] = steeringPID.Kd;
        xSemaphoreGive(pidMutex);
    }

    if (xSemaphoreTake(routeMutex, 20) == pdTRUE) {
        JsonObject route_obj = doc.createNestedObject("route");
        switch(currentRoute.status) {
            case ROUTE_INACTIVE: route_obj["status"] = "INACTIVE"; break;
            case ROUTE_ACTIVE: route_obj["status"] = "ACTIVE - Step " + String(currentRoute.currentStep + 1) + "/" + String(currentRoute.waypointCount); break;
            case ROUTE_PAUSED: route_obj["status"] = "PAUSED - Step " + String(currentRoute.currentStep + 1) + "/" + String(currentRoute.waypointCount); break;
            case ROUTE_COMPLETE: route_obj["status"] = "COMPLETE - Returning Home"; break;
        }
        xSemaphoreGive(routeMutex);
    }

    if (xSemaphoreTake(dataMutex, 50) == pdTRUE) {
        if (doc.containsKey("waypoint_idx")) {
            int idx = doc["waypoint_idx"];
            doc["waypoint"] = savedLocations[idx].name;
            doc.remove("waypoint_idx");
        }

        if (localHasFix && savedLocations[0].isSet) {
             doc["distance"] = String(calculateDistance(localLat, localLng, savedLocations[0].lat, savedLocations[0].lng), 2);
        } else {
             doc["distance"] = "N/A";
        }

        JsonObject alerts = doc.createNestedObject("alerts");
        const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
        const char* names[] = {"Home Saved", "Waypoint Saved", "GPS Fix Acquired", "Autopilot Engaged", "Left Hopper Drop", "Right Hopper Drop", "Left Hook Release", "Right Hook Release", "Low Battery", "Wi-Fi Reset", "Armed"};
        for(int i = 0; i < NUM_ALERT_TYPES; i++){
            JsonObject alert_obj = alerts.createNestedObject(keys[i]);
            alert_obj["name"] = names[i];
            alert_obj["b"] = alertSettings[i].beeps; alert_obj["bd"] = alertSettings[i].beepDuration; alert_obj["pd"] = alertSettings[i].pauseDuration;
            alert_obj["f"] = alertSettings[i].flashes; alert_obj["fd"] = alertSettings[i].flashOnDuration; alert_obj["fo"] = alertSettings[i].flashOffDuration; alert_obj["fm"] = alertSettings[i].flashMask;
        }

        JsonArray locationsArray = doc.createNestedArray("locations");
        for (int i = 0; i < 5; i++) {
          JsonObject locObj = locationsArray.createNestedObject();
          locObj["set"] = savedLocations[i].isSet;
          locObj["name"] = savedLocations[i].name;
          if (savedLocations[i].isSet) {
            locObj["lat"] = String(savedLocations[i].lat, 6);
            locObj["lng"] = String(savedLocations[i].lng, 6);
            if (i > 0 && savedLocations[0].isSet) {
                double distToHome = calculateDistance(savedLocations[i].lat, savedLocations[i].lng, savedLocations[0].lat, savedLocations[0].lng);
                locObj["dist_from_home"] = String(distToHome, 1);
            } else {
                locObj["dist_from_home"] = "N/A";
            }
            locObj["dropLeftHopper"] = savedLocations[i].dropLeftHopper;
            locObj["dropRightHopper"] = savedLocations[i].dropRightHopper;
            locObj["releaseLeftHook"] = savedLocations[i].releaseLeftHook;
            locObj["releaseRightHook"] = savedLocations[i].releaseRightHook;
            locObj["autoReturnToHome"] = savedLocations[i].autoReturnToHome;
          }
        }
        xSemaphoreGive(dataMutex);
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  server.on("/start_autotune", HTTP_POST, [](){
    if (xSemaphoreTake(boatStatusMutex, 50) == pdTRUE) {
        if (!boatStatus.nav.has_gps_fix) {
            sendJsonResponse(400, false, "Cannot Auto-Tune without a solid GPS Fix!");
            xSemaphoreGive(boatStatusMutex);
            return;
        }
        boatStatus.mode.current = AUTOTUNE_MODE;
        xSemaphoreGive(boatStatusMutex);
        sendJsonResponse(200, true, "Auto-Tune Started! Boat will weave 5 times.");
    } else {
        sendJsonResponse(503, false, "System busy.");
    }
  });

  server.on("/save_pid", HTTP_POST, [](){
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("p") || !doc.containsKey("i") || !doc.containsKey("d")) {
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return;
    }
    float p = doc["p"], i = doc["i"], d = doc["d"];
    if (isnan(p) || isnan(i) || isnan(d) || p < 0 || p > 100.0 || i < 0 || i > 100.0 || d < 0 || d > 100.0) {
      sendJsonResponse(400, false, "Invalid PID values (must be numbers between 0 and 100)."); return;
    }

    if (xSemaphoreTake(pidMutex, 50) == pdTRUE) {
        steeringPID.Kp = p;
        steeringPID.Ki = i;
        steeringPID.Kd = d;
        xSemaphoreGive(pidMutex);
        flagSettingsChanged();
        sendJsonResponse(200, true, "PID gains updated. Will be saved shortly.");
    } else {
        sendJsonResponse(503, false, "System busy, try again.");
    }
  });

  server.on("/save_system_settings", HTTP_POST, [](){
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("low_batt") || !doc.containsKey("low_batt_rth") || !doc.containsKey("brake_dist")) {
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return;
    }
    
    float low_batt = doc["low_batt"];
    float brake_dist = doc["brake_dist"];
    
    if (isnan(low_batt) || low_batt <= 0 || low_batt > 20.0 || isnan(brake_dist) || brake_dist < 0) {
      sendJsonResponse(400, false, "Invalid values."); return;
    }

    if (xSemaphoreTake(boatStatusMutex, 50) == pdTRUE) {
        boatStatus.mode.low_battery_threshold = low_batt;
        boatStatus.autopilot.low_battery_rth_enabled = doc["low_batt_rth"];
        boatStatus.autopilot.braking_distance = brake_dist;
        xSemaphoreGive(boatStatusMutex);
        flagSettingsChanged();
        sendJsonResponse(200, true, "System settings updated.");
    } else {
        sendJsonResponse(503, false, "System busy.");
    }
  });

  server.on("/save_alert_settings", HTTP_POST, [](){
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("key")) {
      sendJsonResponse(400, false, "Invalid JSON."); return;
    }

    String key = doc["key"];
    const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
    int index = -1;
    for(int i = 0; i < NUM_ALERT_TYPES; i++){ if(key == keys[i]){ index = i; break;} }

    if(index == -1){
        sendJsonResponse(400, false, "Invalid alert key.");
        return;
    }

    int b = doc["b"], f = doc["f"], fm = doc["fm"];
    unsigned long bd = doc["bd"], pd = doc["pd"], fd = doc["fd"], fo = doc["fo"];

    if (xSemaphoreTake(dataMutex, 50) == pdTRUE) {
        alertSettings[index] = {b, bd, pd, f, fd, fo, (byte)fm};
        xSemaphoreGive(dataMutex);
        flagSettingsChanged();
        sendJsonResponse(200, true, "Alert settings updated.");
    } else {
        sendJsonResponse(503, false, "System busy.");
    }
  });

  server.on("/save_wifi", HTTP_POST, [](){
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("ssid")) {
      sendJsonResponse(400, false, "Invalid JSON."); return;
    }

    const char* new_ssid = doc["ssid"];
    const char* new_pass = doc["pass"];

    if (!new_ssid || strlen(new_ssid) == 0) { sendJsonResponse(400, false, "SSID cannot be empty."); return; }
    
    if (xSemaphoreTake(nvsMutex, 1000) == pdTRUE) {
        preferences.begin("baitboat", false);
        preferences.putString("wifi_ssid", new_ssid);
        if (new_pass && strlen(new_pass) > 0) {
            preferences.putString("wifi_pass", new_pass);
        } else if (new_pass && strlen(new_pass) == 0) {
            preferences.remove("wifi_pass");
        }
        preferences.end();
        xSemaphoreGive(nvsMutex);
        
        sendJsonResponse(200, true, "Wi-Fi saved. Rebooting...");
        delay(1000);
        ESP.restart();
    } else {
        sendJsonResponse(503, false, "NVS busy.");
    }
  });

  server.on("/control_route", HTTP_POST, [](){
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("command")) {
      sendJsonResponse(400, false, "Invalid JSON."); return;
    }
    String command = doc["command"];

    if (xSemaphoreTake(boatStatusMutex, 50) == pdTRUE) {
        if (xSemaphoreTake(routeMutex, 50) == pdTRUE) {
            
            if (command == "start") {
                JsonArray waypoints = doc["waypoints"];
                if (waypoints.size() > 0 && waypoints.size() <= MAX_ROUTE_WAYPOINTS) {
                    currentRoute.waypointCount = waypoints.size();
                    for(int i=0; i < currentRoute.waypointCount; i++) {
                        currentRoute.waypoints[i] = waypoints[i].as<int>();
                    }
                    currentRoute.currentStep = 0;
                    currentRoute.status = ROUTE_ACTIVE;
                    sendJsonResponse(200, true, "Route started!");
                } else {
                    sendJsonResponse(400, false, "Invalid route.");
                }

            } else if (command == "stop") {
                currentRoute.status = ROUTE_INACTIVE;
                boatStatus.autopilot.engaged = false;
                boatStatus.autopilot.rth_active = false;
                boatStatus.autopilot.target_waypoint_index = -1;
                boatStatus.autopilot.arrival_state = AP_IDLE;
                sendJsonResponse(200, true, "Route stopped!");

            } else if (command == "pause") {
                if (currentRoute.status == ROUTE_ACTIVE) currentRoute.status = ROUTE_PAUSED;
                sendJsonResponse(200, true, "Route paused!");

            } else if (command == "resume") {
                if (currentRoute.status == ROUTE_PAUSED) currentRoute.status = ROUTE_ACTIVE;
                sendJsonResponse(200, true, "Route resumed!");
            }

            xSemaphoreGive(routeMutex);
        } else {
            sendJsonResponse(503, false, "Route system busy.");
        }
        xSemaphoreGive(boatStatusMutex);
    } else {
        sendJsonResponse(503, false, "System busy.");
    }
  });

  server.on("/save_location_name", HTTP_POST, [](){
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("index") || !doc.containsKey("name")) {
      sendJsonResponse(400, false, "Invalid JSON."); return;
    }

    int locIndex = doc["index"];
    String newName = doc["name"];
    if (locIndex < 1 || locIndex >= 5) { sendJsonResponse(400, false, "Invalid index."); return; }

    if (xSemaphoreTake(dataMutex, 50) == pdTRUE) {
        savedLocations[locIndex].name = newName;
        xSemaphoreGive(dataMutex);
        flagSettingsChanged();
        sendJsonResponse(200, true, "Location name updated.");
    } else {
        sendJsonResponse(503, false, "System busy.");
    }
  });

  server.on("/set_location_actions", HTTP_POST, [](){
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("locationIndex")) {
      sendJsonResponse(400, false, "Invalid JSON."); return;
    }
    int locIndex = doc["locationIndex"];
    
    if (xSemaphoreTake(dataMutex, 50) == pdTRUE) {
        if (locIndex >= 1 && locIndex < 5 && savedLocations[locIndex].isSet) {
            savedLocations[locIndex].dropLeftHopper = doc["dropLeftHopper"];
            savedLocations[locIndex].dropRightHopper = doc["dropRightHopper"];
            savedLocations[locIndex].releaseLeftHook = doc["releaseLeftHook"];
            savedLocations[locIndex].releaseRightHook = doc["releaseRightHook"];
            savedLocations[locIndex].autoReturnToHome = doc["autoReturnToHome"];
            xSemaphoreGive(dataMutex);
            flagSettingsChanged();
            sendJsonResponse(200, true, "Actions updated.");
        } else {
            xSemaphoreGive(dataMutex);
            sendJsonResponse(400, false, "Invalid location.");
        }
    } else {
        sendJsonResponse(503, false, "System busy.");
    }
  });

  server.on("/delete_location", HTTP_POST, [](){
    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    int locIndex = doc["locationIndex"];
    if (locIndex < 1 || locIndex >= 5) { sendJsonResponse(400, false, "Invalid index."); return; }

    if (xSemaphoreTake(nvsMutex, 200) == pdTRUE) {
        preferences.begin("baitboat", false);
        preferences.remove(("loc" + String(locIndex) + "Lat").c_str());
        preferences.remove(("loc" + String(locIndex) + "Lng").c_str());
        preferences.putBool(("loc" + String(locIndex) + "Set").c_str(), false);
        preferences.end();
        xSemaphoreGive(nvsMutex);
        
        if (xSemaphoreTake(dataMutex, 50) == pdTRUE) {
            savedLocations[locIndex] = {0.0, 0.0, false, "Waypoint " + String(locIndex), false, false, false, false, false};
            xSemaphoreGive(dataMutex);
        }
        sendJsonResponse(200, true, "Location deleted.");
    } else {
        sendJsonResponse(503, false, "Storage busy.");
    }
  });

  server.on("/backup_settings", HTTP_GET, [](){
      if (xSemaphoreTake(nvsMutex, 1000) == pdTRUE) {
          preferences.begin("baitboat", true);
          DynamicJsonDocument doc(4096);

          doc["pid_p"] = preferences.getFloat("pid_p", 4.0);
          doc["pid_i"] = preferences.getFloat("pid_i", 0.1);
          doc["pid_d"] = preferences.getFloat("pid_d", 0.5);
          doc["low_batt"] = preferences.getFloat("low_batt", 10.5);
          doc["lb_rth_en"] = preferences.getBool("lb_rth_en", false);
          doc["brake_dist"] = preferences.getFloat("brake_dist", 3.0);

          const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
          char key_buffer[25];
          
          for(int i = 0; i < NUM_ALERT_TYPES; i++) {
              bool is_lb = (i == 8); 
              bool is_wr = (i == 9); 
              bool is_armed = (i == 10); 
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
              doc[key_buffer] = preferences.getInt(key_buffer, is_lb ? 10000 : (is_wr ? 3 : (is_armed ? 2 : (i==0?4:1))));
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
              doc[key_buffer] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 250 : (is_armed ? 80 : 150)));
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
              doc[key_buffer] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 150 : (is_armed ? 80 : 100)));
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
              doc[key_buffer] = preferences.getInt(key_buffer, is_lb ? 10000 : (is_wr ? 3 : (is_armed ? 2 : (i<4?3:7))));
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
              doc[key_buffer] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 250 : (is_armed ? 80 : 150)));
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
              doc[key_buffer] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 150 : (is_armed ? 80 : 150)));
              
              snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
              doc[key_buffer] = preferences.getUChar(key_buffer, 255); 
          }
          
          String response;
          serializeJson(doc, response);
          
          preferences.end();
          xSemaphoreGive(nvsMutex);

          server.sendHeader("Content-Disposition", "attachment; filename=baitboat_backup.json");
          server.send(200, "application/json", response);
      } else {
          server.send(503, "text/plain", "System busy");
      }
  });

  server.on("/set_orientation", HTTP_POST, [](){
      sendJsonResponse(200, true, "WT901 handles orientation internally.");
  });

  server.on("/save_imu_cal", HTTP_POST, [](){
    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if(error || !doc.containsKey("command")) {
      sendJsonResponse(400, false, "Invalid JSON format."); return;
    }
    
    String cmd = doc["command"];
    
    if (cmd == "start") {
        if (startMagneticCalibration()) {
            sendJsonResponse(200, true, "Calibration started. Spin the boat 360 degrees on all axes, then click Finish.");
        } else {
            sendJsonResponse(500, false, "Failed to connect to WT901 via I2C.");
        }
    } else if (cmd == "stop") {
        if (endMagneticCalibration()) {
            sendJsonResponse(200, true, "Magnetic Calibration saved perfectly to WT901 Flash!");
        } else {
            sendJsonResponse(500, false, "Failed to save IMU calibration.");
        }
    } else {
        sendJsonResponse(400, false, "Unknown command.");
    }
  });

  server.on("/restore_settings", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      if (restoreUploadStatus == RESTORE_SUCCESS) {
        String successPage = R"rawliteral(<html><head><meta http-equiv='refresh' content='5;url=/'/></head><body><h2>Restored! Rebooting...</h2></body></html>)rawliteral";
        server.send(200, "text/html", successPage);
        delay(1000);
        ESP.restart();
      } else {
        server.send(400, "text/html", "Restore Failed");
      }
      restoreUploadStatus = RESTORE_IDLE;
  }, handleRestoreUpload);

  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "404: Not Found");
    }
  });

  server.begin();
}

void handleRestoreUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    restoreUploadStatus = RESTORE_IDLE;
  } else if (upload.status == UPLOAD_FILE_END) {
    
    DynamicJsonDocument* doc = new DynamicJsonDocument(4096);
    DeserializationError error = deserializeJson(*doc, upload.buf, upload.currentSize);

    if (error) { 
        restoreUploadStatus = RESTORE_FAILED; 
        delete doc; 
        return; 
    }

    if (xSemaphoreTake(nvsMutex, 2000) == pdTRUE) {
        preferences.begin("baitboat", false);
        
        if (doc->containsKey("pid_p")) preferences.putFloat("pid_p", (*doc)["pid_p"]);
        if (doc->containsKey("pid_i")) preferences.putFloat("pid_i", (*doc)["pid_i"]);
        if (doc->containsKey("pid_d")) preferences.putFloat("pid_d", (*doc)["pid_d"]);
        if (doc->containsKey("low_batt")) preferences.putFloat("low_batt", (*doc)["low_batt"]);
        if (doc->containsKey("lb_rth_en")) preferences.putBool("lb_rth_en", (*doc)["lb_rth_en"]);
        if (doc->containsKey("brake_dist")) preferences.putFloat("brake_dist", (*doc)["brake_dist"]);

        const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
        char key_buffer[25];
        
        for (int i = 0; i < NUM_ALERT_TYPES; i++) {
            snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putInt(key_buffer, (*doc)[key_buffer]);
            
            snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putULong(key_buffer, (*doc)[key_buffer]);
            
            snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putULong(key_buffer, (*doc)[key_buffer]);
            
            snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putInt(key_buffer, (*doc)[key_buffer]);
            
            snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putULong(key_buffer, (*doc)[key_buffer]);
            
            snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putULong(key_buffer, (*doc)[key_buffer]);
            
            snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
            if (doc->containsKey(key_buffer)) preferences.putUChar(key_buffer, (*doc)[key_buffer]);
        }
        
        preferences.end();
        xSemaphoreGive(nvsMutex);
        restoreUploadStatus = RESTORE_SUCCESS;
    } else {
        restoreUploadStatus = RESTORE_FAILED;
    }
    
    delete doc; 
  }
}