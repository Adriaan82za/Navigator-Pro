/*
 * Project: Bait Boat Control System (ESP32) - Telemetry Module
 * Description: Handles the web server and provides a web-based UI for
 * monitoring and configuration.
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 18, 2025
 * Version: 7.8.5 (Final Bug Fixes)
 */

#include "telemetry.h"
#include "autopilot.h"
#include <WebServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>

// State variable to track the outcome of the restore process
enum RestoreStatus { RESTORE_IDLE, RESTORE_SUCCESS, RESTORE_FAILED };
static volatile RestoreStatus restoreUploadStatus = RESTORE_IDLE;

// Helper function to flag that settings need to be saved
void flagSettingsChanged() {
    boatStatus.persistence.settings_dirty = true;
    boatStatus.persistence.last_change_ms = millis();
}

// Helper function for sending standardized JSON responses
void sendJsonResponse(int code, bool success, const char* message) {
    StaticJsonDocument<128> doc;
    doc["success"] = success;
    doc["message"] = message;
    String response;
    serializeJson(doc, response);
    server.send(code, "application/json", response);
}

// Helper function to get the content type for a given file extension
String getContentType(String filename) {
  if (server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".json")) return "application/json";
  else if (filename.endsWith(".csv")) return "text/csv";
  return "text/plain";
}

// Helper function to serve files from SPIFFS
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

// ==============================
// Web Server Setup
// ==============================
void handleRestoreUpload();

void setupWebServerRoutes() {
  if (MDNS.begin("baitboat")) {
    MDNS.addService("http", "tcp", 80);
  }

  // API endpoint for all boat telemetry data
  server.on("/telemetry", [](){
    StaticJsonDocument<2048> doc;
    
    // <<< FIX: Protect all reads of shared data
    portENTER_CRITICAL(&boatStatusMutex); 
    
    doc["ip_address"] = WiFi.softAPIP().toString();
    
    switch (boatStatus.mode.current) {
      case MANUAL_MODE: doc["mode"] = "MANUAL"; break;
      case AUTOPILOT_MODE: doc["mode"] = "AUTOPILOT"; break;
      case LOCATION_SAVE_MODE: doc["mode"] = "LOCATION SAVE"; break;
      case ANCHOR_MODE: doc["mode"] = "ANCHOR"; break;
    }
    
    doc["armed"] = boatStatus.mode.is_armed;
    doc["autopilot_status"] = boatStatus.autopilot.engaged ? "YES" : "NO";
    doc["rth"] = boatStatus.autopilot.rth_active ? "YES" : "NO";
    if (boatStatus.autopilot.engaged && boatStatus.autopilot.target_waypoint_index != -1) {
      doc["waypoint"] = savedLocations[boatStatus.autopilot.target_waypoint_index].name;
    } else {
      doc["waypoint"] = "NONE";
    }

    doc["heading"] = String(boatStatus.nav.heading, 1);
    doc["pitch"] = String(boatStatus.nav.pitch, 1);
    doc["roll"] = String(boatStatus.nav.roll, 1);
    doc["wifi_ssid"] = String(ssid);
    doc["imu_confidence"] = boatStatus.nav.imu_accuracy;
    
    doc["low_batt"] = boatStatus.mode.low_battery_threshold;
    doc["low_batt_rth"] = boatStatus.autopilot.low_battery_rth_enabled;
    doc["declination"] = String(boatStatus.nav.magnetic_declination, 2);

    JsonObject pid_obj = doc.createNestedObject("pid");
    pid_obj["p"] = steeringPID.Kp;
    pid_obj["i"] = steeringPID.Ki;
    pid_obj["d"] = steeringPID.Kd;
    
    JsonObject route_obj = doc.createNestedObject("route");
    switch(currentRoute.status) {
        case ROUTE_INACTIVE: route_obj["status"] = "INACTIVE"; break;
        case ROUTE_ACTIVE: route_obj["status"] = "ACTIVE - Step " + String(currentRoute.currentStep + 1) + "/" + String(currentRoute.waypointCount); break;
        case ROUTE_PAUSED: route_obj["status"] = "PAUSED - Step " + String(currentRoute.currentStep + 1) + "/" + String(currentRoute.waypointCount); break;
        case ROUTE_COMPLETE: route_obj["status"] = "COMPLETE - Returning Home"; break;
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

    doc["battery"] = String(boatStatus.vitals.battery_voltage, 2);
    doc["gps_fix"] = boatStatus.nav.has_gps_fix ? "YES" : "NO";
    doc["satellites"] = gps.satellites.isValid() ? String(gps.satellites.value()) : "N/A";
    
    doc["latitude"] = boatStatus.nav.has_gps_fix ? String(boatStatus.nav.latitude, 6) : "N/A";
    doc["longitude"] = boatStatus.nav.has_gps_fix ? String(boatStatus.nav.longitude, 6) : "N/A";
    doc["speed"] = boatStatus.nav.has_gps_fix ? String(boatStatus.nav.speed_mps, 2) : "N/A";
    doc["distance"] = (boatStatus.nav.has_gps_fix && savedLocations[0].isSet) ? String(calculateDistance(boatStatus.nav.latitude, boatStatus.nav.longitude, savedLocations[0].lat, savedLocations[0].lng), 2) : "N/A";

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
    
    portEXIT_CRITICAL(&boatStatusMutex); // <<< FIX: Release mutex
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  server.on("/save_pid", HTTP_POST, [](){
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("p") || !doc.containsKey("i") || !doc.containsKey("d")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return; 
    }
    float p = doc["p"], i = doc["i"], d = doc["d"];
    if (isnan(p) || isnan(i) || isnan(d) || p < 0 || p > 100.0 || i < 0 || i > 100.0 || d < 0 || d > 100.0) {
      sendJsonResponse(400, false, "Invalid PID values (must be numbers between 0 and 100)."); return;
    }
    
    // <<< FIX: Protect writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    steeringPID.Kp = p;
    steeringPID.Ki = i;
    steeringPID.Kd = d;
    flagSettingsChanged();
    portEXIT_CRITICAL(&boatStatusMutex);
    
    sendJsonResponse(200, true, "PID gains updated. Will be saved shortly.");
  });

  server.on("/save_system_settings", HTTP_POST, [](){
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("low_batt") || !doc.containsKey("low_batt_rth")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return; 
    }
    float low_batt = doc["low_batt"];
    if (isnan(low_batt) || low_batt <= 0 || low_batt > 20.0) {
      sendJsonResponse(400, false, "Invalid low battery threshold (must be between 0 and 20)."); return;
    }

    // <<< FIX: Protect writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    boatStatus.mode.low_battery_threshold = low_batt;
    boatStatus.autopilot.low_battery_rth_enabled = doc["low_batt_rth"];
    flagSettingsChanged();
    portEXIT_CRITICAL(&boatStatusMutex);
    
    sendJsonResponse(200, true, "System settings updated. Will be saved shortly.");
  });

  server.on("/save_alert_settings", HTTP_POST, [](){
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("key") || !doc.containsKey("b") || !doc.containsKey("bd") || !doc.containsKey("pd") ||
       !doc.containsKey("f") || !doc.containsKey("fd") || !doc.containsKey("fo") || !doc.containsKey("fm")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return; 
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
    if (b < 0 || f < 0 || fm < 0 || bd == 0 || pd == 0 || fd == 0 || fo == 0) {
      sendJsonResponse(400, false, "Invalid alert values (must be positive integers)."); return;
    }

    // <<< FIX: Protect writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    alertSettings[index] = {b, bd, pd, f, fd, fo, (byte)fm};
    flagSettingsChanged();
    portEXIT_CRITICAL(&boatStatusMutex);
    
    sendJsonResponse(200, true, "Alert settings updated. Will be saved shortly.");
  });
  
  server.on("/save_wifi", HTTP_POST, [](){
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("ssid") || !doc.containsKey("pass")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return; 
    }
    
    const char* new_ssid = doc["ssid"];
    const char* new_pass = doc["pass"];

    if (!new_ssid || strlen(new_ssid) == 0) {
        sendJsonResponse(400, false, "SSID cannot be empty.");
        return;
    }
    if (strlen(new_ssid) > 32) {
        sendJsonResponse(400, false, "SSID too long (max 32 chars).");
        return;
    }

    if (new_pass && strlen(new_pass) > 0 && strlen(new_pass) < 8) {
        sendJsonResponse(400, false, "New password must be at least 8 characters.");
        return;
    }

    // <<< FIX: Protect NVS access
    portENTER_CRITICAL(&nvsMutex);
    preferences.begin("baitboat", false);
    preferences.putString("wifi_ssid", new_ssid);
    if (new_pass && strlen(new_pass) > 0) {
        preferences.putString("wifi_pass", new_pass);
    }
    preferences.end();
    portEXIT_CRITICAL(&nvsMutex);
    
    String msg = "Wi-Fi settings saved. Boat will reboot now.";
    sendJsonResponse(200, true, msg.c_str());
    delay(1000);
    ESP.restart();
  });

  server.on("/control_route", HTTP_POST, [](){
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("command")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing command."); return; 
    }

    String command = doc["command"];
    
    // <<< FIX: Protect writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    
    if (command == "start") {
        JsonArray waypoints = doc["waypoints"];
        if (waypoints.size() == 0) {
            portEXIT_CRITICAL(&boatStatusMutex);
            server.send(400, "text/plain", "Cannot start an empty route."); return;
        }
        if (waypoints.size() > MAX_ROUTE_WAYPOINTS) {
            portEXIT_CRITICAL(&boatStatusMutex);
            server.send(400, "text/plain", "Route exceeds maximum waypoint limit."); return;
        }

        currentRoute.waypointCount = waypoints.size();
        for(int i=0; i < currentRoute.waypointCount; i++) {
            currentRoute.waypoints[i] = waypoints[i].as<int>();
        }
        currentRoute.currentStep = 0;
        currentRoute.status = ROUTE_ACTIVE;
        portEXIT_CRITICAL(&boatStatusMutex);
        server.send(200, "text/plain", "Route started!");
    } else if (command == "stop") {
        currentRoute.status = ROUTE_INACTIVE;
        boatStatus.autopilot.engaged = false;
        boatStatus.autopilot.rth_active = false;
        boatStatus.autopilot.target_waypoint_index = -1;
        portEXIT_CRITICAL(&boatStatusMutex); // Release before calling function
        voidMotors();
        server.send(200, "text/plain", "Route stopped!");
    } else if (command == "pause") {
        if (currentRoute.status == ROUTE_ACTIVE) {
            currentRoute.status = ROUTE_PAUSED;
        }
        portEXIT_CRITICAL(&boatStatusMutex);
        server.send(200, "text/plain", "Route paused!");
    } else if (command == "resume") {
        if (currentRoute.status == ROUTE_PAUSED) {
            currentRoute.status = ROUTE_ACTIVE;
        }
        portEXIT_CRITICAL(&boatStatusMutex);
        server.send(200, "text/plain", "Route resumed!");
    } else {
        portEXIT_CRITICAL(&boatStatusMutex);
    }
  });

  server.on("/save_location_name", HTTP_POST, [](){
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("index") || !doc.containsKey("name")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return; 
    }

    int locIndex = doc["index"];
    String newName = doc["name"];
    if (locIndex < 1 || locIndex >= 5 || newName.length() == 0) {
        sendJsonResponse(400, false, "Invalid location index or empty name.");
        return;
    }
    
    // <<< FIX: Protect writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    savedLocations[locIndex].name = newName;
    flagSettingsChanged();
    portEXIT_CRITICAL(&boatStatusMutex);
    
    sendJsonResponse(200, true, "Location name updated. Will be saved shortly.");
  });

  server.on("/set_location_actions", HTTP_POST, [](){
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("locationIndex")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing index."); return; 
    }

    int locIndex = doc["locationIndex"];

    // <<< FIX: Protect reads/writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    if (locIndex < 1 || locIndex >= 5 || !savedLocations[locIndex].isSet) {
        portEXIT_CRITICAL(&boatStatusMutex);
        sendJsonResponse(400, false, "Invalid location or location not set.");
        return;
    }
    savedLocations[locIndex].dropLeftHopper = doc["dropLeftHopper"];
    savedLocations[locIndex].dropRightHopper = doc["dropRightHopper"];
    savedLocations[locIndex].releaseLeftHook = doc["releaseLeftHook"];
    savedLocations[locIndex].releaseRightHook = doc["releaseRightHook"];
    savedLocations[locIndex].autoReturnToHome = doc["autoReturnToHome"];
    flagSettingsChanged();
    portEXIT_CRITICAL(&boatStatusMutex);
    
    sendJsonResponse(200, true, "Location actions updated. Will be saved shortly.");
  });

  server.on("/delete_location", HTTP_POST, [](){
    StaticJsonDocument<64> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if(error || !doc.containsKey("locationIndex")) { 
      sendJsonResponse(400, false, "Invalid JSON or missing index."); return; 
    }

    int locIndex = doc["locationIndex"];
    if (locIndex < 1 || locIndex >= 5) {
        server.send(400, "text/plain", "Invalid index");
        return;
    }
    
    // <<< FIX: Protect NVS access
    portENTER_CRITICAL(&nvsMutex);
    preferences.begin("baitboat", false);
    preferences.remove(("loc" + String(locIndex) + "Lat").c_str());
    preferences.remove(("loc" + String(locIndex) + "Lng").c_str());
    preferences.putBool(("loc" + String(locIndex) + "Set").c_str(), false);
    preferences.putString(("loc" + String(locIndex) + "Name").c_str(), "Waypoint " + String(locIndex));
    preferences.putBool(("loc" + String(locIndex) + "DLH").c_str(), false);
    preferences.putBool(("loc" + String(locIndex) + "DRH").c_str(), false);
    preferences.putBool(("loc" + String(locIndex) + "RLH").c_str(), false);
    preferences.putBool(("loc" + String(locIndex) + "RRH").c_str(), false);
    preferences.putBool(("loc" + String(locIndex) + "RTH").c_str(), false);
    preferences.end();
    portEXIT_CRITICAL(&nvsMutex);

    // <<< FIX: Protect writes to shared data
    portENTER_CRITICAL(&boatStatusMutex);
    savedLocations[locIndex] = {0.0, 0.0, false, "Waypoint " + String(locIndex), false, false, false, false, false};
    portEXIT_CRITICAL(&boatStatusMutex);
    
    server.send(200, "text/plain", "Location deleted");
  });

  server.on("/backup_settings", HTTP_GET, [](){
      // <<< FIX: Protect NVS access
      portENTER_CRITICAL(&nvsMutex);
      preferences.begin("baitboat", true);
      StaticJsonDocument<4096> doc;
      
      doc["pid_p"] = preferences.getFloat("pid_p", 4.0);
      doc["pid_i"] = preferences.getFloat("pid_i", 0.1);
      doc["pid_d"] = preferences.getFloat("pid_d", 0.5);
      doc["low_batt"] = preferences.getFloat("low_batt", 10.5);
      doc["lb_rth_en"] = preferences.getBool("lb_rth_en", false);
      
      const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
      JsonObject alerts = doc.createNestedObject("alerts");
      for(int i = 0; i < NUM_ALERT_TYPES; i++){
          JsonObject alert_obj = alerts.createNestedObject(keys[i]);
          char key_buffer[25];
          snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
          alert_obj["b"] = preferences.getInt(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
          alert_obj["bd"] = preferences.getULong(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
          alert_obj["pd"] = preferences.getULong(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
          alert_obj["f"] = preferences.getInt(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
          alert_obj["fd"] = preferences.getULong(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
          alert_obj["fo"] = preferences.getULong(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
          alert_obj["fm"] = preferences.getUChar(key_buffer);
      }

      JsonArray locations = doc.createNestedArray("locations");
      for(int i=1; i<5; i++) {
          JsonObject loc = locations.createNestedObject();
          char key_buffer[25];
          snprintf(key_buffer, sizeof(key_buffer), "loc%dName", i);
          loc["name"] = preferences.getString(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dDLH", i);
          loc["dlh"] = preferences.getBool(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dDRH", i);
          loc["drh"] = preferences.getBool(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dRLH", i);
          loc["rlh"] = preferences.getBool(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dRRH", i);
          loc["rrh"] = preferences.getBool(key_buffer);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dRTH", i);
          loc["rth"] = preferences.getBool(key_buffer);
      }
      
      preferences.end();
      portEXIT_CRITICAL(&nvsMutex);
      
      String response;
      serializeJson(doc, response);
      server.sendHeader("Content-Disposition", "attachment; filename=baitboat_backup.json");
      server.send(200, "application/json", response);
  });
  
  server.on("/restore_settings", HTTP_POST, []() {
    if (restoreUploadStatus == RESTORE_SUCCESS) {
      String successPage = "<html><head><meta http-equiv='refresh' content='5;url=/'></head><body><h2>Restore Successful!</h2><p>Settings have been restored. The boat is now rebooting.</p><p>Please reconnect to the BaitBoat Wi-Fi network in a few moments.</p></body></html>";
      server.send(200, "text/html", successPage);
      delay(1000);
      ESP.restart();
    } else { 
      String errorPage = "<html><body><h2>Restore Failed!</h2><p>The uploaded file was invalid or corrupted. Please try again with a valid backup file.</p><a href='/'>Go Back</a></body></html>";
      server.send(400, "text/html", errorPage);
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
    StaticJsonDocument<4096> doc;
    DeserializationError error = deserializeJson(doc, upload.buf, upload.currentSize);

    if (error) {
      restoreUploadStatus = RESTORE_FAILED;
      return;
    }
    
    // Validate the restored settings before applying them
    if (!doc.containsKey("pid_p") || !doc.containsKey("pid_i") || !doc.containsKey("pid_d") ||
        !doc.containsKey("low_batt") || !doc.containsKey("lb_rth_en") || !doc.containsKey("alerts") || !doc.containsKey("locations")) {
      restoreUploadStatus = RESTORE_FAILED;
      return;
    }

    // <<< FIX: Protect NVS access
    portENTER_CRITICAL(&nvsMutex);
    preferences.begin("baitboat", false);
    
    preferences.putFloat("pid_p", doc["pid_p"]);
    preferences.putFloat("pid_i", doc["pid_i"]);
    preferences.putFloat("pid_d", doc["pid_d"]);
    preferences.putFloat("low_batt", doc["low_batt"]);
    preferences.putBool("lb_rth_en", doc["lb_rth_en"]);

    const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
    JsonObject alerts = doc["alerts"];
    for(int i = 0; i < NUM_ALERT_TYPES; i++){
        JsonObject alert_obj = alerts[keys[i]];
        char key_buffer[25];
        snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
        preferences.putInt(key_buffer, alert_obj["b"]);
        snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
        preferences.putULong(key_buffer, alert_obj["bd"]);
        snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
        preferences.putULong(key_buffer, alert_obj["pd"]);
        snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
        preferences.putInt(key_buffer, alert_obj["f"]);
        snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
        preferences.putULong(key_buffer, alert_obj["fd"]);
        snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
        preferences.putULong(key_buffer, alert_obj["fo"]);
        snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
        preferences.putUChar(key_buffer, alert_obj["fm"]);
    }
    
    JsonArray locations = doc["locations"];
    for(int i=0; i<locations.size(); i++) {
        JsonObject loc = locations[i];
        int locIndex = i + 1;
        char key_buffer[25];
        snprintf(key_buffer, sizeof(key_buffer), "loc%dName", locIndex);
        preferences.putString(key_buffer, loc["name"].as<const char*>());
        snprintf(key_buffer, sizeof(key_buffer), "loc%dDLH", locIndex);
        preferences.putBool(key_buffer, loc["dlh"]);
        snprintf(key_buffer, sizeof(key_buffer), "loc%dDRH", locIndex);
        preferences.putBool(key_buffer, loc["drh"]);
        snprintf(key_buffer, sizeof(key_buffer), "loc%dRLH", locIndex);
        preferences.putBool(key_buffer, loc["rlh"]);
        snprintf(key_buffer, sizeof(key_buffer), "loc%dRRH", locIndex);
        preferences.putBool(key_buffer, loc["rrh"]);
        snprintf(key_buffer, sizeof(key_buffer), "loc%dRTH", locIndex);
        preferences.putBool(key_buffer, loc["rth"]);
    }

    preferences.end();
    portEXIT_CRITICAL(&nvsMutex);
    
    restoreUploadStatus = RESTORE_SUCCESS;
  }
}