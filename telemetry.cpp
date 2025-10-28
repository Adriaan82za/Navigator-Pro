/*
 * Project: Bait Boat Control System (ESP32) - Telemetry Module
 * Description: Handles the web server and provides a web-based UI for
 * monitoring and configuration.
 * Author: [Adriaan v.d.Westhuizen] & Gemini
 * Date: October 28, 2025
 * Version: 12.1.5 (Modularized - Fixed Includes)
 */

#include "telemetry.h"
#include "autopilot.h"
#include <WebServer.h> // <-- ADDED THIS LINE
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
void handleRestoreUpload(); // Forward declaration for the upload handler

void setupWebServerRoutes() {
  if (MDNS.begin("baitboat")) {
    MDNS.addService("http", "tcp", 80);
  }

  // API endpoint for all boat telemetry data
  server.on("/telemetry", HTTP_GET, [](){ // Added HTTP_GET for clarity
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
      // Ensure index is valid before accessing
      if (boatStatus.autopilot.target_waypoint_index >= 0 && boatStatus.autopilot.target_waypoint_index < 5) {
        doc["waypoint"] = savedLocations[boatStatus.autopilot.target_waypoint_index].name;
      } else {
        doc["waypoint"] = "INVALID"; // Should not happen, but good practice
      }
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
    // Allow zero beeps/flashes, but not zero duration if there are beeps/flashes
    if (b < 0 || f < 0 || fm < 0 || (b > 0 && bd == 0) || (b > 1 && pd == 0) || (f > 0 && fd == 0) || (f > 1 && fo == 0) ) {
      sendJsonResponse(400, false, "Invalid alert values."); return;
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
    if (new_pass && strlen(new_pass) > 63) { // Max WiFi password length
        sendJsonResponse(400, false, "Password too long (max 63 chars).");
        return;
    }

    // <<< FIX: Protect NVS access
    portENTER_CRITICAL(&nvsMutex);
    preferences.begin("baitboat", false);
    preferences.putString("wifi_ssid", new_ssid);
    if (new_pass && strlen(new_pass) > 0) {
        preferences.putString("wifi_pass", new_pass);
    } else if (new_pass && strlen(new_pass) == 0) {
        // If password field is explicitly empty, clear the saved password
        preferences.remove("wifi_pass");
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
            sendJsonResponse(400, false, "Cannot start an empty route."); return;
        }
        if (waypoints.size() > MAX_ROUTE_WAYPOINTS) {
            portEXIT_CRITICAL(&boatStatusMutex);
            sendJsonResponse(400, false, "Route exceeds maximum waypoint limit."); return;
        }

        currentRoute.waypointCount = waypoints.size();
        for(int i=0; i < currentRoute.waypointCount; i++) {
            currentRoute.waypoints[i] = waypoints[i].as<int>();
        }
        currentRoute.currentStep = 0;
        currentRoute.status = ROUTE_ACTIVE;
        portEXIT_CRITICAL(&boatStatusMutex);
        sendJsonResponse(200, true, "Route started!");
    } else if (command == "stop") {
        currentRoute.status = ROUTE_INACTIVE;
        boatStatus.autopilot.engaged = false;
        boatStatus.autopilot.rth_active = false;
        boatStatus.autopilot.target_waypoint_index = -1;
        boatStatus.autopilot.arrival_state = AP_IDLE; // Reset arrival state
        portEXIT_CRITICAL(&boatStatusMutex); // Release before calling function
        voidMotors();
        sendJsonResponse(200, true, "Route stopped!");
    } else if (command == "pause") {
        if (currentRoute.status == ROUTE_ACTIVE) {
            currentRoute.status = ROUTE_PAUSED;
            // Optionally void motors when pausing
            // voidMotors();
        }
        portEXIT_CRITICAL(&boatStatusMutex);
        sendJsonResponse(200, true, "Route paused!");
    } else if (command == "resume") {
        if (currentRoute.status == ROUTE_PAUSED) {
            currentRoute.status = ROUTE_ACTIVE;
        }
        portEXIT_CRITICAL(&boatStatusMutex);
        sendJsonResponse(200, true, "Route resumed!");
    } else {
        portEXIT_CRITICAL(&boatStatusMutex);
        sendJsonResponse(400, false, "Unknown route command.");
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
    if (locIndex < 1 || locIndex >= 5 || newName.length() == 0 || newName.length() > 32) { // Added length check
        sendJsonResponse(400, false, "Invalid index, empty name, or name too long (max 32).");
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
    if(error || !doc.containsKey("locationIndex") ||
       !doc.containsKey("dropLeftHopper") || !doc.containsKey("dropRightHopper") ||
       !doc.containsKey("releaseLeftHook") || !doc.containsKey("releaseRightHook") ||
       !doc.containsKey("autoReturnToHome") ) {
      sendJsonResponse(400, false, "Invalid JSON or missing keys."); return;
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
        sendJsonResponse(400, false, "Invalid location index.");
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

    sendJsonResponse(200, true, "Location deleted successfully.");
  });

  server.on("/backup_settings", HTTP_GET, [](){
      // <<< FIX: Protect NVS access
      portENTER_CRITICAL(&nvsMutex);
      preferences.begin("baitboat", true); // Read-only access needed here
      StaticJsonDocument<4096> doc;

      // Use defaults that match initAutopilot and loadAllSettings if key not found
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

          // Provide default values similar to loadAllSettings
          bool is_lb = (i == AlertType::ALERT_LOW_BATTERY);
          bool is_wr = (i == AlertType::ALERT_WIFI_RESET);
          bool is_armed = (i == AlertType::ALERT_ARMED);

          snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
          alert_obj["b"] = preferences.getInt(key_buffer, is_lb ? 10000 : (is_wr ? 3 : (is_armed ? 2 : (i==AlertType::ALERT_HOME_SAVED?4:1))));
          snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
          alert_obj["bd"] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 250 : (is_armed ? 80 : 150)));
          snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
          alert_obj["pd"] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 150 : (is_armed ? 80 : 100)));
          snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
          alert_obj["f"] = preferences.getInt(key_buffer, is_lb ? 10000 : (is_wr ? 3 : (is_armed ? 2 : (i<4?3:7))));
          snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
          alert_obj["fd"] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 250 : (is_armed ? 80 : 150)));
          snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
          alert_obj["fo"] = preferences.getULong(key_buffer, is_lb ? 500 : (is_wr ? 150 : (is_armed ? 80 : 150)));
          snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
          alert_obj["fm"] = preferences.getUChar(key_buffer, ALL_LIGHTS_MASK);
      }

      JsonArray locations = doc.createNestedArray("locations");
      for(int i=1; i<5; i++) { // Only backup waypoints 1-4
          JsonObject loc = locations.createNestedObject();
          char key_buffer[25];
          snprintf(key_buffer, sizeof(key_buffer), "loc%dName", i);
          loc["name"] = preferences.getString(key_buffer, "Waypoint " + String(i));
          snprintf(key_buffer, sizeof(key_buffer), "loc%dDLH", i);
          loc["dlh"] = preferences.getBool(key_buffer, false);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dDRH", i);
          loc["drh"] = preferences.getBool(key_buffer, false);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dRLH", i);
          loc["rlh"] = preferences.getBool(key_buffer, false);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dRRH", i);
          loc["rrh"] = preferences.getBool(key_buffer, false);
          snprintf(key_buffer, sizeof(key_buffer), "loc%dRTH", i);
          loc["rth"] = preferences.getBool(key_buffer, false);
          // Don't back up lat/lng/set status - only name and actions
      }

      preferences.end();
      portEXIT_CRITICAL(&nvsMutex);

      String response;
      serializeJson(doc, response);
      server.sendHeader("Content-Disposition", "attachment; filename=baitboat_backup.json");
      server.send(200, "application/json", response);
  });

  // Handle the file upload (intermediate step)
  server.on("/restore_settings", HTTP_POST, []() {
      // This is the final response handler after upload is complete
      server.sendHeader("Connection", "close");
      if (restoreUploadStatus == RESTORE_SUCCESS) {
        String successPage = R"rawliteral(
          <html>
            <head>
              <title>Restore Success</title>
              <meta http-equiv='refresh' content='5;url=/'/>
              <style>body{font-family:sans-serif; padding: 20px; background-color: #f0f0f0;} h2{color: #4CAF50;} p{color: #333;}</style>
            </head>
            <body>
              <h2>✅ Restore Successful!</h2>
              <p>Settings have been restored from the backup file.</p>
              <p><strong>The boat is now rebooting.</strong> Please wait a few moments and reconnect to its Wi-Fi network.</p>
              <p>(You will be redirected back automatically in 5 seconds...)</p>
            </body>
          </html>
        )rawliteral";
        server.send(200, "text/html", successPage);
        delay(1000); // Give time for the response to send
        ESP.restart(); // Reboot after successful restore
      } else {
        String errorPage = R"rawliteral(
          <html>
            <head><title>Restore Failed</title>
            <style>body{font-family:sans-serif; padding: 20px; background-color: #f0f0f0;} h2{color: #f44336;} p{color: #333;} a{color:#007bff; text-decoration:none;}</style>
            </head>
            <body>
              <h2>❌ Restore Failed!</h2>
              <p>The uploaded file was invalid, corrupted, or not a compatible backup file.</p>
              <p>Please ensure you are uploading a valid JSON backup created by this boat.</p>
              <p><a href='/'>Click here to go back</a></p>
            </body>
          </html>
        )rawliteral";
        server.send(400, "text/html", errorPage);
      }
      restoreUploadStatus = RESTORE_IDLE; // Reset status
  }, handleRestoreUpload); // Link the upload handler function

  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "404: Not Found");
    }
  });

  server.begin();
}

// ==============================
// Restore Settings Upload Handler
// ==============================
void handleRestoreUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    restoreUploadStatus = RESTORE_IDLE; // Reset status on new upload
    // Optional: Check filename/type here if needed
    // String filename = upload.filename;
    // if (!filename.endsWith(".json")) { server.send(400, ...); return; }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // This part is handled internally by the WebServer library
  } else if (upload.status == UPLOAD_FILE_END) {
    // File reception is complete, now process the buffer
    StaticJsonDocument<4096> doc; // Use a sufficiently large buffer
    DeserializationError error = deserializeJson(doc, upload.buf, upload.currentSize);

    if (error) {
      Serial.printf("Restore JSON parsing failed: %s\n", error.c_str());
      restoreUploadStatus = RESTORE_FAILED;
      return; // Stop processing
    }

    // Basic validation of the JSON structure
    if (!doc.containsKey("pid_p") || !doc.containsKey("pid_i") || !doc.containsKey("pid_d") ||
        !doc.containsKey("low_batt") || !doc.containsKey("lb_rth_en") || !doc["alerts"].is<JsonObject>() || !doc["locations"].is<JsonArray>()) {
      Serial.println("Restore JSON missing required keys.");
      restoreUploadStatus = RESTORE_FAILED;
      return;
    }

    // If validation passes, proceed to write to NVS
    // <<< FIX: Protect NVS access
    portENTER_CRITICAL(&nvsMutex);
    preferences.begin("baitboat", false); // Open for writing

    // Apply settings from JSON
    preferences.putFloat("pid_p", doc["pid_p"].as<float>());
    preferences.putFloat("pid_i", doc["pid_i"].as<float>());
    preferences.putFloat("pid_d", doc["pid_d"].as<float>());
    preferences.putFloat("low_batt", doc["low_batt"].as<float>());
    preferences.putBool("lb_rth_en", doc["lb_rth_en"].as<bool>());

    JsonObject alerts = doc["alerts"].as<JsonObject>();
    const char* keys[] = {"hs", "ws", "gf", "ae", "lh", "rh", "lk", "rk", "lb", "wr", "ar"};
    for(int i = 0; i < NUM_ALERT_TYPES; i++){
        if (alerts.containsKey(keys[i])) {
            JsonObject alert_obj = alerts[keys[i]].as<JsonObject>();
            char key_buffer[25];
            snprintf(key_buffer, sizeof(key_buffer), "%s_b", keys[i]);
            preferences.putInt(key_buffer, alert_obj["b"].as<int>());
            snprintf(key_buffer, sizeof(key_buffer), "%s_bd", keys[i]);
            preferences.putULong(key_buffer, alert_obj["bd"].as<unsigned long>());
            snprintf(key_buffer, sizeof(key_buffer), "%s_pd", keys[i]);
            preferences.putULong(key_buffer, alert_obj["pd"].as<unsigned long>());
            snprintf(key_buffer, sizeof(key_buffer), "%s_f", keys[i]);
            preferences.putInt(key_buffer, alert_obj["f"].as<int>());
            snprintf(key_buffer, sizeof(key_buffer), "%s_fd", keys[i]);
            preferences.putULong(key_buffer, alert_obj["fd"].as<unsigned long>());
            snprintf(key_buffer, sizeof(key_buffer), "%s_fo", keys[i]);
            preferences.putULong(key_buffer, alert_obj["fo"].as<unsigned long>());
            snprintf(key_buffer, sizeof(key_buffer), "%s_fm", keys[i]);
            preferences.putUChar(key_buffer, alert_obj["fm"].as<byte>());
        }
    }

    JsonArray locations = doc["locations"].as<JsonArray>();
    for(int i=0; i<locations.size(); i++) { // Loop through locations in the backup
        JsonObject loc = locations[i].as<JsonObject>();
        int locIndex = i + 1; // Assuming backup array matches indices 1-4
        if (locIndex < 1 || locIndex >= 5) continue; // Skip if index is out of bounds

        char key_buffer[25];
        if (loc.containsKey("name")) {
            snprintf(key_buffer, sizeof(key_buffer), "loc%dName", locIndex);
            preferences.putString(key_buffer, loc["name"].as<const char*>());
        }
        if (loc.containsKey("dlh")) {
            snprintf(key_buffer, sizeof(key_buffer), "loc%dDLH", locIndex);
            preferences.putBool(key_buffer, loc["dlh"].as<bool>());
        }
        if (loc.containsKey("drh")) {
            snprintf(key_buffer, sizeof(key_buffer), "loc%dDRH", locIndex);
            preferences.putBool(key_buffer, loc["drh"].as<bool>());
        }
        if (loc.containsKey("rlh")) {
            snprintf(key_buffer, sizeof(key_buffer), "loc%dRLH", locIndex);
            preferences.putBool(key_buffer, loc["rlh"].as<bool>());
        }
         if (loc.containsKey("rrh")) {
            snprintf(key_buffer, sizeof(key_buffer), "loc%dRRH", locIndex);
            preferences.putBool(key_buffer, loc["rrh"].as<bool>());
        }
         if (loc.containsKey("rth")) {
            snprintf(key_buffer, sizeof(key_buffer), "loc%dRTH", locIndex);
            preferences.putBool(key_buffer, loc["rth"].as<bool>());
        }
        // NOTE: We deliberately DO NOT restore lat/lng/set from the backup.
        // Actions and names are restored.
    }

    preferences.end();
    portEXIT_CRITICAL(&nvsMutex);

    Serial.println("Restore successful, settings applied to NVS.");
    restoreUploadStatus = RESTORE_SUCCESS; // Flag success for the final response handler

  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Serial.println("Restore upload aborted.");
    restoreUploadStatus = RESTORE_FAILED;
  }
}