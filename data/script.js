var currentRoute = [];
var isEditingInput = false;
var telemetryData = {};
var currentCompassHeading = 0;
var voiceAlertsEnabled = false;
var lastReportedMode = "";

// ==========================================
// Native App Tab Control
// ==========================================
function showTab(tabName) {
  document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
  document.querySelectorAll('.nav-item').forEach(link => link.classList.remove('active'));
  document.getElementById(tabName).classList.add('active');
  event.currentTarget.classList.add('active');
}

// ==========================================
// Haptic and Voice Alert System
// ==========================================
function triggerAlert(message, type = 'info') {
    showMessageBox(message, type);
    
    if (!voiceAlertsEnabled) return;

    if (navigator.vibrate) {
        navigator.vibrate(type === 'success' ? [100, 50, 100] : [200]);
    }

    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
        let msg = new SpeechSynthesisUtterance(message);
        msg.rate = 1.0;
        msg.pitch = 1.0;
        window.speechSynthesis.speak(msg);
    }
}

// ==========================================
// Offline Radar Plotter (HTML5 Canvas)
// ==========================================
function drawRadar(data) {
    const canvas = document.getElementById('radarCanvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    const center = width / 2;

    ctx.clearRect(0, 0, width, height);

    ctx.strokeStyle = '#166534'; 
    ctx.lineWidth = 1;
    for (let i = 1; i <= 3; i++) {
        ctx.beginPath();
        ctx.arc(center, center, (center / 3) * i, 0, 2 * Math.PI);
        ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(center, 0); ctx.lineTo(center, height);
    ctx.moveTo(0, center); ctx.lineTo(width, center);
    ctx.stroke();

    let distFromHome = parseFloat(data.distance) || 0;
    let maxRange = 50; 
    if (distFromHome > 35) maxRange = 100;
    if (distFromHome > 80) maxRange = 250;
    if (distFromHome > 200) maxRange = 500;
    document.getElementById('radar_range_text').innerText = `Range: ${maxRange}m`;

    const scale = (center - 10) / maxRange; 

    if (data.locations && data.locations[0] && data.locations[0].set) {
        ctx.fillStyle = '#22c55e'; 
        ctx.beginPath();
        ctx.arc(center, center, 4, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillStyle = '#4ade80';
        ctx.font = '10px monospace';
        ctx.fillText("H", center + 6, center - 6);
    }

    if (data.latitude !== "N/A" && data.locations && data.locations[0] && data.locations[0].set) {
        const latHome = parseFloat(data.locations[0].lat);
        const lonHome = parseFloat(data.locations[0].lng);
        const latBoat = parseFloat(data.latitude);
        const lonBoat = parseFloat(data.longitude);
        
        const mPerLat = 111320; 
        const mPerLon = 111320 * Math.cos(latHome * Math.PI / 180);
        
        const dx = (lonBoat - lonHome) * mPerLon;
        const dy = (latBoat - latHome) * mPerLat; 

        const canvasX = center + (dx * scale);
        const canvasY = center - (dy * scale);

        ctx.save();
        ctx.translate(canvasX, canvasY);
        let rads = (parseFloat(data.heading) - 90) * (Math.PI / 180);
        ctx.rotate(rads);
        
        ctx.fillStyle = '#3b82f6'; 
        ctx.beginPath();
        ctx.moveTo(8, 0);   
        ctx.lineTo(-6, -5); 
        ctx.lineTo(-3, 0);  
        ctx.lineTo(-6, 5);  
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }
}

// ==========================================
// Telemetry Parsing (WebSockets / HTTP)
// ==========================================
function parseTelemetry(data) {
    telemetryData = data;
    
    if (data.mode !== lastReportedMode && lastReportedMode !== "") {
        triggerAlert(`Mode changed to ${data.mode}`, 'info');
    }
    lastReportedMode = data.mode;

    document.getElementById('mode').innerText = data.mode;
    document.getElementById('armed_status').innerText = data.armed ? "YES" : "NO";
    document.getElementById('armed_status').style.color = data.armed ? 'var(--success-color)' : 'var(--error-color)';
    document.getElementById('autopilot').innerText = data.autopilot_status;
    document.getElementById('rth').innerText = data.rth;
    document.getElementById('waypoint').innerText = data.waypoint;

    const newHeading = parseFloat(data.heading);
    let diff = newHeading - (currentCompassHeading % 360);
    if (diff < -180) { diff += 360; } else if (diff > 180) { diff -= 360; }
    currentCompassHeading += diff;

    document.querySelector('.compass-needle').style.transform = `rotate(${currentCompassHeading}deg)`;
    const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
    document.getElementById('heading_display').innerText = isNaN(newHeading) ? "--" : `${directions[Math.round(newHeading / 45) % 8]} (${newHeading.toFixed(1)}°)`;

    const pitchDegrees = parseFloat(data.pitch);
    const rollDegrees = parseFloat(data.roll);
    document.getElementById('horizon').style.top = `calc(50% + ${2 * Math.max(-45, Math.min(45, pitchDegrees))}px)`;
    document.getElementById('roll_indicator').style.transform = `rotate(${-rollDegrees}deg)`;
    document.getElementById('attitude_display').innerText = `P: ${pitchDegrees.toFixed(1)}° R: ${rollDegrees.toFixed(1)}°`;

    if (!isEditingInput) {
        document.getElementById('pid_p').value = data.pid.p;
        document.getElementById('pid_i').value = data.pid.i;
        document.getElementById('pid_d').value = data.pid.d;
        document.getElementById('low_batt_v').value = data.low_batt;
        document.getElementById('low_batt_rth_enable').checked = data.low_batt_rth;
        
        if(document.getElementById('wifi_ssid')) {
            document.getElementById('wifi_ssid').value = data.wifi_ssid;
        }
    }

    document.getElementById('battery').innerText = data.battery;
    document.getElementById('gps_fix').innerText = data.gps_fix;

    // --- NEW: SBAS/WAAS UI LOGIC ---
    if (data.hasOwnProperty('sbas_active')) {
        const sbasElement = document.getElementById('sbas_status');
        if (data.sbas_active) {
            sbasElement.innerText = "YES (Sub-Meter)";
            sbasElement.style.color = "#4ade80"; // Turn bright green on lock
        } else {
            sbasElement.innerText = "NO (Standard)";
            sbasElement.style.color = "var(--text-light)"; // Normal gray text when searching
        }
    }

    document.getElementById('satellites').innerText = data.satellites;
    document.getElementById('latitude').innerText = data.latitude;
    document.getElementById('longitude').innerText = data.longitude;
    document.getElementById('speed').innerText = data.speed === "N/A" ? "N/A" : `${data.speed} m/s`;
    document.getElementById('distance').innerText = data.distance;
    document.getElementById('route_status').innerText = data.route.status;
    
    if (document.getElementById('gps_time')) {
        document.getElementById('gps_time').innerText = data.gps_time || "-";
    }
    
    if (document.getElementById('declination')) {
        document.getElementById('declination').innerText = data.declination + '°';
    }

    drawRadar(data);

    const locationsContainer = document.getElementById('locations_container');
    const waypointSelect = document.getElementById('waypoint_select');
    let currentWpHTML = waypointSelect.innerHTML;
    let availableWaypoints = '';
    locationsContainer.innerHTML = '';

    for (var i = 0; i < data.locations.length; i++) {
      const locData = data.locations[i];
      if (i > 0 && locData.set) availableWaypoints += `<option value="${i}">${locData.name}</option>`;

      let html = `<div class="location-item"><h3>${locData.name}</h3>`;
      if(locData.set) {
        html += `<p>Lat: ${locData.lat}, Lng: ${locData.lng} ${i>0 && locData.dist_from_home && locData.dist_from_home !== "N/A" ? '| Dist: '+locData.dist_from_home+'m' : ''}</p>`;
        if(i > 0) {
          html += `<div class="name-input-group"><input type="text" class="form-group input" id="name_${i}" value="${locData.name}"><button class="btn btn-secondary" onclick="saveLocationName(${i})">Save</button></div>
          <div class="location-actions">
            <label><input type="checkbox" id="dlh_${i}" ${locData.dropLeftHopper ? 'checked' : ''}> Left Hopper</label>
            <label><input type="checkbox" id="drh_${i}" ${locData.dropRightHopper ? 'checked' : ''}> Right Hopper</label>
            <label><input type="checkbox" id="rlh_${i}" ${locData.releaseLeftHook ? 'checked' : ''}> Left Hook</label>
            <label><input type="checkbox" id="rrh_${i}" ${locData.releaseRightHook ? 'checked' : ''}> Right Hook</label>
            <label><input type="checkbox" id="rth_${i}" ${locData.autoReturnToHome ? 'checked' : ''}> Auto RTH</label>
          </div>
          <div class="button-group"><button class="btn btn-primary" onclick="saveLocationActions(${i})">Save Actions</button><button class="btn btn-danger" onclick="deleteLocation(${i})">Delete</button></div>`;
        }
      } else { html += `<p class="location-unset">Not Set</p>`; }
      locationsContainer.innerHTML += html + `</div>`;
    }
    if (currentWpHTML !== availableWaypoints) waypointSelect.innerHTML = availableWaypoints;

    const alertSelect = document.getElementById('alert_select');
    if(alertSelect && alertSelect.options.length === 0 && data.alerts) {
        const alertNames = data.alerts;
        for(const key in alertNames) {
            let option = document.createElement('option');
            option.value = key;
            option.innerText = alertNames[key].name;
            alertSelect.appendChild(option);
        }
        populateAlertFields();
    }
}

function updateTelemetryHTTP() {
    if(isEditingInput) return;
    fetch('/telemetry').then(r => r.json()).then(parseTelemetry).catch(e => console.log('HTTP Fetch Error'));
}

// ==========================================
// Initialization
// ==========================================
window.onload = () => {
    const themeSwitch = document.getElementById('themeSwitch');
    function setTheme(isDark) {
        document.body.classList.toggle('dark-mode', isDark);
        themeSwitch.checked = isDark;
    }
    setTheme(localStorage.getItem('theme') === 'dark');
    themeSwitch.addEventListener('change', function() {
        localStorage.setItem('theme', this.checked ? 'dark' : 'light');
        setTheme(this.checked);
    });

    const voiceSwitch = document.getElementById('voiceSwitch');
    voiceSwitch.checked = localStorage.getItem('voice') === 'true';
    voiceAlertsEnabled = voiceSwitch.checked;
    voiceSwitch.addEventListener('change', function() {
        voiceAlertsEnabled = this.checked;
        localStorage.setItem('voice', this.checked);
        if (this.checked) triggerAlert("Voice alerts enabled", "success");
    });

    document.body.addEventListener('focusin', e => { if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') isEditingInput = true; });
    document.body.addEventListener('focusout', e => { if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') isEditingInput = false; });

    let ws = new WebSocket(`ws://${window.location.hostname}/ws`);
    ws.onmessage = (event) => { if(!isEditingInput) parseTelemetry(JSON.parse(event.data)); };
    ws.onerror = () => {
        console.log("WebSocket failed. Falling back to HTTP polling.");
        setInterval(updateTelemetryHTTP, 200); 
    };
    
    updateTelemetryHTTP();
};

// ==========================================
// Form Submissions and UI Actions
// ==========================================
function showMessageBox(message, type = 'success', callback = null) {
  const existingMessageBox = document.getElementById('custom-message-box');
  if (existingMessageBox) existingMessageBox.remove(); 
  const messageBox = document.createElement('div');
  messageBox.id = 'custom-message-box';
  let bgColor = type === 'success' ? '#22c55e' : type === 'error' ? '#ef4444' : '#334155';
  messageBox.style.cssText = `position: fixed; top: 20px; left: 50%; transform: translateX(-50%); background-color: ${bgColor}; color: white; padding: 15px 25px; border-radius: 30px; box-shadow: 0 4px 20px rgba(0,0,0,0.4); z-index: 10000; font-weight: 600; text-align: center;`;
  messageBox.innerHTML = `<p style="margin:0;">${message}</p>`;

  if (type === 'confirm') {
    messageBox.style.top = '50%'; messageBox.style.transform = 'translate(-50%, -50%)'; messageBox.style.borderRadius = '15px';
    messageBox.innerHTML += `<div style="margin-top:20px;"><button class="btn btn-primary" onclick="document.getElementById('custom-message-box').remove(); window.callback(true)">Yes</button> <button class="btn btn-danger" onclick="document.getElementById('custom-message-box').remove(); window.callback(false)">No</button></div>`;
    window.callback = callback;
  } else {
    setTimeout(() => { messageBox.remove(); }, 3000);
  }
  document.body.appendChild(messageBox);
}

function handleApiResponse(promise) {
    promise.then(response => response.json()).then(data => { showMessageBox(data.message, data.success ? 'success' : 'error'); }).catch(err => showMessageBox('Connection failed.', 'error'));
}

function populateAlertFields() {
    const selectedKey = document.getElementById('alert_select').value;
    if(!telemetryData.alerts || !telemetryData.alerts[selectedKey]) return;
    const alertData = telemetryData.alerts[selectedKey];
    document.getElementById('alert_b').value = alertData.b;
    document.getElementById('alert_bd').value = alertData.bd;
    document.getElementById('alert_pd').value = alertData.pd;
    document.getElementById('alert_f').value = alertData.f;
    document.getElementById('alert_fd').value = alertData.fd;
    document.getElementById('alert_fo').value = alertData.fo;
    document.querySelectorAll('.alert_lm').forEach(cb => { cb.checked = (alertData.fm & parseInt(cb.value)); });
}

function saveAlertSettings() {
    const key = document.getElementById('alert_select').value;
    let mask = 0;
    document.querySelectorAll('.alert_lm:checked').forEach(cb => { mask |= parseInt(cb.value); });
    const payload = {
        key: key, b: parseInt(document.getElementById('alert_b').value), bd: parseInt(document.getElementById('alert_bd').value), pd: parseInt(document.getElementById('alert_pd').value),
        f: parseInt(document.getElementById('alert_f').value), fd: parseInt(document.getElementById('alert_fd').value), fo: parseInt(document.getElementById('alert_fo').value), fm: mask
    };
    handleApiResponse(fetch('/save_alert_settings', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }));
}

function saveWifiSettings() {
    const ssid = document.getElementById('wifi_ssid').value;
    const pass = document.getElementById('wifi_pass').value;
    if (!ssid || ssid.length < 1) return showMessageBox('SSID cannot be empty.', 'error');
    if (pass && pass.length > 0 && pass.length < 8) return showMessageBox('Password must be at least 8 characters long.', 'error');
    handleApiResponse(fetch('/save_wifi', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ssid: ssid, pass: pass }) }));
}

function savePIDGains() { handleApiResponse(fetch('/save_pid', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ p: parseFloat(document.getElementById('pid_p').value), i: parseFloat(document.getElementById('pid_i').value), d: parseFloat(document.getElementById('pid_d').value) }) })); }
function saveSystemSettings() { handleApiResponse(fetch('/save_system_settings', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ low_batt: parseFloat(document.getElementById('low_batt_v').value), low_batt_rth: document.getElementById('low_batt_rth_enable').checked }) })); }
function addWaypointToRoute() {
    const sel = document.getElementById('waypoint_select');
    if (!sel.value) return showMessageBox('Select a waypoint.', 'error');
    if (currentRoute.length >= 10) return showMessageBox('Max route length reached.', 'error');
    currentRoute.push({index: parseInt(sel.value), name: sel.options[sel.selectedIndex].text});
    document.getElementById('route_display').innerText = currentRoute.map(wp => wp.name).join(' -> ');
}
function clearRoute() { currentRoute = []; document.getElementById('route_display').innerText = '-'; controlRoute('stop'); }
function controlRoute(command) { fetch('/control_route', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ command: command, waypoints: currentRoute.map(wp => wp.index) }) }).then(r => r.text()).then(data => showMessageBox(data)); }
function saveLocationName(i) { handleApiResponse(fetch('/save_location_name', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ index: i, name: document.getElementById(`name_${i}`).value }) })); }
function saveLocationActions(i) { handleApiResponse(fetch('/set_location_actions', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ locationIndex: i, dropLeftHopper: document.getElementById(`dlh_${i}`).checked, dropRightHopper: document.getElementById(`drh_${i}`).checked, releaseLeftHook: document.getElementById(`rlh_${i}`).checked, releaseRightHook: document.getElementById(`rrh_${i}`).checked, autoReturnToHome: document.getElementById(`rth_${i}`).checked }) })); }
function deleteLocation(i) { showMessageBox('Delete Waypoint?', 'confirm', res => { if(res) fetch('/delete_location', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ locationIndex: i }) }).then(r => r.text()).then(d => { showMessageBox(d); updateTelemetryHTTP(); }); }); }

// ==========================================
// Native WT901 Calibration Commands
// ==========================================
function startIMUCalibration() {
    fetch('/save_imu_cal', { 
        method: 'POST', 
        headers: { 'Content-Type': 'application/json' }, 
        body: JSON.stringify({ command: 'start' }) 
    })
    .then(r => r.json())
    .then(data => showMessageBox(data.message, data.success ? 'success' : 'error'));
}

function finishIMUCalibration() {
    fetch('/save_imu_cal', { 
        method: 'POST', 
        headers: { 'Content-Type': 'application/json' }, 
        body: JSON.stringify({ command: 'stop' }) 
    })
    .then(r => r.json())
    .then(data => showMessageBox(data.message, data.success ? 'success' : 'error'));
}