var currentRoute = [];
var isEditingInput = false;
var telemetryData = {};
var currentCompassHeading = 0;

function showTab(tabName) {
  document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
  document.querySelectorAll('.tab-link').forEach(link => link.classList.remove('active'));
  document.getElementById(tabName).classList.add('active');
  event.currentTarget.classList.add('active');
}

function getCardinalDirection(angle) {
    const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
    return directions[Math.round(angle / 45) % 8];
}

function showMessageBox(message, type = 'success', callback = null) {
  const existingMessageBox = document.getElementById('custom-message-box');
  if (existingMessageBox) { existingMessageBox.remove(); }

  const messageBox = document.createElement('div');
  messageBox.id = 'custom-message-box';

  let bgColor;
  switch(type) {
    case 'success': bgColor = 'var(--success-color)'; break;
    case 'error': bgColor = 'var(--error-color)'; break;
    case 'confirm': bgColor = '#333'; break;
    default: bgColor = '#333';
  }

  messageBox.style.cssText = `position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%); background-color: ${bgColor}; color: white; padding: 25px; border-radius: 8px; box-shadow: 0 4px 20px rgba(0,0,0,0.4); z-index: 10000; font-family: 'Inter', sans-serif; font-size: 1.1em; text-align: center; max-width: 90%; min-width: 250px;`;
  messageBox.innerHTML = `<p style="margin:0;">${message}</p>`;

  if (type === 'confirm') {
    const buttonContainer = document.createElement('div');
    buttonContainer.style.marginTop = '20px';
    const yesButton = document.createElement('button');
    yesButton.innerText = 'Yes';
    yesButton.className = 'btn btn-primary';
    yesButton.style.margin = '0 10px';
    yesButton.onclick = () => { messageBox.remove(); if (callback) callback(true); };
    const noButton = document.createElement('button');
    noButton.innerText = 'No';
    noButton.className = 'btn btn-danger';
    noButton.style.margin = '0 10px';
    noButton.onclick = () => { messageBox.remove(); if (callback) callback(false); };
    buttonContainer.appendChild(yesButton);
    buttonContainer.appendChild(noButton);
    messageBox.appendChild(buttonContainer);
  } else {
    setTimeout(() => { messageBox.remove(); }, 4000);
  }
  document.body.appendChild(messageBox);
}

function handleApiResponse(promise) {
    promise.then(response => response.json()).then(data => {
        showMessageBox(data.message, data.success ? 'success' : 'error');
    }).catch(err => {
        showMessageBox('Request failed. Could not reach the boat.', 'error');
    });
}

function updateTelemetry() {
  if(isEditingInput) return;
  fetch('/telemetry').then(response => response.json()).then(data => {
    telemetryData = data;
    document.getElementById('mode').innerText = data.mode;

    const armedStatusEl = document.getElementById('armed_status');
    if (data.armed) {
      armedStatusEl.innerText = "YES";
      armedStatusEl.style.color = 'var(--success-color)';
    } else {
      armedStatusEl.innerText = "NO";
      armedStatusEl.style.color = 'var(--error-color)';
    }

    document.getElementById('autopilot').innerText = data.autopilot_status;
    document.getElementById('rth').innerText = data.rth;
    document.getElementById('waypoint').innerText = data.waypoint;

    const newHeading = parseFloat(data.heading);
    const cardinal = getCardinalDirection(newHeading);

    let diff = newHeading - (currentCompassHeading % 360);
    if (diff < -180) {
        diff += 360;
    } else if (diff > 180) {
        diff -= 360;
    }

    currentCompassHeading += diff;

    document.querySelector('.compass-needle').style.transform = `rotate(${currentCompassHeading}deg)`;
    document.getElementById('heading_display').innerText = isNaN(newHeading) ? "--" : `${cardinal} (${newHeading.toFixed(1)}°)`;

    const pitchDegrees = parseFloat(data.pitch);
    const rollDegrees = parseFloat(data.roll);
    const horizon = document.getElementById('horizon');
    const rollIndicator = document.getElementById('roll_indicator');

    const pitchTranslation = 2 * Math.max(-45, Math.min(45, pitchDegrees));
    horizon.style.top = `calc(50% + ${pitchTranslation}px)`;

    rollIndicator.style.transform = `rotate(${-rollDegrees}deg)`;

    document.getElementById('attitude_display').innerText = `P: ${pitchDegrees.toFixed(1)}° R: ${rollDegrees.toFixed(1)}°`;

    if (!isEditingInput) {
        document.getElementById('pid_p').value = data.pid.p;
        document.getElementById('pid_i').value = data.pid.i;
        document.getElementById('pid_d').value = data.pid.d;
        document.getElementById('low_batt_v').value = data.low_batt;
        document.getElementById('low_batt_rth_enable').checked = data.low_batt_rth;
        document.getElementById('wifi_ssid').value = data.wifi_ssid;
    }

    document.getElementById('battery').innerText = data.battery;
    document.getElementById('gps_fix').innerText = data.gps_fix;
    document.getElementById('satellites').innerText = data.satellites;
    document.getElementById('latitude').innerText = data.latitude;
    document.getElementById('longitude').innerText = data.longitude;
    document.getElementById('speed').innerText = data.speed;
    document.getElementById('distance').innerText = data.distance;
    document.getElementById('declination').innerText = data.declination;

    const confidenceLevels = ['Unreliable', 'Low', 'Medium', 'High'];
    document.getElementById('imu_confidence').innerText = confidenceLevels[data.imu_confidence] || 'N/A';

    document.getElementById('route_status').innerText = data.route.status;

    const locationsContainer = document.getElementById('locations_container');
    const waypointSelect = document.getElementById('waypoint_select');
    let currentWpHTML = waypointSelect.innerHTML;
    let availableWaypoints = '';
    locationsContainer.innerHTML = '';

    for (var i = 0; i < data.locations.length; i++) {
      const locData = data.locations[i];
      const locationIndex = i;

      if (locationIndex > 0 && locData.set) {
          availableWaypoints += `<option value="${locationIndex}">${locData.name}</option>`;
      }

      let locDiv = document.createElement('div');
      locDiv.className = 'location-item';
      let html = `<h3>${locData.name}</h3>`;
      if(locData.set) {
        let details = `Lat: ${locData.lat}, Lng: ${locData.lng}`;
        if (i > 0 && locData.dist_from_home && locData.dist_from_home !== "N/A") {
            details += ` | Distance from Home: ${locData.dist_from_home}m`;
        }
        html += `<p>${details}</p>`;

        if(locationIndex > 0) {
          html += `<div class="name-input-group">
            <input type="text" class="form-group input" id="name_${locationIndex}" placeholder="Rename Waypoint" value="${locData.name}">
            <button class="btn btn-secondary" onclick="saveLocationName(${locationIndex})">Save Name</button>
          </div>`;
          html += `<div class="location-actions">
            <label><input type="checkbox" id="dlh_${locationIndex}" ${locData.dropLeftHopper ? 'checked' : ''}> Left Hopper</label>
            <label><input type="checkbox" id="drh_${locationIndex}" ${locData.dropRightHopper ? 'checked' : ''}> Right Hopper</label>
            <label><input type="checkbox" id="rlh_${locationIndex}" ${locData.releaseLeftHook ? 'checked' : ''}> Left Hook</label>
            <label><input type="checkbox" id="rrh_${locationIndex}" ${locData.releaseRightHook ? 'checked' : ''}> Right Hook</label>
            <label><input type="checkbox" id="rth_${locationIndex}" ${locData.autoReturnToHome ? 'checked' : ''}> Auto RTH</label>
          </div>
          <div class="button-group">
            <button class="btn btn-primary" onclick="saveLocationActions(${locationIndex})">Save Actions</button>
            <button class="btn btn-danger" onclick="deleteLocation(${locationIndex})">Delete</button>
          </div>`;
        }
      } else {
        html += `<p class="location-unset">Not Set</p>`;
      }
      locDiv.innerHTML = html;
      locationsContainer.appendChild(locDiv);
    }
    if (currentWpHTML !== availableWaypoints) {
        waypointSelect.innerHTML = availableWaypoints;
    }

    const alertSelect = document.getElementById('alert_select');
    if(alertSelect.options.length === 0 && data.alerts) {
        const alertNames = data.alerts;
        for(const key in alertNames) {
            let option = document.createElement('option');
            option.value = key;
            option.innerText = alertNames[key].name;
            alertSelect.appendChild(option);
        }
        populateAlertFields();
    }

  }).catch(error => console.error('Error fetching telemetry:', error));
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
    document.querySelectorAll('.alert_lm').forEach(cb => {
        cb.checked = (alertData.fm & parseInt(cb.value));
    });
}

function savePIDGains() {
  const payload = { p: parseFloat(document.getElementById('pid_p').value), i: parseFloat(document.getElementById('pid_i').value), d: parseFloat(document.getElementById('pid_d').value) };
  handleApiResponse(fetch('/save_pid', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }));
}

function saveSystemSettings() {
  const payload = {
    low_batt: parseFloat(document.getElementById('low_batt_v').value),
    low_batt_rth: document.getElementById('low_batt_rth_enable').checked
  };
  handleApiResponse(fetch('/save_system_settings', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }));
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
    if (!ssid || ssid.length < 1) { showMessageBox('SSID cannot be empty.', 'error'); return; }
    if (pass && pass.length > 0 && pass.length < 8) { showMessageBox('Password must be at least 8 characters long.', 'error'); return; }
    const payload = { ssid: ssid, pass: pass };
    handleApiResponse(fetch('/save_wifi', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }));
}

// Function removed: saveIMUCalibration() - As it's non-functional

function addWaypointToRoute() {
    const select = document.getElementById('waypoint_select');
    const wpIndex = select.value;
    if (!wpIndex) { showMessageBox('Please select a waypoint.', 'error'); return; }
    if (currentRoute.length >= 10) { showMessageBox('Maximum route length (10) reached.', 'error'); return; }
    const wpName = select.options[select.selectedIndex].text;
    currentRoute.push({index: parseInt(wpIndex), name: wpName});
    document.getElementById('route_display').innerText = currentRoute.map(wp => wp.name).join(' -> ');
}
function clearRoute() {
    currentRoute = [];
    document.getElementById('route_display').innerText = '-';
    controlRoute('stop');
}
function controlRoute(command) {
    const payload = { command: command, waypoints: currentRoute.map(wp => wp.index) };
    fetch('/control_route', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) })
        .then(response => response.text()).then(data => showMessageBox(data));
}
function saveLocationName(locationIndex) {
    const name = document.getElementById(`name_${locationIndex}`).value;
    if (!name) { showMessageBox('Name cannot be empty.', 'error'); return; }
    const payload = { index: locationIndex, name: name };
    handleApiResponse(fetch('/save_location_name', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }));
}
function saveLocationActions(locationIndex) {
  const payload = { locationIndex: locationIndex, dropLeftHopper: document.getElementById(`dlh_${locationIndex}`).checked, dropRightHopper: document.getElementById(`drh_${locationIndex}`).checked, releaseLeftHook: document.getElementById(`rlh_${locationIndex}`).checked, releaseRightHook: document.getElementById(`rrh_${locationIndex}`).checked, autoReturnToHome: document.getElementById(`rth_${locationIndex}`).checked };
  handleApiResponse(fetch('/set_location_actions', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }));
}
function deleteLocation(locationIndex) {
  showMessageBox('Are you sure you want to delete Waypoint ' + (locationIndex) + '?', 'confirm', (result) => {
    if (result) {
      fetch('/delete_location', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ locationIndex: locationIndex }) })
        .then(response => response.text()).then(data => { showMessageBox(data); updateTelemetry(); });
    }
  });
}

window.onload = () => {
    document.body.addEventListener('focusin', (event) => {
        if (event.target.tagName === 'INPUT' || event.target.tagName === 'SELECT') { isEditingInput = true; }
    });
    document.body.addEventListener('focusout', (event) => {
        if (event.target.tagName === 'INPUT' || event.target.tagName === 'SELECT') { isEditingInput = false; }
    });

    document.getElementById('restore_form').addEventListener('submit', function(e) {
        e.preventDefault();
        if (!document.getElementById('restore_file').files.length) {
            showMessageBox('Please select a file to restore.', 'error');
            return;
        }

        showMessageBox('This will overwrite all settings and reboot the boat. Are you sure?', 'confirm', (confirmed) => {
            if (confirmed) {
                e.target.submit();
            }
        });
    });

    const restoreFileInput = document.getElementById('restore_file');
    const fileChosenSpan = document.getElementById('file_chosen');
    restoreFileInput.addEventListener('change', function(){
      if(this.files.length > 0){
        fileChosenSpan.textContent = this.files[0].name;
      } else {
        fileChosenSpan.textContent = 'No file chosen';
      }
    });

    const themeSwitch = document.getElementById('themeSwitch');
    const themeLabel = document.getElementById('themeLabel');
    function setTheme(isDark) {
        if (isDark) {
            document.body.classList.add('dark-mode');
            themeSwitch.checked = true;
            themeLabel.innerText = "Light Mode";
        } else {
            document.body.classList.remove('dark-mode');
            themeSwitch.checked = false;
            themeLabel.innerText = "Dark Mode";
        }
    }

    setTheme(localStorage.getItem('theme') === 'dark');

    themeSwitch.addEventListener('change', function() {
        const isDark = this.checked;
        localStorage.setItem('theme', isDark ? 'dark' : 'light');
        setTheme(isDark);
    });

    updateTelemetry();
    setInterval(updateTelemetry, 200);
};