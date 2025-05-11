// Map initialization and variables
let map;
let waypoints = [];
let isConnected = false;
let isAddingWaypoint = false;
let telemetryInterval;
let waypointLayer;
let polyline;
let simulatedAltitude = 0;
let simulatedSpeed = 0;
let simulatedHeading = 0;
let simulatedBattery = 100;
let vesselMarker;
let currentMissionType = "none"; // none, zigzag, obstacle, docking
let obstacles = [];
let obstacleLayer;
let dockingPoint = null;
let dockingMarker = null;
let zigzagPath = [];
let missionInProgress = false;
let currentWaypointIndex = 0;
let missionProgressInterval;

// Add log entry
function addLogEntry(message) {
    const logContainer = document.getElementById('log-container');
    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry';
    
    // Add timestamp to log entry
    const now = new Date();
    const timestamp = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
    
    logEntry.textContent = `[${timestamp}] ${message}`;
    logContainer.appendChild(logEntry);
    
    // Auto-scroll to bottom
    logContainer.scrollTop = logContainer.scrollHeight;
    
    // Keep only the last 100 log entries to prevent performance issues
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.children[0]);
    }
}

// Initialize map with satellite imagery
function initMap() {
    // Default coordinates (Change to your preferred starting location)
    const defaultLat = 39.9334;
    const defaultLng = 32.8597;
    
    map = L.map('map').setView([defaultLat, defaultLng], 13);
    
    // Use ESRI satellite imagery instead of Mapbox (more reliable without API key)
    const satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
        maxZoom: 19
    }).addTo(map);
    
    // Add standard street layer as an option
    const streetsLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    });
    
    const baseMaps = {
        "Satellite": satelliteLayer,
        "Streets": streetsLayer
    };
    
    L.control.layers(baseMaps).addTo(map);
    
    // Force satellite layer to be active
    map.removeLayer(streetsLayer);
    map.addLayer(satelliteLayer);
    
    // Initialize empty layers
    waypointLayer = L.layerGroup().addTo(map);
    obstacleLayer = L.layerGroup().addTo(map);
    
    // Initialize polyline for mission path
    polyline = L.polyline([], {color: 'yellow', weight: 3}).addTo(map);
    
    // Add vessel marker with ship icon
    const vesselIcon = L.divIcon({
        html: '<div class="vessel-icon">⛵</div>',
        className: 'vessel-marker',
        iconSize: [30, 30],
        iconAnchor: [15, 15]
    });
    
    vesselMarker = L.marker([defaultLat, defaultLng], {icon: vesselIcon}).addTo(map);
    
    // Map click handler for adding waypoints
    map.on('click', function(e) {
        if (isAddingWaypoint && isConnected) {
            addWaypoint(e.latlng.lat, e.latlng.lng);
            isAddingWaypoint = false;
            document.getElementById('add-waypoint-btn').textContent = 'Add Waypoint';
            addLogEntry(`Waypoint added at: ${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`);
        } else if (currentMissionType === "obstacle" && isConnected && document.getElementById('add-obstacle-btn').classList.contains('active')) {
            addObstacle(e.latlng.lat, e.latlng.lng);
            addLogEntry(`Obstacle added at: ${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`);
        } else if (currentMissionType === "docking" && isConnected && document.getElementById('set-docking-btn').classList.contains('active')) {
            setDockingPoint(e.latlng.lat, e.latlng.lng);
            document.getElementById('set-docking-btn').classList.remove('active');
            addLogEntry(`Docking point set at: ${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`);
        }
    });
    
    addLogEntry("Map initialized with satellite view");
}

// Add a waypoint to the mission
function addWaypoint(lat, lng) {
    const waypointNumber = waypoints.length + 1;
    const waypoint = {
        id: waypointNumber,
        lat: lat,
        lng: lng,
        alt: 0 // For water surface
    };
    
    waypoints.push(waypoint);
    
    // Add marker to map
    const marker = L.marker([lat, lng], {
        draggable: true,
        title: `Waypoint ${waypointNumber}`
    })
    .bindPopup(`<b>Waypoint ${waypointNumber}</b><br>Lat: ${lat.toFixed(6)}<br>Lng: ${lng.toFixed(6)}`)
    .addTo(waypointLayer);
    
    // Update marker position when dragged
    marker.on('dragend', function(e) {
        const position = e.target.getLatLng();
        waypoint.lat = position.lat;
        waypoint.lng = position.lng;
        updateMissionPath();
        e.target.bindPopup(`<b>Waypoint ${waypointNumber}</b><br>Lat: ${position.lat.toFixed(6)}<br>Lng: ${position.lng.toFixed(6)}`);
        addLogEntry(`Waypoint ${waypointNumber} moved to: ${position.lat.toFixed(6)}, ${position.lng.toFixed(6)}`);
    });
    
    // Update polyline
    updateMissionPath();
    
    // Add to waypoint list
    const waypointsList = document.getElementById('waypoints');
    const listItem = document.createElement('li');
    listItem.textContent = `WP${waypointNumber}: ${lat.toFixed(5)}, ${lng.toFixed(5)}`;
    waypointsList.appendChild(listItem);
    
    // Enable upload button if connected
    if (isConnected) {
        document.getElementById('upload-mission-btn').disabled = false;
    }
}

// Update the mission path line on the map
function updateMissionPath() {
    const points = waypoints.map(wp => [wp.lat, wp.lng]);
    polyline.setLatLngs(points);
}

// Clear all waypoints
function clearMission() {
    waypoints = [];
    waypointLayer.clearLayers();
    polyline.setLatLngs([]);
    document.getElementById('waypoints').innerHTML = '';
    document.getElementById('upload-mission-btn').disabled = true;
    
    // Also clear any mission-specific elements
    obstacles = [];
    obstacleLayer.clearLayers();
    if (dockingMarker) {
        map.removeLayer(dockingMarker);
        dockingPoint = null;
        dockingMarker = null;
    }
    zigzagPath = [];
    
    addLogEntry("Mission cleared");
    
    // Stop any in-progress mission
    if (missionInProgress) {
        stopMission();
    }
}

// Simulate connection to vessel controller
function connectToPixhawk() {
    const statusElement = document.getElementById('connection-status');
    const connectBtn = document.getElementById('connect-btn');
    const portSelect = document.getElementById('port-select');
    const baudSelect = document.getElementById('baud-select');
    
    if (isConnected) {
        // Disconnect
        statusElement.textContent = 'Disconnected';
        statusElement.className = 'disconnected';
        connectBtn.textContent = 'Connect';
        isConnected = false;
        
        // Stop telemetry simulation
        clearInterval(telemetryInterval);
        
        // Disable mission upload
        document.getElementById('upload-mission-btn').disabled = true;
        
        // Hide mission type buttons
        document.getElementById('mission-types').style.display = 'none';
        
        addLogEntry(`Disconnected from vessel controller`);
    } else {
        // Connect animation
        statusElement.textContent = 'Connecting...';
        statusElement.className = 'connecting';
        connectBtn.disabled = true;
        
        const selectedPort = portSelect.value;
        const selectedBaud = baudSelect.value;
        
        addLogEntry(`Connecting to vessel controller on ${selectedPort} at ${selectedBaud} baud...`);
        
        // Simulate connection delay
        setTimeout(() => {
            statusElement.textContent = 'Connected';
            statusElement.className = 'connected';
            connectBtn.textContent = 'Disconnect';
            connectBtn.disabled = false;
            isConnected = true;
            
            // Enable mission upload if waypoints exist
            if (waypoints.length > 0) {
                document.getElementById('upload-mission-btn').disabled = false;
            }
            
            // Start simulated telemetry data
            startTelemetrySimulation();
            
            // Show mission type buttons
            document.getElementById('mission-types').style.display = 'block';
            
            addLogEntry(`Connected to vessel controller on ${selectedPort}`);
            addLogEntry(`Firmware version: MarinePilot V2.3 (simulated)`);
            addLogEntry(`Vehicle type: Autonomous Surface Vessel`);
        }, 2000);
    }
}

// Simulate telemetry data
function startTelemetrySimulation() {
    addLogEntry("Telemetry stream started");
    
    telemetryInterval = setInterval(() => {
        // Generate random variations for simulated data
        simulatedAltitude = 0; // Always 0 for surface vessel
        simulatedSpeed = Math.max(0, simulatedSpeed + (Math.random() * 0.5 - 0.25));
        simulatedHeading = (simulatedHeading + (Math.random() * 5 - 2.5)) % 360;
        simulatedBattery = Math.max(0, Math.min(100, simulatedBattery - 0.05));
        
        // Update telemetry display
        document.getElementById('altitude').textContent = simulatedAltitude.toFixed(1);
        document.getElementById('speed').textContent = simulatedSpeed.toFixed(1);
        document.getElementById('heading').textContent = simulatedHeading.toFixed(0);
        document.getElementById('battery').textContent = simulatedBattery.toFixed(0);
        
        // If mission in progress, move vessel along path
        if (missionInProgress && waypoints.length > 0) {
            moveVesselAlongMission();
        }
        // Otherwise just add slight random movement if there are waypoints
        else if (waypoints.length > 0 && !missionInProgress) {
            const idx = Math.floor(Math.random() * waypoints.length);
            const randomWp = waypoints[idx];
            
            // Move slightly toward a random waypoint
            const currentLatLng = vesselMarker.getLatLng();
            const newLat = currentLatLng.lat + (randomWp.lat - currentLatLng.lat) * 0.001;
            const newLng = currentLatLng.lng + (randomWp.lng - currentLatLng.lng) * 0.001;
            
            vesselMarker.setLatLng([newLat, newLng]);
        }
        
        // Rotate vessel icon based on heading
        const vesselIcon = document.querySelector('.vessel-icon');
        if (vesselIcon) {
            vesselIcon.style.transform = `rotate(${simulatedHeading}deg)`;
        }
        
        // Occasionally log telemetry updates
        if (Math.random() < 0.05) {
            addLogEntry(`Telemetry: Speed=${simulatedSpeed.toFixed(1)}m/s, Heading=${simulatedHeading.toFixed(0)}°`);
        }
    }, 1000);
}

// Simulate mission upload to vessel controller
function uploadMission() {
    const uploadBtn = document.getElementById('upload-mission-btn');
    uploadBtn.disabled = true;
    uploadBtn.textContent = 'Uploading...';
    
    addLogEntry(`Uploading mission with ${waypoints.length} waypoints...`);
    
    // Simulate upload delay
    setTimeout(() => {
        uploadBtn.textContent = 'Upload Complete';
        addLogEntry('Mission upload complete');
        
        // Enable start mission button
        document.getElementById('start-mission-btn').disabled = false;
        
        setTimeout(() => {
            uploadBtn.textContent = 'Upload Mission';
            uploadBtn.disabled = false;
        }, 2000);
    }, 3000);
}

// Set mission type
function setMissionType(type) {
    // Reset any active buttons
    document.querySelectorAll('.mission-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    
    // Set the current button as active
    document.getElementById(`${type}-mission-btn`).classList.add('active');
    
    // Update current mission type
    currentMissionType = type;
    
    // Clear previous mission elements
    clearMission();
    
    // Show relevant mission controls and hide others
    document.querySelectorAll('.mission-controls').forEach(panel => {
        panel.style.display = 'none';
    });
    
    if (type !== "none") {
        document.getElementById(`${type}-controls`).style.display = 'block';
    }
    
    addLogEntry(`Mission type set to: ${type}`);
}

// Generate zigzag path
function generateZigzag() {
    // Need at least start and end points
    if (waypoints.length < 2) {
        alert('Please add at least start and end points for zigzag path');
        addLogEntry('Error: At least 2 waypoints needed for zigzag path');
        return;
    }
    
    const startPoint = waypoints[0];
    const endPoint = waypoints[waypoints.length - 1];
    
    // Get zigzag parameters
    const width = parseFloat(document.getElementById('zigzag-width').value);
    const segments = parseInt(document.getElementById('zigzag-segments').value);
    
    if (isNaN(width) || isNaN(segments) || width <= 0 || segments <= 0) {
        alert('Please enter valid zigzag parameters');
        return;
    }
    
    // Clear existing waypoints except start and end
    waypoints = [waypoints[0], waypoints[waypoints.length - 1]];
    waypointLayer.clearLayers();
    document.getElementById('waypoints').innerHTML = '';
    
    // Re-add start waypoint to UI
    addWaypointToUI(startPoint.lat, startPoint.lng, 1);
    
    // Calculate direction vector from start to end
    const dx = endPoint.lng - startPoint.lng;
    const dy = endPoint.lat - startPoint.lat;
    const distance = Math.sqrt(dx * dx + dy * dy);
    
    // Unit vector in the direction of travel
    const ux = dx / distance;
    const uy = dy / distance;
    
    // Perpendicular unit vector (for zigzag width)
    const vx = -uy;
    const vy = ux;
    
    // Length of each segment
    const segmentLength = distance / segments;
    
    // Generate zigzag waypoints
    zigzagPath = []; // Clear previous zigzag path
    zigzagPath.push([startPoint.lat, startPoint.lng]); // Add start point
    
    for (let i = 1; i < segments; i++) {
        // Alternate zigzag direction
        const side = i % 2 === 1 ? 1 : -1;
        
        // Position along the straight path
        const pos = i / segments;
        const x = startPoint.lng + dx * pos;
        const y = startPoint.lat + dy * pos;
        
        // Add zigzag offset
        const zigzagX = x + vx * width * side;
        const zigzagY = y + vy * width * side;
        
        // Add waypoint
        const wpNumber = i + 1;
        addWaypoint(zigzagY, zigzagX);
        zigzagPath.push([zigzagY, zigzagX]);
    }
    
    // Make sure end point is added
    zigzagPath.push([endPoint.lat, endPoint.lng]);
    
    // Re-add end waypoint to UI if it's not the same as last zigzag point
    const lastPointIdx = waypoints.length - 1;
    if (waypoints[lastPointIdx].lat !== endPoint.lat || waypoints[lastPointIdx].lng !== endPoint.lng) {
        addWaypointToUI(endPoint.lat, endPoint.lng, waypoints.length + 1);
    }
    
    // Update polyline
    updateMissionPath();
    
    addLogEntry(`Zigzag path generated with ${segments} segments and ${width}m width`);
}

// Add waypoint to UI without adding to mission
function addWaypointToUI(lat, lng, number) {
    // Add marker to map
    const marker = L.marker([lat, lng], {
        draggable: true,
        title: `Waypoint ${number}`
    })
    .bindPopup(`<b>Waypoint ${number}</b><br>Lat: ${lat.toFixed(6)}<br>Lng: ${lng.toFixed(6)}`)
    .addTo(waypointLayer);
    
    // Add to waypoint list
    const waypointsList = document.getElementById('waypoints');
    const listItem = document.createElement('li');
    listItem.textContent = `WP${number}: ${lat.toFixed(5)}, ${lng.toFixed(5)}`;
    waypointsList.appendChild(listItem);
}

// Add obstacle for obstacle avoidance mission
function addObstacle(lat, lng) {
    const obstacleId = obstacles.length + 1;
    const obstacle = {
        id: obstacleId,
        lat: lat,
        lng: lng,
        radius: parseFloat(document.getElementById('obstacle-size').value) || 10 // Default 10m radius
    };
    
    obstacles.push(obstacle);
    
    // Add circle to map
    const circle = L.circle([lat, lng], {
        color: 'red',
        fillColor: '#f03',
        fillOpacity: 0.3,
        radius: obstacle.radius
    }).addTo(obstacleLayer);
    
    circle.bindPopup(`<b>Obstacle ${obstacleId}</b><br>Radius: ${obstacle.radius}m`);
    
    // Add to obstacle list
    const obstaclesList = document.getElementById('obstacles-list');
    const listItem = document.createElement('li');
    listItem.textContent = `Obstacle ${obstacleId}: ${lat.toFixed(5)}, ${lng.toFixed(5)}, ${obstacle.radius}m`;
    obstaclesList.appendChild(listItem);
    
    addLogEntry(`Obstacle ${obstacleId} added with ${obstacle.radius}m radius`);
}

// Set docking point for docking mission
function setDockingPoint(lat, lng) {
    // Remove existing docking marker if exists
    if (dockingMarker) {
        map.removeLayer(dockingMarker);
    }
    
    dockingPoint = {lat: lat, lng: lng};
    
    // Create custom docking point icon
    const dockIcon = L.divIcon({
        html: '<div class="dock-icon">🚢</div>',
        className: 'dock-marker',
        iconSize: [40, 40],
        iconAnchor: [20, 20]
    });
    
    // Add marker
    dockingMarker = L.marker([lat, lng], {icon: dockIcon}).addTo(map);
    dockingMarker.bindPopup(`<b>Docking Point</b><br>Lat: ${lat.toFixed(6)}<br>Lng: ${lng.toFixed(6)}`);
    
    // Update docking info in UI
    document.getElementById('docking-info').textContent = `Docking point set at: ${lat.toFixed(5)}, ${lng.toFixed(5)}`;
    
    // Add final waypoint approach for docking
    if (waypoints.length > 0) {
        // Calculate approach point (30m away from docking in the direction the vessel will be coming from)
        const lastWaypoint = waypoints[waypoints.length - 1];
        
        // Direction vector from last waypoint to docking point
        const dx = dockingPoint.lng - lastWaypoint.lng;
        const dy = dockingPoint.lat - lastWaypoint.lat;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        if (distance > 0) {
            // Normalized direction vector
            const nx = dx / distance;
            const ny = dy / distance;
            
            // Approach point (30m before docking point)
            const approachDist = Math.min(30, distance * 0.5);
            const approachLng = dockingPoint.lng - nx * approachDist;
            const approachLat = dockingPoint.lat - ny * approachDist;
            
            // Add approach waypoint
            addWaypoint(approachLat, approachLng);
            
            // Add docking point as final waypoint
            addWaypoint(dockingPoint.lat, dockingPoint.lng);
            
            addLogEntry("Approach path to docking point created");
        }
    }
}

// Toggle add obstacle mode
function toggleAddObstacle() {
    const addObstacleBtn = document.getElementById('add-obstacle-btn');
    
    if (addObstacleBtn.classList.contains('active')) {
        addObstacleBtn.classList.remove('active');
        addObstacleBtn.textContent = 'Add Obstacle';
    } else {
        addObstacleBtn.classList.add('active');
        addObstacleBtn.textContent = 'Click on map';
        addLogEntry("Click on map to add obstacle");
    }
}

// Toggle set docking point mode
function toggleSetDocking() {
    const setDockingBtn = document.getElementById('set-docking-btn');
    
    if (setDockingBtn.classList.contains('active')) {
        setDockingBtn.classList.remove('active');
        setDockingBtn.textContent = 'Set Docking Point';
    } else {
        setDockingBtn.classList.add('active');
        setDockingBtn.textContent = 'Click on map';
        addLogEntry("Click on map to set docking point");
    }
}

// Start mission execution
function startMission() {
    if (waypoints.length < 2) {
        alert('Please add at least 2 waypoints for the mission');
        return;
    }
    
    missionInProgress = true;
    currentWaypointIndex = 0;
    simulatedSpeed = 2.0; // Set initial speed
    
    document.getElementById('start-mission-btn').disabled = true;
    document.getElementById('stop-mission-btn').disabled = false;
    
    // Position vessel at first waypoint
    vesselMarker.setLatLng([waypoints[0].lat, waypoints[0].lng]);
    
    addLogEntry("Mission started - Proceeding to first waypoint");
    
    // Change status display
    const missionStatusElement = document.getElementById('mission-status');
    if (missionStatusElement) {
        missionStatusElement.textContent = "In Progress";
        missionStatusElement.className = "status-in-progress";
    }
}

// Stop mission execution
function stopMission() {
    missionInProgress = false;
    
    document.getElementById('start-mission-btn').disabled = false;
    document.getElementById('stop-mission-btn').disabled = true;
    
    addLogEntry("Mission stopped by user");
    
    // Change status display
    const missionStatusElement = document.getElementById('mission-status');
    if (missionStatusElement) {
        missionStatusElement.textContent = "Stopped";
        missionStatusElement.className = "status-stopped";
    }
}

// Move vessel along mission path
function moveVesselAlongMission() {
    if (!missionInProgress || currentWaypointIndex >= waypoints.length) {
        return;
    }
    
    const currentPos = vesselMarker.getLatLng();
    const targetWaypoint = waypoints[currentWaypointIndex];
    
    // Calculate distance to waypoint
    const dx = targetWaypoint.lng - currentPos.lng;
    const dy = targetWaypoint.lat - currentPos.lat;
    const distanceToWaypoint = Math.sqrt(dx * dx + dy * dy);
    
    // Calculate heading to waypoint
    let heading = Math.atan2(dy, dx) * (180 / Math.PI);
    if (heading < 0) heading += 360;
    
    // Gradually adjust vessel heading
    const headingDiff = (heading - simulatedHeading + 360) % 360;
    const turnDirection = headingDiff > 180 ? -1 : 1;
    const turnAmount = Math.min(5, Math.abs(headingDiff > 180 ? 360 - headingDiff : headingDiff));
    simulatedHeading = (simulatedHeading + turnAmount * turnDirection + 360) % 360;
    
    // Check if we've reached the waypoint (within 10m)
    if (distanceToWaypoint < 0.0001) { // Roughly 10 meters
        addLogEntry(`Reached waypoint ${currentWaypointIndex + 1}`);
        currentWaypointIndex++;
        
        // Mission complete if we've reached all waypoints
        if (currentWaypointIndex >= waypoints.length) {
            addLogEntry("Mission complete - All waypoints reached");
            missionInProgress = false;
            document.getElementById('start-mission-btn').disabled = false;
            document.getElementById('stop-mission-btn').disabled = true;
            
            // Change status display
            const missionStatusElement = document.getElementById('mission-status');
            if (missionStatusElement) {
                missionStatusElement.textContent = "Completed";
                missionStatusElement.className = "status-completed";
            }
            
            return;
        }
    }
    
    // Calculate movement based on heading and speed
    const movementFactor = 0.00001 * simulatedSpeed; // Adjust for vessel speed
    const newLng = currentPos.lng + Math.cos(simulatedHeading * Math.PI / 180) * movementFactor;
    const newLat = currentPos.lat + Math.sin(simulatedHeading * Math.PI / 180) * movementFactor;
    
    // Update vessel position
    vesselMarker.setLatLng([newLat, newLng]);
    
    // Rotate vessel icon based on heading
    const vesselIcon = document.querySelector('.vessel-icon');
    if (vesselIcon) {
        vesselIcon.style.transform = `rotate(${simulatedHeading}deg)`;
    }
}

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initMap();
    addLogEntry('System initialized');
    addLogEntry('Ready to connect to vessel controller');
    
    // Mobile viewport height fix (for mobile browsers)
    setVhVariable();
    window.addEventListener('resize', setVhVariable);
    
    // Add mobile menu button and overlay to DOM
    setupMobileInterface();
    
    // Event Listeners
    document.getElementById('connect-btn').addEventListener('click', connectToPixhawk);
    document.getElementById('clear-mission-btn').addEventListener('click', clearMission);
    document.getElementById('upload-mission-btn').addEventListener('click', uploadMission);
    
    document.getElementById('add-waypoint-btn').addEventListener('click', function() {
        if (!isConnected) {
            alert('Please connect to vessel controller first');
            addLogEntry('Error: Cannot add waypoint while disconnected');
            return;
        }
        
        isAddingWaypoint = !isAddingWaypoint;
        if (isAddingWaypoint) {
            this.textContent = 'Click on map';
            addLogEntry('Click on map to add waypoint');
            
            // On mobile, close the sidebar after selecting this option
            if (window.innerWidth <= 768) {
                toggleMobileSidebar(false);
            }
        } else {
            this.textContent = 'Add Waypoint';
        }
    });
    
    // Mission type buttons
    document.getElementById('zigzag-mission-btn').addEventListener('click', () => {
        setMissionType('zigzag');
        // On mobile, close sidebar after selecting mission type
        if (window.innerWidth <= 768) {
            setTimeout(() => toggleMobileSidebar(false), 300);
        }
    });
    
    document.getElementById('obstacle-mission-btn').addEventListener('click', () => {
        setMissionType('obstacle');
        // On mobile, close sidebar after selecting mission type
        if (window.innerWidth <= 768) {
            setTimeout(() => toggleMobileSidebar(false), 300);
        }
    });
    
    document.getElementById('docking-mission-btn').addEventListener('click', () => {
        setMissionType('docking');
        // On mobile, close sidebar after selecting mission type
        if (window.innerWidth <= 768) {
            setTimeout(() => toggleMobileSidebar(false), 300);
        }
    });
    
    // Zigzag controls
    document.getElementById('generate-zigzag-btn').addEventListener('click', generateZigzag);
    
    // Obstacle controls
    document.getElementById('add-obstacle-btn').addEventListener('click', function() {
        toggleAddObstacle();
        // On mobile, close the sidebar after selecting this option
        if (window.innerWidth <= 768 && this.classList.contains('active')) {
            toggleMobileSidebar(false);
        }
    });
    
    // Docking controls
    document.getElementById('set-docking-btn').addEventListener('click', function() {
        toggleSetDocking();
        // On mobile, close the sidebar after selecting this option
        if (window.innerWidth <= 768 && this.classList.contains('active')) {
            toggleMobileSidebar(false);
        }
    });
    
    // Mission control buttons
    document.getElementById('start-mission-btn').addEventListener('click', startMission);
    document.getElementById('stop-mission-btn').addEventListener('click', stopMission);
    
    // If on mobile, make sure map controls are usable
    if (window.innerWidth <= 768) {
        // Initialize with sidebar closed
        toggleMobileSidebar(false);
        
        // Delay map invalidation to ensure proper sizing
        setTimeout(() => {
            if (map) map.invalidateSize();
        }, 300);
    }
});

// Set the --vh CSS variable based on actual viewport height
// This fixes issues with mobile browser address bars
function setVhVariable() {
    const vh = window.innerHeight * 0.01;
    document.documentElement.style.setProperty('--vh', `${vh}px`);
    
    // Force map to redraw if it exists
    if (map) {
        setTimeout(() => map.invalidateSize(), 100);
    }
}

// Setup mobile interface elements
function setupMobileInterface() {
    // Only add if they don't already exist
    if (!document.querySelector('.mobile-menu-btn')) {
        // Create mobile menu button
        const menuBtn = document.createElement('button');
        menuBtn.className = 'mobile-menu-btn';
        menuBtn.innerHTML = '☰';
        menuBtn.setAttribute('aria-label', 'Toggle Menu');
        document.body.appendChild(menuBtn);
        
        // Create overlay for sidebar
        const overlay = document.createElement('div');
        overlay.className = 'sidebar-overlay';
        document.body.appendChild(overlay);
        
        // Event listeners for mobile controls
        menuBtn.addEventListener('click', function() {
            toggleMobileSidebar();
        });
        
        overlay.addEventListener('click', function() {
            toggleMobileSidebar(false);
        });
        
        // Add swipe detection to sidebar
        const sidebar = document.querySelector('.sidebar');
        let touchStartX = 0;
        let touchEndX = 0;
        
        sidebar.addEventListener('touchstart', (e) => {
            touchStartX = e.changedTouches[0].screenX;
        }, { passive: true });
        
        sidebar.addEventListener('touchend', (e) => {
            touchEndX = e.changedTouches[0].screenX;
            if (touchStartX - touchEndX > 50) {
                // Swipe left - close sidebar
                toggleMobileSidebar(false);
            }
        }, { passive: true });
    }
}

// Toggle mobile sidebar visibility
function toggleMobileSidebar(forceState) {
    const sidebar = document.querySelector('.sidebar');
    const overlay = document.querySelector('.sidebar-overlay');
    const menuBtn = document.querySelector('.mobile-menu-btn');
    
    if (forceState === undefined) {
        // Toggle based on current state
        sidebar.classList.toggle('active');
        overlay.classList.toggle('active');
        menuBtn.innerHTML = sidebar.classList.contains('active') ? '✕' : '☰';
    } else if (forceState) {
        // Force open
        sidebar.classList.add('active');
        overlay.classList.add('active');
        menuBtn.innerHTML = '✕';
    } else {
        // Force close
        sidebar.classList.remove('active');
        overlay.classList.remove('active');
        menuBtn.innerHTML = '☰';
    }
    
    // On any sidebar state change, ensure map is properly sized
    if (map) {
        setTimeout(() => map.invalidateSize(), 300);
    }
}

// Update the initMap function to be more mobile-friendly
const originalInitMap = initMap;
initMap = function() {
    originalInitMap();
    
    // Make map controls more touch-friendly
    if (map) {
        // Add touch detection
        if ('ontouchstart' in window || navigator.maxTouchPoints) {
            map.addHandler('tap', L.Map.Tap);
            map.tap.enable();
        }
        
        // Disable map zoom when double clicking
        map.doubleClickZoom.disable();
        
        // Move zoom control to bottom right for better thumb reach on mobile
        if (window.innerWidth <= 768) {
            map.zoomControl.remove();
            L.control.zoom({ position: 'bottomright' }).addTo(map);
        }
    }
};

// Modify connectToPixhawk to handle camera offline panel on mobile
const originalConnectToPixhawk = connectToPixhawk;
connectToPixhawk = function() {
    originalConnectToPixhawk();
    
    // Update camera offline display on mobile
    const cameraPanel = document.querySelector('.camera-offline-panel');
    if (cameraPanel) {
        if (isConnected) {
            cameraPanel.classList.remove('show-mobile');
        } else {
            // On disconnect, show the camera offline panel briefly for mobile
            if (window.innerWidth <= 768) {
                cameraPanel.classList.add('show-mobile');
                setTimeout(() => {
                    cameraPanel.classList.remove('show-mobile');
                }, 3000);
            }
        }
    }
};