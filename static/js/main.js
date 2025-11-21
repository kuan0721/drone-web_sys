
// 全域變數
let scene, camera, renderer, droneGroup;
let controls; // 3D Interaction Controls (Orbit only)
let propellers = []; // 儲存螺旋槳 mesh 以便旋轉動畫
let map;
let chargingInterval = null;
let chargingStartTime = null;
let allHistoryData = []; // Store all history data for sorting/filtering
let droneMarker;

// Flight Path & GPS State
let mapPolyline; // Leaflet Polyline
let flightPathLine; // Three.js Line
let flightPathGeometry;
let flightPathPoints = []; // Array of Vector3
let initialPos = null; // {lat, lon} for 0,0,0 reference
const MAX_TRAIL_POINTS = 500; // Limit trail length for performance

// Flight & Animation State
let isMouseOverCanvas = false;
let currentRenderAltitude = 0; // The current Y position in 3D scene
let currentRenderPos = new THREE.Vector3(0, 0, 0); // Target X,Y,Z
let targetAltitude = 0;        // The target altitude from API
let targetPos = new THREE.Vector3(0, 0, 0);

// Smooth Rotation State (Quaternion)
let targetQuaternion = new THREE.Quaternion();
let currentQuaternion = new THREE.Quaternion();

// State Caching (Prevent Flickering)
let lastFlightMode = null;
let lastArmStatus = null;
let lastRtkStatus = null;

// 設定與初始化
document.addEventListener('DOMContentLoaded', () => {
    initThreeJS();
    initMap();
    startDataPolling();
    setupEventListeners();
    updateTime();
    setInterval(updateTime, 1000);
});

/* =========================================
   THREE.JS 3D SCENE SETUP
   ========================================= */
function initThreeJS() {
    const container = document.getElementById('canvas-container');

    // Mouse Interaction Listeners
    container.addEventListener('mouseenter', () => { isMouseOverCanvas = true; });
    container.addEventListener('mouseleave', () => { isMouseOverCanvas = false; });

    // 1. Scene
    scene = new THREE.Scene();
    scene.fog = new THREE.FogExp2(0x3E3E3E, 0.015); // Adjusted fog density

    // 2. Camera
    camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 5000);
    camera.position.set(0, 5.0, 8.0); 
    camera.lookAt(0, 0, 0);

    // 3. Renderer (Enable Shadows)
    renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.2;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    // 4. Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.8);
    dirLight.position.set(20, 50, 20);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 200;
    dirLight.shadow.bias = -0.0005;
    dirLight.shadow.radius = 4;
    scene.add(dirLight);
    
    const backLight = new THREE.DirectionalLight(0x56B3D5, 0.8);
    backLight.position.set(-10, 10, -20);
    scene.add(backLight);

    // 5. Grid (Floor) & Shadow Plane
    // Infinite-looking grid
    const gridHelper = new THREE.GridHelper(2000, 1000, 0x555555, 0x444444);
    scene.add(gridHelper);

    const shadowPlane = new THREE.Mesh(
        new THREE.PlaneGeometry(2000, 2000),
        new THREE.ShadowMaterial({ opacity: 0.3 })
    );
    shadowPlane.rotation.x = -Math.PI / 2;
    shadowPlane.receiveShadow = true;
    scene.add(shadowPlane);

    // 6. Flight Path Line (Trail)
    flightPathGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(MAX_TRAIL_POINTS * 3); // Pre-allocate
    flightPathGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    flightPathGeometry.setDrawRange(0, 0); // Start empty

    const trailMaterial = new THREE.LineBasicMaterial({
        color: 0x56B3D5, // Cyan/Sky Blue
        linewidth: 2,
        opacity: 0.8,
        transparent: true
    });
    flightPathLine = new THREE.Line(flightPathGeometry, trailMaterial);
    flightPathLine.frustumCulled = false; // Always render
    scene.add(flightPathLine);

    // 7. Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 - 0.05;
    controls.minDistance = 2;
    controls.maxDistance = 50;

    // 8. Build Detailed Drone Model
    buildDroneModel(3.0);

    // 9. Animation Loop
    animate();

    // Resize Handler
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });
}

/* --- Procedural Textures Helpers --- */
function createAdvancedMetalTexture() {
    const size = 512;
    const canvas = document.createElement('canvas');
    canvas.width = size; canvas.height = size;
    const ctx = canvas.getContext('2d');

    // Diffuse
    ctx.fillStyle = '#2b2b2b'; 
    ctx.fillRect(0, 0, size, size);
    for (let i = 0; i < 4000; i++) {
        const x = Math.random() * size;
        const y = Math.random() * size;
        const len = Math.random() * 100 + 20;
        ctx.fillStyle = Math.random() > 0.5 ? 'rgba(80,80,80,0.8)' : 'rgba(0,0,0,0.1)';
        ctx.fillRect(x, y, len, 1);
    }
    const map = new THREE.CanvasTexture(canvas);
    map.wrapS = THREE.RepeatWrapping;
    map.wrapT = THREE.RepeatWrapping;

    // Roughness
    const roughCanvas = document.createElement('canvas');
    roughCanvas.width = size; roughCanvas.height = size;
    const rctx = roughCanvas.getContext('2d');
    rctx.fillStyle = '#555555'; 
    rctx.fillRect(0, 0, size, size);
    for (let i = 0; i < 5000; i++) {
        const x = Math.random() * size;
        const y = Math.random() * size;
        const w = Math.random() * 2 + 1;
        rctx.fillStyle = Math.random() > 0.5 ? '#777777' : '#333333';
        rctx.fillRect(x, y, w, 1);
    }
    const roughnessMap = new THREE.CanvasTexture(roughCanvas);
    roughnessMap.wrapS = THREE.RepeatWrapping;
    roughnessMap.wrapT = THREE.RepeatWrapping;

    return { map, roughnessMap };
}

function createCarbonFiberTexture() {
    const size = 1024; // Higher res
    const canvas = document.createElement('canvas');
    canvas.width = size; canvas.height = size;
    const ctx = canvas.getContext('2d');

    // Base
    ctx.fillStyle = '#111111';
    ctx.fillRect(0, 0, size, size);

    const tileSize = 32;
    
    // Draw weave pattern
    for (let y = 0; y < size; y += tileSize) {
        for (let x = 0; x < size; x += tileSize) {
            // Checkboard pattern logic
            const isVertical = ((x / tileSize) + (y / tileSize)) % 2 === 0;
            
            const grad = ctx.createLinearGradient(x, y, x + tileSize, y + tileSize);
            
            if (isVertical) {
                // Vertical-ish weave strand
                grad.addColorStop(0, '#000000');
                grad.addColorStop(0.2, '#2a2a2a');
                grad.addColorStop(0.5, '#444444'); // Highlight
                grad.addColorStop(0.8, '#2a2a2a');
                grad.addColorStop(1, '#000000');
            } else {
                // Horizontal-ish weave strand (Darker for contrast)
                grad.addColorStop(0, '#050505');
                grad.addColorStop(0.5, '#1a1a1a');
                grad.addColorStop(1, '#050505');
            }
            
            ctx.fillStyle = grad;
            ctx.fillRect(x, y, tileSize, tileSize);
            
            // Add fine thread details
            ctx.strokeStyle = 'rgba(0,0,0,0.5)';
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.moveTo(x, y + tileSize);
            ctx.lineTo(x + tileSize, y);
            ctx.stroke();
        }
    }

    const map = new THREE.CanvasTexture(canvas);
    map.wrapS = THREE.RepeatWrapping;
    map.wrapT = THREE.RepeatWrapping;
    map.repeat.set(4, 4); // Repeat to make the weave small relative to drone size
    map.anisotropy = 16;  // Sharp at angles

    return map;
}


function buildDroneModel(armLength = 3) {
    if (droneGroup) {
        scene.remove(droneGroup);
        propellers = []; 
    }

    droneGroup = new THREE.Group();
    droneGroup.castShadow = true;

    // --- Textures ---
    const metalTextures = createAdvancedMetalTexture();
    const carbonTexture = createCarbonFiberTexture();

    // --- Materials ---
    const matBody = new THREE.MeshPhysicalMaterial({ 
        map: carbonTexture,
        color: 0xffffff, 
        metalness: 0.0,
        roughness: 0.4,
        clearcoat: 1.0,        
        clearcoatRoughness: 0.2,
        reflectivity: 0.8,
        side: THREE.DoubleSide
    }); 
    
    const matDark = new THREE.MeshStandardMaterial({ 
        color: 0x1a1a1a, 
        roughness: 0.7, 
        metalness: 0.3 
    }); 
    
    const matMetal = new THREE.MeshStandardMaterial({ 
        map: metalTextures.map,
        roughnessMap: metalTextures.roughnessMap,
        color: 0xaaaaaa,
        roughness: 0.5, 
        metalness: 0.8 
    });

    const matGold = new THREE.MeshPhysicalMaterial({ 
        color: 0xd4af37, 
        roughness: 0.3, 
        metalness: 1.0,
        clearcoat: 0.8
    });

    // Propeller Colors
    const matPropFront = new THREE.MeshStandardMaterial({ color: 0xff6600, roughness: 0.4, metalness: 0.1 });
    const matPropRear = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.4, metalness: 0.1 });

    const matGlowCyan = new THREE.MeshBasicMaterial({ color: 0x56B3D5 }); 
    const matLightRed = new THREE.MeshBasicMaterial({ color: 0xef4444 }); 
    const matLightGreen = new THREE.MeshBasicMaterial({ color: 0x22c55e }); 

    function cast(mesh) { mesh.castShadow = true; mesh.receiveShadow = true; return mesh; }

    // --- 1. FUSELAGE (Carbon) ---
    const fuselageGroup = new THREE.Group();
    
    // Main Body Block
    const bodyGeo = new THREE.BoxGeometry(0.6, 0.15, 1.0);
    // UV Mapping adjustment for box to show texture properly
    fuselageGroup.add(cast(new THREE.Mesh(bodyGeo, matBody)));

    // Top Plate
    const topPlateGeo = new THREE.BoxGeometry(0.45, 0.03, 0.8);
    const topPlate = cast(new THREE.Mesh(topPlateGeo, matBody)); 
    topPlate.position.y = 0.09;
    fuselageGroup.add(topPlate);

    // Battery
    const battGeo = new THREE.BoxGeometry(0.3, 0.12, 0.5);
    const batt = cast(new THREE.Mesh(battGeo, matDark));
    batt.position.y = 0.15;
    fuselageGroup.add(batt);

    // Vents
    const ventGeo = new THREE.BoxGeometry(0.65, 0.05, 0.4);
    const vent = cast(new THREE.Mesh(ventGeo, matMetal));
    vent.position.y = 0;
    fuselageGroup.add(vent);

    // Antennas
    const antL = new THREE.Group();
    const stemL = cast(new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.4), matMetal));
    const tipL = new THREE.Mesh(new THREE.SphereGeometry(0.04), matGlowCyan);
    tipL.position.y = 0.2;
    antL.add(stemL, tipL);
    antL.position.set(-0.15, 0.2, 0.45);
    antL.rotation.x = -0.4; 
    antL.rotation.z = 0.4; 
    fuselageGroup.add(antL);

    const antR = antL.clone();
    antR.position.set(0.15, 0.2, 0.45);
    antR.rotation.z = -0.4;
    fuselageGroup.add(antR);

    droneGroup.add(fuselageGroup);

    // --- 2. ARMS (Carbon) ---
    const armGeo = new THREE.BoxGeometry(0.1, 0.06, armLength);
    const arm1 = cast(new THREE.Mesh(armGeo, matBody)); 
    arm1.rotation.y = Math.PI / 4;
    droneGroup.add(arm1);

    const arm2 = arm1.clone();
    arm2.rotation.y = -Math.PI / 4;
    droneGroup.add(arm2);

    const braceGeo = new THREE.BoxGeometry(0.4, 0.02, 0.02);
    const brace1 = cast(new THREE.Mesh(braceGeo, matMetal));
    brace1.position.z = 0.4;
    droneGroup.add(brace1);
    const brace2 = cast(new THREE.Mesh(braceGeo, matMetal));
    brace2.position.z = -0.4;
    droneGroup.add(brace2);

    // --- 3. MOTORS & PROPS ---
    const dist = 0.45 * armLength; 
    const angle = Math.PI / 4;
    
    const motorPositions = [
        { x: -Math.sin(angle)*dist, z: -Math.cos(angle)*dist, dir: 1, color: matLightRed, propMat: matPropFront },
        { x: Math.sin(angle)*dist, z: -Math.cos(angle)*dist, dir: -1, color: matLightGreen, propMat: matPropFront },
        { x: -Math.sin(angle)*dist, z: Math.cos(angle)*dist, dir: -1, color: matLightRed, propMat: matPropRear },
        { x: Math.sin(angle)*dist, z: Math.cos(angle)*dist, dir: 1, color: matLightGreen, propMat: matPropRear }
    ];

    motorPositions.forEach(pos => {
        const mGroup = new THREE.Group();
        mGroup.position.set(pos.x, 0.03, pos.z);

        const mBase = cast(new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.14, 0.15, 32), matMetal));
        mBase.position.y = 0.05;
        mGroup.add(mBase);
        
        const mCoil = cast(new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.13, 0.05, 32), matGold));
        mCoil.position.y = 0.12;
        mGroup.add(mCoil);

        const light = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.02, 0.06), pos.color);
        light.position.y = -0.02; 
        mGroup.add(light);

        const prop = cast(new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.015, 0.15), pos.propMat));
        prop.position.y = 0.21;
        mGroup.add(prop);
        
        const spin = cast(new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.1, 32), matMetal));
        spin.position.y = 0.23;
        mGroup.add(spin);

        prop.userData = { dir: pos.dir };
        propellers.push(prop);
        droneGroup.add(mGroup);
    });

    // --- 4. LANDING GEAR ---
    const skidGeo = new THREE.CylinderGeometry(0.025, 0.025, 1.4);
    const strutGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.6);

    function buildLeg(xPos) {
        const legGroup = new THREE.Group();
        const skid = cast(new THREE.Mesh(skidGeo, matBody)); // Carbon
        skid.rotation.x = Math.PI / 2;
        skid.position.set(xPos, -0.5, 0);
        legGroup.add(skid);

        const v1 = cast(new THREE.Mesh(strutGeo, matMetal));
        v1.position.set(xPos, -0.25, 0.35);
        v1.rotation.x = -0.2;
        legGroup.add(v1);
        
        const v2 = cast(new THREE.Mesh(strutGeo, matMetal));
        v2.position.set(xPos, -0.25, -0.35);
        v2.rotation.x = 0.2;
        legGroup.add(v2);

        const diag = cast(new THREE.Mesh(strutGeo, matMetal));
        diag.scale.y = 1.3;
        diag.position.set(xPos * 0.5, -0.25, 0);
        diag.rotation.z = (xPos > 0) ? 0.6 : -0.6; 
        legGroup.add(diag);
        return legGroup;
    }
    droneGroup.add(buildLeg(-0.4));
    droneGroup.add(buildLeg(0.4));

    // --- 5. GIMBAL ---
    const camGroup = new THREE.Group();
    camGroup.position.set(0, -0.15, -0.6);
    camGroup.add(cast(new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.2, 0.15), matMetal)));
    const camBall = cast(new THREE.Mesh(new THREE.SphereGeometry(0.12, 32, 32), matDark));
    camBall.position.y = -0.18;
    camGroup.add(camBall);
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.05, 32), matMetal);
    lens.rotation.x = Math.PI / 2;
    lens.position.set(0, -0.18, -0.1);
    camGroup.add(lens);
    droneGroup.add(camGroup);

    scene.add(droneGroup);
}

function animate() {
    requestAnimationFrame(animate);

    if(controls) controls.update();

    // Propeller Animation
    propellers.forEach((prop) => {
        prop.rotation.y += 0.8 * prop.userData.dir;
    });

    // --- FLIGHT & SMOOTH ANIMATION LOGIC ---
    if (droneGroup) {
        // 1. Position Interpolation (X, Y, Z)
        // Lerp Altitude
        currentRenderAltitude = THREE.MathUtils.lerp(currentRenderAltitude, targetAltitude, 0.03);
        
        // Lerp Horizontal Position
        currentRenderPos.lerp(targetPos, 0.03);

        // Update Drone Position
        droneGroup.position.set(currentRenderPos.x, currentRenderAltitude, currentRenderPos.z);

        // 2. Rotation Interpolation
        droneGroup.quaternion.slerp(targetQuaternion, 0.05);

        // 3. Camera Follow (Chase Mode)
        // Keep relative offset, but follow drone center
        const camOffset = new THREE.Vector3(0, 5, 8); // Default offset
        // Optional: Rotate offset based on drone yaw if desired (skipping for stability)
        
        controls.target.set(currentRenderPos.x, currentRenderAltitude, currentRenderPos.z);
        
        // Smoothly move camera to maintain offset
        const targetCamPos = currentRenderPos.clone().add(camOffset);
        targetCamPos.y = currentRenderAltitude + 5; // Ensure height clearance
        
        camera.position.lerp(targetCamPos, 0.05);
    }

    renderer.render(scene, camera);
}

/* =========================================
   DATA POLLING & UI UPDATE
   ========================================= */

async function startDataPolling() {
    setInterval(fetchOrientation, 50); 
    setInterval(fetchTelemetry, 500);   
    setInterval(fetchStatus, 1000);     
    fetchChargingHistory();             
}

// 1. Orientation
async function fetchOrientation() {
    try {
        const res = await fetch('/get_drone_orientation');
        const data = await res.json();
        
        const pitch = data.pitch * (Math.PI / 180);
        const yaw = -data.yaw * (Math.PI / 180);
        const roll = -data.roll * (Math.PI / 180);

        const euler = new THREE.Euler(pitch, yaw, roll, 'XYZ');
        targetQuaternion.setFromEuler(euler);

        document.getElementById('pitch-val').innerText = data.pitch.toFixed(1) + '°';
        document.getElementById('roll-val').innerText = data.roll.toFixed(1) + '°';
        document.getElementById('yaw-val').innerText = data.yaw.toFixed(1) + '°';

    } catch (e) { console.error("Orientation Error", e); }
}

// 2. Telemetry (Speed, Alt, Battery)
async function fetchTelemetry() {
    try {
        const batRes = await fetch('/get_battery');
        const batData = await batRes.json();
        updateBatteryUI(batData);

        const spdRes = await fetch('/get_speed');
        const spdData = await spdRes.json();
        const spd = spdData.speed || 0;
        document.getElementById('speed-val').innerText = spd.toFixed(1);
        document.getElementById('spd-bar').style.height = Math.min((spd / 20) * 100, 100) + '%';

        const altRes = await fetch('/get_altitude');
        const altData = await altRes.json();
        const alt = altData.altitude || 0;
        targetAltitude = Math.max(0, alt); // Update Target Y
        
        document.getElementById('altitude-val').innerText = alt.toFixed(1);
        document.getElementById('alt-bar').style.height = Math.min((alt / 100) * 100, 100) + '%';

    } catch (e) { console.error("Telemetry Error", e); }
}

function updateBatteryUI(data) {
    const percent = data.battery_percent || 0;
    const segments = document.querySelectorAll('.battery-segments .seg');
    const activeCount = Math.ceil((percent / 100) * segments.length);

    document.getElementById('battery-percent').innerText = percent + '%';
    document.getElementById('battery-volt').innerText = (data.volt || 0).toFixed(1) + 'V';
    document.getElementById('battery-current').innerText = (data.current || 0).toFixed(1) + 'A';

    segments.forEach((seg, idx) => {
        if (idx < activeCount) {
            seg.classList.add('active');
            seg.classList.remove('low', 'med');
            if (percent < 20) seg.classList.add('low');
            else if (percent < 50) seg.classList.add('med');
        } else {
            seg.classList.remove('active', 'low', 'med');
        }
    });
}

// 3. Status, GPS & PATH TRACKING
async function fetchStatus() {
    try {
        // ... (Mode and Arm status logic unchanged) ...
        const modeRes = await fetch('/get_flight_mode');
        if (modeRes.ok) {
            const modeData = await modeRes.json();
            const newMode = (modeData.flight_mode || 'UNKNOWN').toString().toUpperCase();
            if (newMode !== lastFlightMode) {
                document.getElementById('flight-mode').innerText = newMode;
                lastFlightMode = newMode;
            }
        }

        const armRes = await fetch('/get_arming_status');
        if (armRes.ok) {
            const armData = await armRes.json();
            const newArm = (armData.arm_status || 'UNKNOWN').toString().toUpperCase();
            if (newArm !== lastArmStatus) {
                const armEl = document.getElementById('arming-status');
                armEl.innerText = newArm;
                if (newArm === 'ARMED') armEl.classList.add('armed');
                else armEl.classList.remove('armed');
                lastArmStatus = newArm;
            }
        }

        // RTK
        const rtkRes = await fetch('/get_rtk_status');
        if (rtkRes.ok) {
            const rtkData = await rtkRes.json();
            const newRtk = (rtkData.rtk_status || 'UNKNOWN').toString().toUpperCase();
            if (newRtk !== lastRtkStatus) {
                const rtkEl = document.getElementById('rtk-status');
                rtkEl.innerText = newRtk;
                if(newRtk.includes('FIX')) rtkEl.classList.add('status-ok');
                else rtkEl.classList.remove('status-ok');
                lastRtkStatus = newRtk;
            }
        }

        // --- POSITION & TRAIL LOGIC ---
        const posRes = await fetch('/get_drone_position');
        const posData = await posRes.json();
        const lat = posData.lat || 0;
        const lon = posData.lon || 0;
        
        document.getElementById('lat-val').innerText = lat.toFixed(6);
        document.getElementById('lon-val').innerText = lon.toFixed(6);

        if (lat !== 0 && lon !== 0) {
            // 1. Set Initial Home Position if not set
            if (!initialPos) {
                initialPos = { lat: lat, lon: lon };
                if (map) map.panTo([lat, lon]);
            }

            // 2. Update 2D Map
            if(map) {
                if(droneMarker) droneMarker.setLatLng([lat, lon]);
                if(!mapPolyline) {
                    // Initialize Polyline
                    mapPolyline = L.polyline([], {color: '#56B3D5', weight: 3}).addTo(map);
                }
                // Add point if it moved slightly
                const latLngs = mapPolyline.getLatLngs();
                const lastPt = latLngs.length > 0 ? latLngs[latLngs.length-1] : null;
                
                // Simple distance check to avoid clutter
                if (!lastPt || (Math.abs(lastPt.lat - lat) > 0.00001 || Math.abs(lastPt.lng - lon) > 0.00001)) {
                    mapPolyline.addLatLng([lat, lon]);
                    if (document.getElementById('arming-status').classList.contains('armed')) {
                         map.panTo([lat, lon]); // Auto pan only if armed
                    }
                }
            }

            // 3. Update 3D Target Position
            // Convert Lat/Lon delta to Meters (Approx)
            const R = 6378137; // Earth Radius meters
            const dLat = (lat - initialPos.lat) * Math.PI / 180;
            const dLon = (lon - initialPos.lon) * Math.PI / 180;
            
            // Local 3D coords: X = East(Lon), Z = South(-Lat) for convention
            // Latitude delta in meters = dLat * R
            // Longitude delta in meters = dLon * R * cos(lat)
            const x = dLon * R * Math.cos(initialPos.lat * Math.PI / 180);
            const z = -dLat * R; // Negative because usually -Z is North in 3D

            // Update Target Vector for Animation Loop
            targetPos.set(x, 0, z); // Y is handled by altitude

            // 4. Update 3D Trail Line
            // Only add point if moved enough to save performance
            const currentPoint = new THREE.Vector3(x, targetAltitude, z);
            if (flightPathPoints.length === 0 || flightPathPoints[flightPathPoints.length-1].distanceTo(currentPoint) > 0.5) {
                flightPathPoints.push(currentPoint);
                if (flightPathPoints.length > MAX_TRAIL_POINTS) flightPathPoints.shift(); // Remove old

                // Update Geometry
                const positions = flightPathGeometry.attributes.position.array;
                let idx = 0;
                for (let i = 0; i < flightPathPoints.length; i++) {
                    positions[idx++] = flightPathPoints[i].x;
                    positions[idx++] = flightPathPoints[i].y;
                    positions[idx++] = flightPathPoints[i].z;
                }
                flightPathGeometry.setDrawRange(0, flightPathPoints.length);
                flightPathGeometry.attributes.position.needsUpdate = true;
            }
        }

    } catch (e) { console.error("Status Error", e); }
}

/* =========================================
   MAP & CHARTS
   ========================================= */
function initMap() {
    map = L.map('map', { 
        zoomControl: false,
        attributionControl: false 
    }).setView([25.0330, 121.5654], 16);
    
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
        maxZoom: 20
    }).addTo(map);

    const droneIcon = L.divIcon({
        className: 'drone-map-icon',
        html: '<i class="bi bi-cursor-fill" style="color:#56B3D5; font-size:20px; transform: rotate(-45deg); filter: drop-shadow(0 0 5px rgba(86, 179, 213,0.6));"></i>',
        iconSize: [20, 20],
        iconAnchor: [10, 10]
    });

    droneMarker = L.marker([25.0330, 121.5654], {icon: droneIcon}).addTo(map);
}

/* =========================================
   UI INTERACTIONS & CHARGING
   ========================================= */

function updateTime() {
    const now = new Date();
    document.getElementById('current-time').innerText = now.toLocaleTimeString('en-US', {hour12: false});
}

function setupEventListeners() {
    document.getElementById('btn-start-charge').addEventListener('click', startCharge);
    document.getElementById('btn-stop-charge').addEventListener('click', stopCharge);
    
    document.getElementById('history-sort').addEventListener('change', renderHistory);
    document.getElementById('history-filter-date').addEventListener('input', renderHistory);
    document.getElementById('history-clear-filter').addEventListener('click', () => {
        document.getElementById('history-filter-date').value = '';
        renderHistory();
    });

    document.getElementById('btn-update-model').addEventListener('click', () => {
        const length = parseFloat(document.getElementById('arm-length-input').value);
        if(length && length > 0) buildDroneModel(length);
    });
}

async function startCharge() {
    if(chargingInterval) return;
    
    chargingStartTime = new Date();
    document.getElementById('btn-start-charge').disabled = true;
    document.getElementById('btn-stop-charge').disabled = false;
    
    const statusEl = document.getElementById('charging-status-text');
    statusEl.innerText = "ACTIVE";
    statusEl.style.color = "#22c55e"; 

    chargingInterval = setInterval(() => {
        const now = new Date();
        const diff = Math.floor((now - chargingStartTime) / 1000);
        const m = Math.floor(diff / 60).toString().padStart(2, '0');
        const s = (diff % 60).toString().padStart(2, '0');
        document.getElementById('charging-timer').innerText = `${m}:${s}`;
    }, 1000);
    
    await fetch('/start_charging_monitor', { method: 'POST' });
}

async function stopCharge() {
    if(!chargingInterval) return;
    
    clearInterval(chargingInterval);
    chargingInterval = null;
    
    document.getElementById('btn-start-charge').disabled = false;
    document.getElementById('btn-stop-charge').disabled = true;
    
    const statusEl = document.getElementById('charging-status-text');
    statusEl.innerText = "COMPLETE";
    statusEl.style.color = "#56B3D5"; 
    
    const now = new Date();
    const vStart = parseFloat(document.getElementById('battery-volt').innerText) || 0;
    const vEnd = vStart; 

    const record = {
        start_time: chargingStartTime.toISOString(),
        end_time: now.toISOString(),
        duration_minutes: (now - chargingStartTime)/60000,
        start_voltage: vStart,
        end_voltage: vEnd
    };

    await fetch('/save_charging_record', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(record)
    });

    fetchChargingHistory();
}

function toggleHistory() {
    document.getElementById('history-panel').classList.toggle('open');
}

async function fetchChargingHistory() {
    try {
        const res = await fetch('/get_charging_history');
        const data = await res.json();
        allHistoryData = data.charging_sessions || [];
        renderHistory();
    } catch (e) { console.error(e); }
}

function renderHistory() {
    const tbody = document.getElementById('history-body');
    tbody.innerHTML = '';

    const filterDate = document.getElementById('history-filter-date').value;
    let displayData = [...allHistoryData];

    if (filterDate) {
        displayData = displayData.filter(item => {
            if(!item.start_time) return false;
            const itemDate = new Date(item.start_time).toLocaleDateString('en-CA');
            return itemDate === filterDate;
        });
    }

    const sortMode = document.getElementById('history-sort').value;
    displayData.sort((a, b) => {
        const dateA = new Date(a.start_time || 0);
        const dateB = new Date(b.start_time || 0);
        const durA = a.duration_minutes || 0;
        const durB = b.duration_minutes || 0;

        switch (sortMode) {
            case 'oldest': return dateA - dateB;
            case 'duration-desc': return durB - durA;
            case 'duration-asc': return durA - durB;
            case 'newest': default: return dateB - dateA;
        }
    });

    if (displayData.length === 0) {
        tbody.innerHTML = '<tr><td colspan="4" style="color:#777; padding:20px;">NO DATA LOGGED</td></tr>';
        return;
    }

    displayData.forEach(s => {
        const row = `<tr>
            <td>${s.start_time ? new Date(s.start_time).toLocaleString() : '-'}</td>
            <td>${(s.duration_minutes || 0).toFixed(1)} MIN</td>
            <td>${(s.start_voltage || 0).toFixed(1)}V</td>
            <td>${(s.end_voltage || 0).toFixed(1)}V</td>
        </tr>`;
        tbody.innerHTML += row;
    });
}
