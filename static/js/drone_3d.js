
// Global variables for 3D scene
let scene, camera, renderer, controls;
let droneGroup;
let propellers = [];

const container = document.getElementById('canvas-container');

function init3D() {
    // 1. Setup Scene
    scene = new THREE.Scene();
    
    // Lighting
    const ambientLight = new THREE.AmbientLight(0x404040, 2); // Soft white light
    scene.add(ambientLight);
    
    const dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.position.set(10, 20, 10);
    scene.add(dirLight);

    const accentLight = new THREE.PointLight(0x00f2ff, 2, 50);
    accentLight.position.set(0, 5, 0);
    scene.add(accentLight);

    // 2. Setup Camera
    const aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 1000);
    camera.position.set(3, 3, 5); // Default starting position

    // 3. Setup Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    // 4. Orbit Controls
    // Allows user to rotate (Left Click), Zoom (Scroll), Pan (Right Click)
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Smooth motion
    controls.dampingFactor = 0.05;
    controls.minDistance = 2;
    controls.maxDistance = 20;

    // 5. Build Environment
    const grid = new THREE.GridHelper(50, 50, 0x333333, 0x111111);
    scene.add(grid);

    // 6. Build Drone
    buildProceduralDrone();

    // 7. Start Animation Loop
    animate();

    // Handle Resize
    window.addEventListener('resize', onWindowResize);

    // Bind Reset Button
    const resetBtn = document.getElementById('reset-cam-btn');
    if(resetBtn) {
        resetBtn.addEventListener('click', () => {
            controls.reset();
            camera.position.set(3, 3, 5);
        });
    }
}

function buildProceduralDrone() {
    droneGroup = new THREE.Group();

    // Materials
    const matBody = new THREE.MeshPhongMaterial({ color: 0x222222 });
    const matArm = new THREE.MeshPhongMaterial({ color: 0x333333 });
    const matAccent = new THREE.MeshBasicMaterial({ color: 0x00f2ff });
    const matProp = new THREE.MeshBasicMaterial({ color: 0x555555, transparent: true, opacity: 0.8 });

    // 1. Center Body
    const bodyGeo = new THREE.BoxGeometry(1, 0.3, 1.5);
    const body = new THREE.Mesh(bodyGeo, matBody);
    droneGroup.add(body);

    // 2. Arms (X-Shape)
    const armGeo = new THREE.BoxGeometry(3, 0.15, 0.3);
    
    const arm1 = new THREE.Mesh(armGeo, matArm);
    arm1.rotation.y = Math.PI / 4; // 45 degrees
    droneGroup.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, matArm);
    arm2.rotation.y = -Math.PI / 4; // -45 degrees
    droneGroup.add(arm2);

    // 3. Motors and Propellers
    // Positions relative to center: Front-Left, Front-Right, Back-Left, Back-Right
    const positions = [
        { x: 1.06, z: 1.06 },
        { x: -1.06, z: 1.06 },
        { x: 1.06, z: -1.06 },
        { x: -1.06, z: -1.06 }
    ];

    positions.forEach(pos => {
        // Motor
        const motorGeo = new THREE.CylinderGeometry(0.15, 0.15, 0.2, 16);
        const motor = new THREE.Mesh(motorGeo, matBody);
        motor.position.set(pos.x, 0.2, pos.z);
        droneGroup.add(motor);

        // Propeller blade
        const propGeo = new THREE.BoxGeometry(1.6, 0.02, 0.15);
        const prop = new THREE.Mesh(propGeo, matProp);
        prop.position.set(0, 0.15, 0); // On top of motor
        
        // Parent prop to motor, or create a pivot group
        const propGroup = new THREE.Group();
        propGroup.position.set(pos.x, 0.2, pos.z);
        propGroup.add(prop);
        droneGroup.add(propGroup);

        propellers.push(propGroup);

        // LED Light under motor
        const ledGeo = new THREE.SphereGeometry(0.05);
        const led = new THREE.Mesh(ledGeo, matAccent);
        led.position.set(pos.x, -0.1, pos.z);
        droneGroup.add(led);
    });

    scene.add(droneGroup);
}

function updateDroneAttitude(pitch, roll, yaw) {
    if(!droneGroup) return;

    // Three.js rotation is in Radians
    const d2r = Math.PI / 180;
    
    // Smoothly interpolate or set directly. Setting directly for responsiveness.
    // Mapped to Three.js coordinate system
    droneGroup.rotation.x = pitch * d2r; // Pitch
    droneGroup.rotation.z = -roll * d2r; // Roll (negative for intuitive tilt)
    droneGroup.rotation.y = -yaw * d2r;  // Yaw
}

function animate() {
    requestAnimationFrame(animate);

    // 1. Update Controls
    controls.update();

    // 2. Spin Propellers
    propellers.forEach(prop => {
        prop.rotation.y += 0.5; // Speed
    });

    // 3. Hover Effect (Subtle floating independent of telemetry)
    if(droneGroup) {
        droneGroup.position.y = Math.sin(Date.now() * 0.002) * 0.1;
    }

    // 4. Render
    renderer.render(scene, camera);
}

function onWindowResize() {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// Initialize
document.addEventListener('DOMContentLoaded', init3D);
