// Explorer Robot Web GUI JavaScript - Simplified Version
// Depends on WebSocketWrapper (websocket_wrapper.js)

class ExplorerWebGUI {
    constructor() {
        this.cameraObjectUrl = null;

        this.sockets = {
            data: new WebSocketWrapper('data', '/ws', {
                onMessage: (event) => {
                    try {
                        this.handleMessage(JSON.parse(event.data));
                    } catch (e) {
                        console.error('Error parsing WebSocket message:', e);
                    }
                },
            }),
            camera: new WebSocketWrapper('camera', '/ws/camera', {
                binary_data: true,
                onMessage: (event) => this.handleCameraFrame(event.data),
            }),
        };
    }

    handleCameraFrame(arrayBuffer) {
        const cameraImage = document.getElementById('camera-feed-image');
        if (!cameraImage) return;

        const blob = new Blob([arrayBuffer], { type: 'image/jpeg' });
        const newUrl = URL.createObjectURL(blob);

        if (this.cameraObjectUrl) {
            URL.revokeObjectURL(this.cameraObjectUrl);
        }
        this.cameraObjectUrl = newUrl;
        cameraImage.src = newUrl;
    }

    handleMessage(data) {
        console.log('Received message:', data);
        switch (data.type) {
            case 'drink_led_update':
                this.updateDrinkLED(data.active);
                break;
            case 'initial':
                this.updateMode(data.mode);
                this.updateSpeedLevel(data.speed_level);
                this.updateRetractStatus(data.retract_status);
                break;

            case 'joint_state_update':
                this.updateJointState(data);
                break;

            case 'mode_update':
                this.updateMode(data.mode);
                break;

            case 'speed_level_update':
                this.updateSpeedLevel(data.speed_level);
                break;

            case 'retract_status_update':
                this.updateRetractStatus(data.retract_status);
                break;
        }
    }

    updateMode(mode) {
        const currentModeElement = document.getElementById('current-mode');
        console.log('[DEBUG] updateMode called with:', mode);
        if (mode === 'drink' || (typeof mode === 'string' && mode.toLowerCase().includes('drink'))) {
            console.log('[DEBUG] DRINK MODE ACTIVE (frontend)');
        }
        if (currentModeElement) {
            currentModeElement.textContent = mode;

            currentModeElement.style.display = "inline-block";
            currentModeElement.style.position = "relative";
            currentModeElement.style.transform = "translate(-5px, 0px)";
        }

        // Update mode image
        this.updateModeImage(mode);
    }

    updateModeImage(mode) {
        const modeImageContainer = document.getElementById('mode-image-container');
        const noImageText = document.getElementById('no-image-text');
        if (!modeImageContainer) return;

        // Remove previous overlays (but keep background)
        Array.from(modeImageContainer.querySelectorAll('.mode-overlay')).forEach(img => img.remove());
        if (noImageText) noImageText.style.display = 'none';

        // Get config (assume available as window.explorerConfig)
        const config = window.explorerConfig;
        if (!config || !config.button_mappings) {
            console.warn('No config or button_mappings found');
            return;
        }
        // Try both direct and uppercase mode keys
        let mapping = config.button_mappings[mode];
        if (!mapping) mapping = config.button_mappings[mode.toUpperCase()];
        if (!mapping || !mapping.axes) {
            console.warn('No mapping or axes found for mode:', mode);
            return;
        }
        let overlayFound = false;
        mapping.axes.forEach(axis => {
            if (axis.image) {
                // Debug: show the full image path and check if it exists in the DOM
                const imagePath = `/static/images/${axis.image}`;
                console.log('Trying overlay image:', imagePath);
                const overlayImg = document.createElement('img');
                overlayImg.src = imagePath;
                overlayImg.className = 'mode-overlay';
                overlayImg.style.position = 'absolute';
                overlayImg.style.top = '0';
                overlayImg.style.left = '0';
                overlayImg.style.width = '100%';
                overlayImg.style.height = '100%';
                overlayImg.style.objectFit = 'contain';
                overlayImg.style.zIndex = '2';
                overlayImg.style.pointerEvents = 'none';
                overlayImg.onload = () => {
                    overlayImg.style.display = 'block';
                    console.log('Overlay image loaded:', imagePath);
                };
                overlayImg.onerror = () => {
                    overlayImg.style.display = 'none';
                    console.warn('Overlay image failed to load:', imagePath);
                };
                modeImageContainer.appendChild(overlayImg);
                overlayFound = true;
            }
        });
        if (!overlayFound && noImageText) noImageText.style.display = 'block';
    }

    updateSpeedLevel(level) {
        const speedLevelElement = document.getElementById('speed-level');
        if (speedLevelElement) {
            speedLevelElement.textContent = (level !== undefined && level !== null) ? level : '--';
        }
    }

    updateDrinkLED(active) {
        const drinkLed = document.getElementById('drink-led');
        if (drinkLed) {
            if (active) {
                drinkLed.classList.add('active');
            } else {
                drinkLed.classList.remove('active');
            }
        }
    }

    updateRetractStatus(status) {
        const statusLed = document.getElementById('status-led');
        if (!statusLed) {
            return;
        }
        // Remove all status classes
        statusLed.classList.remove('status-green', 'status-orange', 'status-red');

        // Map status string to CSS class
        // "ready" -> green (deployed, ready to use)
        // "in progress" -> orange (moving and not yet at ready position)
        // "retracted"  -> red (retracted)
        if (status === 'ready') {
            statusLed.classList.add('status-green');
        } else if (status === 'in progress') {
            statusLed.classList.add('status-orange');
        } else {
            // "retracted"
            statusLed.classList.add('status-red');
        }
    }

    updateJointState(data) {
        const jointLed = document.getElementById('joint-led-' + data.joint_index);
        if (!jointLed) {
            return;
        }
        // Remove previous status classes
        jointLed.classList.remove('status-green', 'status-orange', 'status-red');

        switch (data.state) {
            case "Init": case "Idle": 
                jointLed.classList.add('status-orange');
                break;
            case "Enable":
                jointLed.classList.add('status-green');
                break;
            case "Hold":
            case "Brake":
            case "EStop":
            case "unknown":
                jointLed.classList.add('status-red');
                break;
        }
    }
}

// Initialize the web GUI when the page loads
document.addEventListener('DOMContentLoaded', () => {
    console.log('Explorer Robot Web GUI starting...');
    window.explorerGUI = new ExplorerWebGUI();
});

// Handle page visibility changes
document.addEventListener('visibilitychange', () => {
    if (!document.hidden && window.explorerGUI) {
        for (const socket of Object.values(window.explorerGUI.sockets)) {
            socket.ensureConnected();
        }
    }
});
