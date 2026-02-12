// Explorer Robot Web GUI JavaScript - Simplified Version

class ExplorerWebGUI {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        
        this.initializeWebSocket();
    }
    
    initializeWebSocket() {
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${wsProtocol}//${window.location.host}/ws`;
        
        console.log('Connecting to WebSocket:', wsUrl);
        
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
            console.log('WebSocket connected');
            this.connected = true;
            this.reconnectAttempts = 0;
        };
        
        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this.handleMessage(data);
            } catch (e) {
                console.error('Error parsing WebSocket message:', e);
            }
        };
        
        this.ws.onclose = () => {
            console.log('WebSocket disconnected');
            this.connected = false;
            this.scheduleReconnect();
        };
        
        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
    }
    
    handleMessage(data) {
        console.log('Received message:', data);
        if (data.type === 'drink_led_update') {
            console.log('[DEBUG] Frontend: drink_led_update received:', data);
            this.updateDrinkLED(data.active);
        }
        switch (data.type) {
            case 'initial':
                this.updateMode(data.mode);
                this.updateSpeedLevel(data.speed_level);
                this.updateRetractStatus(data.retract_status);
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
            
            // Add highlight animation
            currentModeElement.classList.add('status-update');
            setTimeout(() => {
                currentModeElement.classList.remove('status-update');
            }, 500);
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
        if (statusLed) {
            // Remove all status classes
            statusLed.classList.remove('status-ready', 'status-moving', 'status-retracted');

            // Map status string to CSS class
            // "ready" -> green (deployed, ready to use)
            // "in progress" -> orange (moving and not yet at ready position)
            // "retracted"  -> red (retracted)
            if (status === 'ready') {
                statusLed.classList.add('status-ready');
            } else if (status === 'in progress') {
                statusLed.classList.add('status-moving');
            } else {
                // "retracted"
                statusLed.classList.add('status-retracted');
            }
        }
    }
    
    scheduleReconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 10000); // Exponential backoff, max 10s
            this.reconnectAttempts++;
            
            console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
            
            setTimeout(() => {
                this.initializeWebSocket();
            }, delay);
        } else {
            console.error('Max reconnection attempts reached');
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
    if (!document.hidden && window.explorerGUI && !window.explorerGUI.connected) {
        console.log('Page visible - Ensuring WebSocket connection');
        window.explorerGUI.initializeWebSocket();
    }
});
