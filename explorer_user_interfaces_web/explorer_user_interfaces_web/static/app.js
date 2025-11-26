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
        
        switch (data.type) {
            case 'initial':
                this.updateMode(data.mode);
                this.updateLED(data.led_color || 0);
                this.updateSpeedLevel(data.speed_level);
                break;
                
            case 'mode_update':
                this.updateMode(data.mode);
                break;
                
            case 'led_update':
                this.updateLED(data.led_color);
                break;
                
            case 'speed_level_update':
                this.updateSpeedLevel(data.speed_level);
                break;
        }
    }
    
    updateMode(mode) {
        const currentModeElement = document.getElementById('current-mode');
        if (currentModeElement) {
            currentModeElement.textContent = mode;
            
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
        const modeImage = document.getElementById('mode-image');
        const noImageText = document.getElementById('no-image-text');
        const modeImageContainer = document.getElementById('mode-image-container');
        
        if (modeImage && mode) {
            // Try to load mode-specific image
            const imagePath = `/static/images/${mode}_simple.png`;
            
            // Create a test image to check if it exists
            const testImg = new Image();
            testImg.onload = () => {
                modeImage.src = imagePath;
                modeImage.style.display = 'block';
                if (noImageText) noImageText.style.display = 'none';
                if (modeImageContainer) modeImageContainer.style.backgroundColor = 'transparent';
            };
            testImg.onerror = () => {
                // Fallback to default image
                modeImage.style.display = 'none';
                if (noImageText) noImageText.style.display = 'block';
            };
            testImg.src = imagePath;
        }
    }
    
    updateLED(colorValue) {
        const statusLed = document.getElementById('status-led');
        
        if (statusLed) {
            // Remove all status LED classes
            statusLed.className = 'status-led';
            
            // Add new status class based on integer value
            const clampedValue = Math.max(0, Math.min(7, colorValue)); // Clamp between 0-7
            statusLed.classList.add(`status-led-${clampedValue}`);
            
            console.log(`LED color updated to: ${clampedValue}`);
        }
    }
    
    updateSpeedLevel(level) {
        const speedLevelElement = document.getElementById('speed-level');
        if (speedLevelElement) {
            speedLevelElement.textContent = (level !== undefined && level !== null) ? level : '--';
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
