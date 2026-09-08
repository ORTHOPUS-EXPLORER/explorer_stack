// Generic WebSocket wrapper
class WebSocketWrapper {
    constructor(name, path, { binary_data = false, maxReconnectAttempts = 5, onMessage, onOpen } = {}) {
        this.name = name;
        this.path = path;
        this.binary_data = binary_data;
        this.maxReconnectAttempts = maxReconnectAttempts;
        this.onMessage = onMessage;
        this.onOpen = onOpen;

        this.ws = null;
        this.connected = false;
        this.reconnectAttempts = 0;

        this.connect();
    }

    connect() {
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${wsProtocol}//${window.location.host}${this.path}`;

        console.log(`Connecting to ${this.name} WebSocket:`, wsUrl);

        const socket = new WebSocket(wsUrl);
        if (this.binary_data) {
            socket.binaryType = 'arraybuffer';
        }
        this.ws = socket;

        socket.onopen = () => {
            console.log(`${this.name} WebSocket connected`);
            this.connected = true;
            this.reconnectAttempts = 0;
            if (this.onOpen) this.onOpen();
        };

        socket.onmessage = (event) => {
            if (this.onMessage) this.onMessage(event);
        };

        socket.onclose = () => {
            console.log(`${this.name} WebSocket disconnected`);
            this.connected = false;
            this.scheduleReconnect();
        };

        socket.onerror = (error) => {
            console.error(`${this.name} WebSocket error:`, error);
        };
    }

    scheduleReconnect() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error(`Max ${this.name} reconnection attempts reached`);
            return;
        }

        const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 10000); // Exponential backoff, max 10s
        this.reconnectAttempts++;

        console.log(`Reconnecting ${this.name} in ${delay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);

        setTimeout(() => this.connect(), delay);
    }

    ensureConnected() {
        if (!this.connected) {
            this.connect();
        }
    }
}
