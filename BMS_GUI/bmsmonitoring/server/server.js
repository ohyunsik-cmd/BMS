const WebSocket = require('ws');
const express = require('express');
const http = require('http');
const cors = require('cors');
const { EventEmitter } = require('events');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

class BMSWebSocketServer extends EventEmitter {
    constructor(options = {}) {
        super();
        
        // 설정
        this.config = {
            clientPort: options.clientPort || 8766,
            httpPort: options.httpPort || 3001,
            reconnectInterval: options.reconnectInterval || 5000,
            simulationInterval: options.simulationInterval || 2000,
            // UART 설정
            serialPath: options.serialPath || null, // 자동 감지
            baudRate: options.baudRate || 115200,
            ...options
        };
        
        // Express 앱 및 서버
        this.app = express();
        this.httpServer = http.createServer(this.app);
        
        // WebSocket 서버
        this.clientWss = null;
        
        // 시리얼 포트
        this.serialPort = null;
        this.parser = null;
        
        // 데이터 및 연결 관리
        this.connectedClients = new Set();
        this.latestBmsData = this.getDefaultBmsData();
        this.simulationInterval = null;
        this.serialConnected = false;
        
        this.setupExpress();
        this.setupWebSocketServers();
        this.setupSerial();
        
        console.log('BMS WebSocket Server with UART initialized');
    }
    
    getDefaultBmsData() {
        return {
            deviceId: 'STM32_BMS',
            packVoltage: 75,
            cellVoltages: [3.7, 4.0, 3.7, 3.9, 4.3, 3.7, 4.0, 3.7, 3.9, 4.3],
            current: 12.5,
            temperatures: {
                ts1: 45,
                ts3: 47,
                intTemp: 48
            },
            protection: {
                ovp: false,
                uvp: false,
                ocp: false,
                otp: false,
                utp: false
            },
            cellBalancing: 0,
            timestamp: Date.now()
        };
    }
    
    async findSTM32Port() {
        try {
            const ports = await SerialPort.list();
            console.log('Available serial ports:');
            ports.forEach(port => {
                console.log(`  ${port.path} - ${port.manufacturer || 'Unknown'} (${port.vendorId}:${port.productId})`);
            });
            
            // ST-Link V3를 찾기 (STM32H723ZG Nucleo 보드)
            const stLinkPort = ports.find(port => 
                port.manufacturer && (
                    port.manufacturer.includes('STMicroelectronics') ||
                    port.manufacturer.includes('ST-LINK') ||
                    port.vendorId === '0483' // ST Microelectronics Vendor ID
                )
            );
            
            if (stLinkPort) {
                console.log(`Found STM32 port: ${stLinkPort.path}`);
                return stLinkPort.path;
            }
            
            // 윈도우에서는 보통 COM 포트로 나타남
            const comPort = ports.find(port => 
                port.path.startsWith('COM') && 
                (port.manufacturer && port.manufacturer.includes('ST'))
            );
            
            if (comPort) {
                console.log(`Found COM port: ${comPort.path}`);
                return comPort.path;
            }
            
            console.warn('STM32 port not found automatically. Please specify manually.');
            return null;
            
        } catch (error) {
            console.error('Error listing serial ports:', error);
            return null;
        }
    }
    
    async setupSerial() {
        try {
            // 포트 자동 감지 또는 수동 설정
            const portPath = this.config.serialPath || await this.findSTM32Port();
            
            if (!portPath) {
                console.warn('No serial port specified. Starting in simulation mode.');
                this.startSimulation();
                return;
            }
            
            console.log(`Attempting to connect to ${portPath} at ${this.config.baudRate} baud`);
            
            this.serialPort = new SerialPort({
                path: portPath,
                baudRate: this.config.baudRate,
                dataBits: 8,
                parity: 'none',
                stopBits: 1,
                autoOpen: false
            });
            
            // 줄바꿈 문자로 구분되는 파서 설정
            this.parser = this.serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));
            
            // 이벤트 핸들러 설정
            this.serialPort.on('open', () => {
                console.log(`Serial port ${portPath} opened successfully`);
                this.serialConnected = true;
                this.stopSimulation();
                this.emit('serialConnected');
            });
            
            this.serialPort.on('close', () => {
                console.log('Serial port closed');
                this.serialConnected = false;
                this.emit('serialDisconnected');
                this.startReconnectTimer();
            });
            
            this.serialPort.on('error', (error) => {
                console.error('Serial port error:', error);
                this.serialConnected = false;
                this.startReconnectTimer();
            });
            
            // 데이터 수신 처리
            this.parser.on('data', (data) => {
                this.handleSerialData(data.toString().trim());
            });
            
            // 포트 열기
            this.serialPort.open((error) => {
                if (error) {
                    console.error('Failed to open serial port:', error);
                    this.startReconnectTimer();
                } else {
                    // STM32에 초기 명령 전송 (필요시)
                    setTimeout(() => {
                        this.sendSerialCommand('START');
                    }, 1000);
                }
            });
            
        } catch (error) {
            console.error('Serial setup error:', error);
            this.startReconnectTimer();
        }
    }
    
    handleSerialData(data) {
        try {
            console.log('Received serial data:', data);
            
            // JSON 형태로 수신되는 경우
            if (data.startsWith('{') && data.endsWith('}')) {
                const bmsData = JSON.parse(data);
                this.processBmsData(bmsData);
                return;
            }
            
            // CSV 형태로 수신되는 경우 (예: "DEVICE_ID,PACK_VOLTAGE,CURRENT,TEMP1,TEMP2,...")
            if (data.includes(',')) {
                const bmsData = this.parseCSVData(data);
                if (bmsData) {
                    this.processBmsData(bmsData);
                }
                return;
            }
            
            // 단순 키-값 형태 (예: "PACK_VOLTAGE:75.2")
            if (data.includes(':')) {
                this.parseKeyValueData(data);
                return;
            }
            
            // 기타 형태의 데이터 처리 (필요시 확장)
            console.log('Unrecognized data format:', data);
            
        } catch (error) {
            console.error('Error parsing serial data:', error, 'Data:', data);
        }
    }
    
    parseCSVData(data) {
        try {
            const values = data.split(',');
            
            // STM32에서 보내는 데이터 형태에 맞게 조정
            // 예: "STM32_BMS,75.2,12.5,45.1,47.2,48.0,0,0,0,0,0,3.7,4.0,3.7,3.9,4.3,3.7,4.0,3.7,3.9,4.3"
            if (values.length >= 21) {
                return {
                    deviceId: values[0],
                    packVoltage: parseFloat(values[1]),
                    current: parseFloat(values[2]),
                    temperatures: {
                        ts1: parseFloat(values[3]),
                        ts3: parseFloat(values[4]),
                        intTemp: parseFloat(values[5])
                    },
                    protection: {
                        ovp: parseInt(values[6]) === 1,
                        uvp: parseInt(values[7]) === 1,
                        ocp: parseInt(values[8]) === 1,
                        otp: parseInt(values[9]) === 1,
                        utp: parseInt(values[10]) === 1
                    },
                    cellVoltages: values.slice(11, 21).map(v => parseFloat(v)),
                    cellBalancing: values[21] ? parseInt(values[21]) : 0,
                    timestamp: Date.now()
                };
            }
            
            return null;
        } catch (error) {
            console.error('CSV parsing error:', error);
            return null;
        }
    }
    
    parseKeyValueData(data) {
        const [key, value] = data.split(':');
        
        switch (key.toUpperCase()) {
            case 'PACK_VOLTAGE':
                this.latestBmsData.packVoltage = parseFloat(value);
                break;
            case 'CURRENT':
                this.latestBmsData.current = parseFloat(value);
                break;
            case 'TEMP1':
                this.latestBmsData.temperatures.ts1 = parseFloat(value);
                break;
            case 'TEMP2':
                this.latestBmsData.temperatures.ts3 = parseFloat(value);
                break;
            case 'TEMP3':
                this.latestBmsData.temperatures.intTemp = parseFloat(value);
                break;
            default:
                console.log('Unknown key-value data:', key, value);
        }
        
        // 데이터 업데이트 후 브로드캐스트
        this.latestBmsData.timestamp = Date.now();
        this.broadcastToClients(this.latestBmsData);
    }
    
    processBmsData(bmsData) {
        console.log('Processing BMS data from STM32:', bmsData.deviceId || 'Unknown');
        
        // 타임스탬프 추가
        bmsData.timestamp = Date.now();
        
        // 데이터 검증
        if (this.validateBmsData(bmsData)) {
            // 최신 데이터 업데이트
            this.latestBmsData = { ...this.latestBmsData, ...bmsData };
            
            // 모든 클라이언트에게 브로드캐스트
            this.broadcastToClients(this.latestBmsData);
            
            this.emit('dataReceived', this.latestBmsData);
        } else {
            console.warn('Invalid BMS data received from STM32:', bmsData);
        }
    }
    
    sendSerialCommand(command) {
        if (this.serialPort && this.serialPort.isOpen) {
            this.serialPort.write(command + '\n', (error) => {
                if (error) {
                    console.error('Error sending serial command:', error);
                } else {
                    console.log('Sent command to STM32:', command);
                }
            });
        }
    }
    
    startReconnectTimer() {
        if (!this.serialConnected) {
            console.log('Starting simulation mode while attempting serial reconnection...');
            this.startSimulation();
            
            setTimeout(() => {
                console.log('Attempting to reconnect to serial port...');
                this.setupSerial();
            }, this.config.reconnectInterval);
        }
    }
    
    setupExpress() {
        // CORS 및 미들웨어 설정
        this.app.use(cors());
        this.app.use(express.json());
        
        // REST API 엔드포인트
        this.app.get('/api/bms/current', (req, res) => {
            res.json({
                success: true,
                data: this.latestBmsData
            });
        });
        
        this.app.get('/api/bms/status', (req, res) => {
            res.json({
                success: true,
                status: {
                    serialConnected: this.serialConnected,
                    clientsConnected: this.connectedClients.size,
                    lastUpdate: this.latestBmsData.timestamp,
                    simulationActive: this.simulationInterval !== null,
                    serialPort: this.serialPort ? this.serialPort.path : null
                }
            });
        });
        
        // 시리얼 포트 목록 조회
        this.app.get('/api/serial/ports', async (req, res) => {
            try {
                const ports = await SerialPort.list();
                res.json({
                    success: true,
                    ports: ports.map(port => ({
                        path: port.path,
                        manufacturer: port.manufacturer,
                        vendorId: port.vendorId,
                        productId: port.productId
                    }))
                });
            } catch (error) {
                res.status(500).json({
                    success: false,
                    error: error.message
                });
            }
        });
        
        // STM32 명령 전송
        this.app.post('/api/serial/command', (req, res) => {
            const { command } = req.body;
            if (!command) {
                return res.status(400).json({
                    success: false,
                    error: 'Command is required'
                });
            }
            
            this.sendSerialCommand(command);
            res.json({
                success: true,
                message: `Command "${command}" sent to STM32`
            });
        });
        
        // 설정 API
        this.app.get('/api/config', (req, res) => {
            res.json({
                success: true,
                config: this.config
            });
        });
        
        // 헬스체크
        this.app.get('/health', (req, res) => {
            res.json({ 
                status: 'OK', 
                timestamp: Date.now(),
                uptime: process.uptime(),
                serialConnected: this.serialConnected
            });
        });
        
        // 정적 파일 서빙 (선택사항)
        this.app.use(express.static('public'));
    }
    
    setupWebSocketServers() {
        // React 클라이언트용 WebSocket 서버
        this.clientWss = new WebSocket.Server({ port: this.config.clientPort });
        
        this.clientWss.on('connection', (ws) => {
            console.log('React client connected');
            this.connectedClients.add(ws);
            
            // 연결 즉시 최신 데이터 전송
            this.sendToClient(ws, {
                type: 'bms_data',
                data: this.latestBmsData
            });
            
            ws.on('message', (message) => {
                this.handleClientMessage(ws, message);
            });
            
            ws.on('close', () => {
                console.log('React client disconnected');
                this.connectedClients.delete(ws);
            });
            
            ws.on('error', (error) => {
                console.error('Client WebSocket error:', error);
                this.connectedClients.delete(ws);
            });
        });
    }
    
    handleClientMessage(ws, message) {
        try {
            const clientMessage = JSON.parse(message.toString());
            console.log('Client message:', clientMessage.type);
            
            switch (clientMessage.type) {
                case 'get_data':
                    this.sendToClient(ws, {
                        type: 'bms_data',
                        data: this.latestBmsData
                    });
                    break;
                    
                case 'ping':
                    this.sendToClient(ws, {
                        type: 'pong',
                        timestamp: Date.now(),
                        serialConnected: this.serialConnected
                    });
                    break;
                    
                case 'send_command':
                    if (clientMessage.command) {
                        this.sendSerialCommand(clientMessage.command);
                        this.sendToClient(ws, {
                            type: 'command_sent',
                            command: clientMessage.command
                        });
                    }
                    break;
                    
                case 'subscribe':
                    // 구독 로직 (필요시 구현)
                    break;
                    
                default:
                    console.warn('Unknown client message type:', clientMessage.type);
            }
            
        } catch (error) {
            console.error('Client message parsing error:', error);
        }
    }
    
    validateBmsData(data) {
        // 기본적인 BMS 데이터 검증
        if (!data || typeof data !== 'object') return false;
        
        
        // 데이터 타입 및 범위 검증
        if ('packVoltage' in data) {
            if (typeof data.packVoltage !== 'number' || data.packVoltage < 0 || data.packVoltage > 1000) {
                console.warn('Invalid pack voltage:', data.packVoltage);
                return false;
            }
        }
        
        if ('current' in data) {
            if (typeof data.current !== 'number') {
                console.warn('Invalid current:', data.current);
                return false;
            }
        }
        
        return true;
    }
    
    sendToClient(ws, data) {
        if (ws.readyState === WebSocket.OPEN) {
            try {
                ws.send(JSON.stringify(data));
                return true;
            } catch (error) {
                console.error('Error sending to client:', error);
                this.connectedClients.delete(ws);
                return false;
            }
        }
        return false;
    }
    
    broadcastToClients(data) {
        const message = JSON.stringify({
            type: 'bms_data',
            data: data
        });
        
        let successCount = 0;
        
        this.connectedClients.forEach(client => {
            if (this.sendToClient(client, { type: 'bms_data', data: data })) {
                successCount++;
            }
        });
        
        if (successCount > 0) {
            console.log(`Broadcasted data to ${successCount}/${this.connectedClients.size} clients`);
        }
        return successCount;
    }
    
    startSimulation() {
        if (this.simulationInterval || this.serialConnected) return;
        
        console.log('Starting simulation mode...');
        
        this.simulationInterval = setInterval(() => {
            // 가상 BMS 데이터 생성
            const simulatedData = this.generateSimulatedData();
            
            this.latestBmsData = simulatedData;
            this.broadcastToClients(simulatedData);
            
        }, this.config.simulationInterval);
        
        this.emit('simulationStarted');
    }
    
    stopSimulation() {
        if (this.simulationInterval) {
            clearInterval(this.simulationInterval);
            this.simulationInterval = null;
            console.log('Simulation mode stopped');
            this.emit('simulationStopped');
        }
    }
    
    generateSimulatedData() {
        const baseData = this.latestBmsData;
        
        return {
            deviceId: 'STM32_BMS_SIM',
            packVoltage: Math.max(0, Math.min(100, 
                baseData.packVoltage + (Math.random() * 4 - 2))),
            cellVoltages: baseData.cellVoltages.map(v => 
                Math.max(3.0, Math.min(4.5, v + (Math.random() * 0.2 - 0.1)))),
            current: baseData.current + (Math.random() * 10 - 5),
            temperatures: {
                ts1: Math.max(0, Math.min(80, baseData.temperatures.ts1 + (Math.random() * 4 - 2))),
                ts3: Math.max(0, Math.min(80, baseData.temperatures.ts3 + (Math.random() * 4 - 2))),
                intTemp: Math.max(0, Math.min(80, baseData.temperatures.intTemp + (Math.random() * 4 - 2)))
            },
            protection: {
                ovp: Math.random() > 0.95,
                uvp: Math.random() > 0.95,
                ocp: Math.random() > 0.95,
                otp: Math.random() > 0.95,
                utp: Math.random() > 0.95
            },
            cellBalancing: Math.random() > 0.7 ? Math.floor(Math.random() * 1024) : baseData.cellBalancing,
            timestamp: Date.now()
        };
    }
    
    start() {
        return new Promise((resolve, reject) => {
            this.httpServer.listen(this.config.httpPort, (error) => {
                if (error) {
                    reject(error);
                    return;
                }
                
                console.log('=== BMS WebSocket Server with UART Started ===');
                console.log(`HTTP Server: http://localhost:${this.config.httpPort}`);
                console.log(`Client WebSocket: ws://localhost:${this.config.clientPort}`);
                console.log(`Serial Port: ${this.config.serialPath || 'Auto-detect'}`);
                console.log(`Baud Rate: ${this.config.baudRate}`);
                console.log('=============================================');
                
                // 연결 상태 모니터링 시작
                this.startStatusMonitoring();
                
                resolve();
            });
        });
    }
    
    startStatusMonitoring() {
        setInterval(() => {
            console.log('=== Connection Status ===');
            console.log(`Serial connected: ${this.serialConnected}`);
            console.log(`Client connections: ${this.connectedClients.size}`);
            console.log(`Simulation active: ${this.simulationInterval !== null}`);
            console.log('========================');
        }, 30000);
    }
    
    stop() {
        return new Promise((resolve) => {
            console.log('Stopping BMS WebSocket Server...');
            
            this.stopSimulation();
            
            if (this.serialPort && this.serialPort.isOpen) {
                this.serialPort.close();
            }
            
            if (this.clientWss) {
                this.clientWss.close();
            }
            
            this.httpServer.close(() => {
                console.log('BMS WebSocket Server stopped');
                resolve();
            });
        });
    }
}

// 서버 인스턴스 생성 및 시작
const server = new BMSWebSocketServer({
    clientPort: process.env.CLIENT_WS_PORT || 8766,
    httpPort: process.env.HTTP_PORT || 3001,
    serialPath: process.env.SERIAL_PORT || null, // 자동 감지
    baudRate: parseInt(process.env.BAUD_RATE) || 115200
});

// 이벤트 리스너
server.on('dataReceived', (data) => {
    console.log(`Data received from ${data.deviceId} at ${new Date(data.timestamp).toLocaleTimeString()}`);
});

server.on('serialConnected', () => {
    console.log('STM32 serial connection established');
});

server.on('serialDisconnected', () => {
    console.log('STM32 serial connection lost');
});

server.on('simulationStarted', () => {
    console.log('Simulation mode activated (no STM32 connection)');
});

// 서버 시작
server.start().catch(console.error);

// 종료 시그널 처리
process.on('SIGINT', async () => {
    console.log('\nReceived SIGINT, shutting down gracefully...');
    await server.stop();
    process.exit(0);
});

process.on('SIGTERM', async () => {
    console.log('\nReceived SIGTERM, shutting down gracefully...');
    await server.stop();
    process.exit(0);
});

module.exports = BMSWebSocketServer;