import React, { useState, useEffect, useRef, useCallback } from 'react';

// WebSocket 훅
const useWebSocket = (url, options = {}) => {
  const [connectionStatus, setConnectionStatus] = useState('연결 중...');
  const [lastMessage, setLastMessage] = useState(null);
  const [lastUpdate, setLastUpdate] = useState(null);
  
  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);
  const mountedRef = useRef(true);
  
  const {
    onConnect = () => {},
    onDisconnect = () => {},
    onMessage = () => {},
    onError = () => {},
    reconnectInterval = 5000,
    maxReconnectAttempts = 10
  } = options;
  
  const connect = useCallback(() => {
    if (!mountedRef.current) return;
    
    try {
      setConnectionStatus('연결 중...');
      
      // 기존 연결이 있다면 정리
      if (wsRef.current && wsRef.current.readyState !== WebSocket.CLOSED) {
        wsRef.current.close();
      }
      
      const ws = new WebSocket(url);
      wsRef.current = ws;
      
      ws.onopen = () => {
        if (!mountedRef.current) return;
        console.log('WebSocket connected');
        setConnectionStatus('연결됨');
        reconnectAttemptsRef.current = 0;
        onConnect();
        
        // 초기 데이터 요청
        try {
          ws.send(JSON.stringify({ type: 'get_data' }));
        } catch (error) {
          console.error('Error sending initial data request:', error);
        }
      };
      
      ws.onmessage = (event) => {
        if (!mountedRef.current) return;
        try {
          const message = JSON.parse(event.data);
          setLastMessage(message);
          setLastUpdate(new Date().toLocaleTimeString());
          onMessage(message);
        } catch (error) {
          console.error('Message parsing error:', error);
        }
      };
      
      ws.onclose = (event) => {
        if (!mountedRef.current) return;
        console.log('WebSocket closed:', event.code, event.reason);
        setConnectionStatus('연결 끊김');
        wsRef.current = null;
        onDisconnect(event);
        
        // 자동 재연결 (정상적인 종료가 아닌 경우만)
        if (event.code !== 1000 && reconnectAttemptsRef.current < maxReconnectAttempts) {
          reconnectTimeoutRef.current = setTimeout(() => {
            if (!mountedRef.current) return;
            reconnectAttemptsRef.current++;
            setConnectionStatus(`재연결 중... (${reconnectAttemptsRef.current}/${maxReconnectAttempts})`);
            connect();
          }, reconnectInterval);
        } else if (reconnectAttemptsRef.current >= maxReconnectAttempts) {
          setConnectionStatus('연결 실패');
        }
      };
      
      ws.onerror = (error) => {
        if (!mountedRef.current) return;
        console.error('WebSocket error:', error);
        setConnectionStatus('연결 오류');
        onError(error);
      };
      
    } catch (error) {
      console.error('WebSocket connection failed:', error);
      if (mountedRef.current) {
        setConnectionStatus('연결 실패');
      }
    }
  }, [url, onConnect, onDisconnect, onMessage, onError, reconnectInterval, maxReconnectAttempts]);
  
  const disconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    if (wsRef.current) {
      wsRef.current.close(1000, 'Manual disconnect');
    }
  }, []);
  
  const sendMessage = useCallback((message) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      try {
        wsRef.current.send(JSON.stringify(message));
        return true;
      } catch (error) {
        console.error('Error sending message:', error);
        return false;
      }
    }
    return false;
  }, []);
  
  const reconnect = useCallback(() => {
    disconnect();
    reconnectAttemptsRef.current = 0;
    setTimeout(() => {
      if (mountedRef.current) {
        connect();
      }
    }, 100);
  }, [disconnect, connect]);
  
  useEffect(() => {
    mountedRef.current = true;
    connect();
    
    return () => {
      mountedRef.current = false;
      disconnect();
    };
  }, [connect, disconnect]);
  
  return {
    connectionStatus,
    lastMessage,
    lastUpdate,
    sendMessage,
    reconnect,
    isConnected: connectionStatus === '연결됨'
  };
};

const BmsMonitoring = () => {
  // 초기 BMS 데이터
  const [bmsData, setBmsData] = useState({
    deviceId: 'Bms 01',
    packVoltage: 75, // 퍼센트로 표시
    cellVoltages: [3.7, 4.0, 3.7, 3.9, 4.3, 3.7, 4.0, 3.7, 3.9, 4.3],
    current: 12.5,
    temperatures: {
      ts1: 45,
      ts3: 47,
      intTemp: 48
    },
    protection: {
      ovp: false, // Over Voltage Protection
      uvp: false, // Under Voltage Protection
      ocp: false, // Over Current Protection
      otp: false, // Over Temperature Protection
      utp: false  // Under Temperature Protection
    },
    // 셀 밸런싱 상태 추가 (16비트 이진수를 사용하지만 하위 10비트만 활용)
    cellBalancing: 0b0000000000000110 // 예: 2, 3번 셀이 밸런싱 중
  });

  // WebSocket 연결
  const { connectionStatus, lastUpdate, reconnect, sendMessage, isConnected } = useWebSocket(
    'ws://localhost:8766',
    {
      onMessage: (message) => {
        try {
          if (message.type === 'bms_data' && message.data) {
            console.log('BMS 데이터 수신:', message.data);
            setBmsData(prevData => ({
              ...prevData,
              ...message.data
            }));
          } else if (message.type === 'pong') {
            console.log('Pong received from server');
          }
        } catch (error) {
          console.error('Error processing message:', error);
        }
      },
      onConnect: () => {
        console.log('Connected to BMS WebSocket server');
      },
      onDisconnect: () => {
        console.log('Disconnected from BMS WebSocket server');
      },
      onError: (error) => {
        console.error('WebSocket connection error:', error);
      }
    }
  );
  
  // 수동 데이터 요청
  const requestData = useCallback(() => {
    const success = sendMessage({ type: 'get_data' });
    if (!success) {
      console.warn('Failed to send data request - not connected');
    }
  }, [sendMessage]);
  
  // 주기적 핑
  useEffect(() => {
    if (!isConnected) return;
    
    const pingInterval = setInterval(() => {
      const success = sendMessage({ type: 'ping' });
      if (!success) {
        console.warn('Failed to send ping - connection lost');
      }
    }, 30000); // 30초마다 핑
    
    return () => clearInterval(pingInterval);
  }, [sendMessage, isConnected]);

  // 연결 상태 스타일
  const getConnectionStatusStyle = () => {
    switch (connectionStatus) {
      case '연결됨':
        return 'text-green-600 bg-green-100';
      case '연결 끊김':
      case '연결 실패':
      case '연결 오류':
        return 'text-red-600 bg-red-100';
      default:
        return 'text-yellow-600 bg-yellow-100';
    }
  };

  return (
    <div className="flex flex-col items-center w-full p-4 bg-gray-100 min-h-screen">
      <h1 className="text-2xl font-bold mb-6">BMS 모니터링 시스템</h1>
      
      {/* 연결 상태 표시 */}
      <div className="flex justify-center items-center space-x-4 mb-6">
        <div className={`px-3 py-1 rounded-full text-sm font-medium ${getConnectionStatusStyle()}`}>
          상태: {connectionStatus}
        </div>
        {lastUpdate && (
          <div className="text-sm text-gray-600">
            마지막 업데이트: {lastUpdate}
          </div>
        )}
        <button
          onClick={reconnect}
          className="px-3 py-1 bg-blue-500 text-white text-sm rounded hover:bg-blue-600 transition-colors"
        >
          재연결
        </button>
        <button
          onClick={requestData}
          className="px-3 py-1 bg-gray-500 text-white text-sm rounded hover:bg-gray-600 transition-colors"
        >
          데이터 요청
        </button>
      </div>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 w-full max-w-4xl">
        {/* 상단 왼쪽 패널 - 기기 ID와 배터리 상태 */}
        <div className="bg-white p-4 rounded-lg shadow">
          <div className="border-b pb-2 mb-4">
            <div className="flex justify-between items-center">
              <span className="font-semibold">Device ID :</span>
              <span>{bmsData.deviceId}</span>
            </div>
          </div>
          
          <div className="mb-4">
            <div className="flex items-center mb-2">
              <div className="mr-4">
                <BatteryIcon percentage={bmsData.packVoltage} />
              </div>
              <div className="flex-1">
                <div className="flex justify-between">
                  <span className="font-semibold">Pack voltage</span>
                  <span>{bmsData.packVoltage}%</span>
                </div>
              </div>
            </div>
          </div>
          
          <div>
            <h3 className="font-semibold mb-4 text-center">Protection</h3>
            <div className="grid grid-cols-5 gap-2 text-center">
              {[
                { key: 'ovp', label: 'OVP' },
                { key: 'uvp', label: 'UVP' },
                { key: 'ocp', label: 'OCP' },
                { key: 'otp', label: 'OTP' },
                { key: 'utp', label: 'UTP' }
              ].map(({ key, label }) => (
                <div key={key} className="flex flex-col items-center">
                  <div className="text-xs uppercase mb-2">{label}</div>
                  <StatusIndicator active={bmsData.protection[key]} />
                </div>
              ))}
            </div>
          </div>
        </div>
        
        {/* 상단 오른쪽 패널 - 셀 전압 그래프 */}
        <div className="bg-white p-4 rounded-lg shadow h-64">
          <CellVoltageGraph 
            cellVoltages={bmsData.cellVoltages} 
            cellBalancing={bmsData.cellBalancing}
          />
        </div>
        
        {/* 셀 밸런싱 상태 표시 패널 */}
        <div className="bg-white p-4 rounded-lg shadow">
          <h3 className="font-semibold mb-2 text-center">셀 밸런싱 상태</h3>
          <CellBalancingStatus cellBalancing={bmsData.cellBalancing} />
        </div>
        
        {/* 전류 게이지 */}
        <div className="bg-white p-4 rounded-lg shadow">
          <h3 className="font-semibold mb-2 text-center">Pack Current</h3>
          <CurrentGauge value={bmsData.current} />
        </div>
        
        {/* 온도 표시 */}
        <div className="bg-white p-4 rounded-lg shadow col-span-1 md:col-span-2">
          <h3 className="font-semibold mb-2 text-center">Temperature</h3>
          <div className="flex justify-around">
            <TemperatureDisplay temp={bmsData.temperatures.ts1} label="TS1" />
            <TemperatureDisplay temp={bmsData.temperatures.ts3} label="TS3" />
            <TemperatureDisplay temp={bmsData.temperatures.intTemp} label="IntTemp" />
          </div>
        </div>
      </div>
    </div>
  );
};

// 셀 밸런싱 상태 표시 컴포넌트
const CellBalancingStatus = ({ cellBalancing }) => {
  // 현재 밸런싱 중인 셀 번호 목록 추출
  const getBalancingCells = () => {
    const cells = [];
    for (let i = 0; i < 10; i++) {
      if ((cellBalancing & (1 << i)) !== 0) {
        cells.push(i + 1); // 1번부터 시작하는 셀 번호
      }
    }
    return cells;
  };
  
  const balancingCells = getBalancingCells();
  const isBalancing = balancingCells.length > 0;
  
  return (
    <div className="flex flex-col items-center">
      <div className="mb-4 text-center">
        {isBalancing ? (
          <div className="text-indigo-600 font-semibold">
            {balancingCells.join(', ')}번 셀 밸런싱 중
          </div>
        ) : (
          <div className="text-gray-500">밸런싱 중인 셀 없음</div>
        )}
      </div>
      
      <div className="w-full">
        <div className="grid grid-cols-5 gap-1 mb-2">
          {Array.from({ length: 10 }).map((_, index) => {
            const isActive = (cellBalancing & (1 << index)) !== 0;
            return (
              <div key={index} className="flex flex-col items-center">
                <div className={`w-8 h-8 rounded-full flex items-center justify-center text-xs
                  ${isActive ? 'bg-indigo-500 text-white animate-pulse' : 'bg-gray-200 text-gray-700'}`}>
                  {index + 1}
                </div>
              </div>
            );
          })}
        </div>
      </div>
      
      {isBalancing && (
        <div className="mt-4">
          <div className="w-20 h-20 flex items-center justify-center">
            <svg className="animate-spin" width="40" height="40" viewBox="0 0 24 24">
              <path
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                d="M12 6V2m0 20v-4m4-8h4M2 12h4m11.5-3.5L19.1 7M4.9 20.1l1.5-1.5m0-11.1L4.9 6m14.2 12.1l-1.5-1.5"
              />
            </svg>
          </div>
        </div>
      )}
    </div>
  );
};

// 배터리 아이콘 컴포넌트
const BatteryIcon = ({ percentage }) => {
  // 배터리 색상 결정 (75% 이상 녹색, 25% 이하 빨간색, 그 외 노란색)
  const getBatteryColor = () => {
    if (percentage >= 75) return 'bg-green-500';
    if (percentage <= 25) return 'bg-red-500';
    return 'bg-yellow-500';
  };

  return (
    <div className="relative w-16 h-8 border-2 border-gray-800 rounded-sm">
      {/* 배터리 노브 */}
      <div className="absolute -right-2 top-1/2 transform -translate-y-1/2 w-1 h-4 bg-gray-800"></div>
      {/* 배터리 레벨 */}
      <div 
        className={`absolute top-0.5 left-0.5 bottom-0.5 ${getBatteryColor()}`} 
        style={{ width: `${Math.max(0, Math.min(100, percentage))}%` }}
      ></div>
    </div>
  );
};

// 상태 표시 컴포넌트 (원형 LED)
const StatusIndicator = ({ active }) => {
  return (
    <div className={`w-6 h-6 rounded-full ${active ? 'bg-red-500' : 'bg-green-500'}`}></div>
  );
};

// 셀 전압 그래프 컴포넌트 - 셀 밸런싱 표시 기능 추가
const CellVoltageGraph = ({ cellVoltages, cellBalancing }) => {
  return (
    <div className="h-full">
      <div className="h-full flex flex-col">
        {/* 바 그래프 영역 */}
        <div className="flex-1 flex mb-2">
          {cellVoltages.map((voltage, index) => {
            // 전압에 따른 높이 계산 (차트 영역의 30-100% 사이)
            const height = 30 + (voltage - 3.0) / (4.5 - 3.0) * 70;
            // 셀 밸런싱 중인지 확인
            const isBalancing = (cellBalancing & (1 << index)) !== 0;
            
            return (
              <div key={index} className="flex-1 flex flex-col justify-end items-center mx-0.5">
                <div 
                  className={`w-full ${isBalancing ? 'bg-indigo-600 animate-pulse' : 'bg-indigo-400'} rounded-t`}
                  style={{ height: `${height}%` }}
                ></div>
              </div>
            );
          })}
        </div>
        
        {/* 라벨 영역 - 각 바 그래프와 동일한 크기로 유지 */}
        <div className="flex">
          {cellVoltages.map((voltage, index) => {
            const isBalancing = (cellBalancing & (1 << index)) !== 0;
            return (
              <div key={index} className="flex-1 text-xs px-0.5 text-center">
                <div className={isBalancing ? 'text-indigo-600 font-bold' : ''}>
                  Cell{index + 1}
                </div>
                <div>{voltage.toFixed(1)}V</div>
              </div>
            );
          })}
        </div>
      </div>
    </div>
  );
};

// 전류 게이지 컴포넌트 - 직선형 게이지
const CurrentGauge = ({ value }) => {
  // value는 -100에서 100 사이의 값
  const normalizedValue = Math.max(-500, Math.min(500, value)); // 값 범위 제한
  // 위치 계산 (게이지 내 상대적 위치: 0% = -100, 50% = 0, 100% = 100)
  const position = ((normalizedValue + 500) / 1000) * 100;
  
  return (
    <div className="flex flex-col items-center">
      <div className="relative w-64 h-24">
        {/* 직선형 게이지 */}
        <svg width="100%" height="100%" viewBox="0 0 200 60">
          {/* 배경 라인 */}
          <rect x="10" y="30" width="180" height="10" rx="5" fill="#ddd" />
          
          {/* 구간 색상 표시 */}
          <rect x="10" y="30" width="45" height="10" rx="5" fill="#f44336" /> {/* 빨간색 (-100 ~ -50) */}
          <rect x="55" y="30" width="45" height="10" rx="0" fill="#ff9800" /> {/* 주황색 (-50 ~ 0) */}
          <rect x="100" y="30" width="45" height="10" rx="0" fill="#8bc34a" /> {/* 연두색 (0 ~ 50) */}
          <rect x="145" y="30" width="45" height="10" rx="5" fill="#4caf50" /> {/* 녹색 (50 ~ 100) */}
          
          {/* 눈금 표시 */}
          <g stroke="#666" strokeWidth="1">
            <line x1="10" y1="50" x2="10" y2="40" />
            <line x1="55" y1="50" x2="55" y2="40" />
            <line x1="100" y1="50" x2="100" y2="40" />
            <line x1="145" y1="50" x2="145" y2="40" />
            <line x1="190" y1="50" x2="190" y2="40" />
            
            {/* 중간 눈금 */}
            <line x1="32.5" y1="48" x2="32.5" y2="40" />
            <line x1="77.5" y1="48" x2="77.5" y2="40" />
            <line x1="122.5" y1="48" x2="122.5" y2="40" />
            <line x1="167.5" y1="48" x2="167.5" y2="40" />
          </g>
          
          {/* 눈금 값 */}
          <text x="10" y="58" textAnchor="middle" fontSize="10">-500</text>
          <text x="55" y="58" textAnchor="middle" fontSize="10">-250</text>
          <text x="100" y="58" textAnchor="middle" fontSize="10">0</text>
          <text x="145" y="58" textAnchor="middle" fontSize="10">250</text>
          <text x="190" y="58" textAnchor="middle" fontSize="10">500</text>
          
          {/* 현재값 표시기 (삼각형 마커) */}
          <polygon 
            points="10,45 17,35 3,35" 
            fill="#2c3e50"
            transform={`translate(${position * 1.8}, 0)`}
          />
          
          {/* 현재값 라인 */}
          <line 
            x1={10 + position * 1.8} 
            y1="35" 
            x2={10 + position * 1.8} 
            y2="40"
            stroke="#2c3e50" 
            strokeWidth="2" 
          />
        </svg>
      </div>
      
      {/* 숫자 표시 */}
      <div className="text-xl font-bold mt-2">
        {value.toFixed(1)}mA
      </div>
    </div>
  );
};

// 온도 표시 컴포넌트
const TemperatureDisplay = ({ temp, label }) => {
  // 온도계 색상 (온도에 따라 변경)
  const getColor = () => {
    if (temp >= 50) return '#e53e3e'; // 빨강
    if (temp >= 20) return '#ed8936'; // 주황
    return '#3182ce'; // 파랑
  };
  
  // 온도계 높이 계산 (0-100°C를 0-100%로 매핑)
  const heightPercent = Math.max(5, Math.min(60, temp));
  
  return (
    <div className="flex flex-col items-center">
      <div className="relative h-32 w-16 flex justify-center">
        <div className="absolute bottom-0 w-4 h-24 bg-gray-200 rounded-t-full overflow-hidden">
          {/* 온도계 내부 액체 */}
          <div 
            className="absolute bottom-0 w-full rounded-t-full" 
            style={{ 
              backgroundColor: getColor(),
              height: `${heightPercent * 1.5}%`
            }}
          ></div>
        </div>
        
        {/* 온도계 외관 */}
        <div className="absolute bottom-0 w-6 h-28">
          {/* 온도계 구부 */}
          <div className="absolute bottom-0 left-1/2 transform -translate-x-1/2 w-6 h-6 rounded-full border-2 border-gray-400 bg-gray-200"></div>
          {/* 온도계 줄기 */}
          <div className="absolute bottom-3 left-1/2 transform -translate-x-1/2 w-2 h-24 rounded-t-full border-2 border-gray-400 bg-gray-100"></div>
        </div>
      </div>
      
      <div className="text-sm font-medium mt-1">{label}</div>
      <div>{temp.toFixed(1)}°C</div>
    </div>
  );
};

export default BmsMonitoring;