import asyncio
import websockets
import json
import logging
from typing import Dict, Any, Optional, Callable

class WebSocketClient:
    """WebSocket 클라이언트 관리 클래스"""
    
    def __init__(self, uri: str = "ws://localhost:8765", reconnect_interval: int = 5):
        self.uri = uri
        self.reconnect_interval = reconnect_interval
        self.websocket: Optional[websockets.WebSocketServerProtocol] = None
        self.is_running = False
        self.logger = logging.getLogger(__name__)
        self.connection_callbacks = []
        self.disconnection_callbacks = []
        
    def add_connection_callback(self, callback: Callable) -> None:
        """연결 시 호출할 콜백 함수 추가"""
        self.connection_callbacks.append(callback)
    
    def add_disconnection_callback(self, callback: Callable) -> None:
        """연결 해제 시 호출할 콜백 함수 추가"""
        self.disconnection_callbacks.append(callback)
    
    async def connect(self) -> bool:
        """WebSocket 서버에 연결"""
        try:
            self.websocket = await websockets.connect(self.uri)
            self.logger.info(f"WebSocket connected to {self.uri}")
            
            # 연결 콜백 실행
            for callback in self.connection_callbacks:
                try:
                    callback()
                except Exception as e:
                    self.logger.error(f"Connection callback error: {e}")
            
            return True
        except Exception as e:
            self.logger.error(f"WebSocket connection failed: {e}")
            return False
    
    async def disconnect(self) -> None:
        """WebSocket 연결 해제"""
        if self.websocket:
            try:
                await self.websocket.close()
                self.logger.info("WebSocket disconnected")
                
                # 연결 해제 콜백 실행
                for callback in self.disconnection_callbacks:
                    try:
                        callback()
                    except Exception as e:
                        self.logger.error(f"Disconnection callback error: {e}")
                        
            except Exception as e:
                self.logger.error(f"Error disconnecting WebSocket: {e}")
            finally:
                self.websocket = None
    
    async def send_data(self, data: Dict[str, Any]) -> bool:
        """데이터 전송"""
        if not self.websocket:
            self.logger.warning("WebSocket not connected, cannot send data")
            return False
        
        try:
            json_data = json.dumps(data)
            await self.websocket.send(json_data)
            self.logger.debug(f"Data sent: {json_data}")
            return True
        except websockets.exceptions.ConnectionClosed:
            self.logger.warning("WebSocket connection closed during send")
            self.websocket = None
            return False
        except Exception as e:
            self.logger.error(f"Error sending data: {e}")
            return False
    
    async def receive_data(self) -> Optional[Dict[str, Any]]:
        """데이터 수신"""
        if not self.websocket:
            return None
        
        try:
            message = await self.websocket.recv()
            return json.loads(message)
        except websockets.exceptions.ConnectionClosed:
            self.logger.warning("WebSocket connection closed during receive")
            self.websocket = None
            return None
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Error receiving data: {e}")
            return None
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.websocket is not None and not self.websocket.closed
    
    async def run_with_auto_reconnect(self, data_handler: Callable[[Dict[str, Any]], None] = None) -> None:
        """자동 재연결 기능이 있는 실행"""
        self.is_running = True
        
        while self.is_running:
            try:
                if not self.is_connected():
                    self.logger.info("Attempting to connect...")
                    if await self.connect():
                        self.logger.info("Connected successfully")
                    else:
                        self.logger.warning(f"Connection failed, retrying in {self.reconnect_interval} seconds...")
                        await asyncio.sleep(self.reconnect_interval)
                        continue
                
                # 연결된 상태에서 데이터 처리
                if data_handler:
                    data = await self.receive_data()
                    if data:
                        data_handler(data)
                
                await asyncio.sleep(0.1)  # CPU 사용률 조절
                
            except websockets.exceptions.ConnectionClosed:
                self.logger.warning("WebSocket connection lost, attempting to reconnect...")
                self.websocket = None
                await asyncio.sleep(self.reconnect_interval)
                
            except Exception as e:
                self.logger.error(f"Unexpected error in WebSocket loop: {e}")
                await asyncio.sleep(self.reconnect_interval)
    
    def stop(self) -> None:
        """실행 중지"""
        self.is_running = False
    
    async def cleanup(self) -> None:
        """정리 작업"""
        self.stop()
        await self.disconnect()