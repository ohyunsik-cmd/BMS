import can
import logging
from typing import Optional, Dict, Any

class CANConfig:
    """CAN 버스 설정 및 관리 클래스"""
    
    # 지원되는 CAN 인터페이스 설정
    CAN_INTERFACES = {
        'socketcan': {
            'default_channel': 'can0',
            'description': 'Linux SocketCAN interface'
        },
        'pcan': {
            'default_channel': 0,
            'description': 'PEAK CAN interface'
        },
        'serial': {
            'default_channel': 'COM8',  # Windows 기본값
            'description': 'Serial CAN interface (USB-CAN)'
        },
        'seeedstudio': {
            'default_channel': 'COM8',  # Seeed Studio USB-CAN 기본값
            'description': 'Seeed Studio USB-CAN Analyzer'
        }
    }
    
    def __init__(self, interface_type: str = 'serial', channel: str = None, bitrate: int = 250000):
        self.interface_type = interface_type
        self.channel = channel or self.CAN_INTERFACES[interface_type]['default_channel']
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """CAN 버스 연결"""
        try:
            # Seeed Studio USB-CAN Analyzer의 경우 serial 인터페이스 사용
            if self.interface_type == 'seeedstudio':
                self.bus = can.Bus(
                    channel=self.channel,
                    bustype='serial',
                    bitrate=self.bitrate,
                    timeout=1.0
                )
            else:
                self.bus = can.Bus(
                    channel=self.channel,
                    bustype=self.interface_type,
                    bitrate=self.bitrate if self.interface_type != 'socketcan' else None
                )
            
            self.logger.info(f"CAN bus connected successfully - Interface: {self.interface_type}, Channel: {self.channel}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect CAN bus: {e}")
            return False
    
    def disconnect(self) -> None:
        """CAN 버스 연결 해제"""
        if self.bus:
            try:
                self.bus.shutdown()
                self.logger.info("CAN bus disconnected")
            except Exception as e:
                self.logger.error(f"Error disconnecting CAN bus: {e}")
            finally:
                self.bus = None
    
    def receive_message(self, timeout: float = 1.0) -> Optional[can.Message]:
        """CAN 메시지 수신"""
        if not self.bus:
            return None
        
        try:
            return self.bus.recv(timeout=timeout)
        except can.CanError as e:
            self.logger.error(f"CAN receive error: {e}")
            return None
    
    def send_message(self, can_id: int, data: bytes) -> bool:
        """CAN 메시지 송신"""
        if not self.bus:
            return False
        
        try:
            msg = can.Message(arbitration_id=can_id, data=data)
            self.bus.send(msg)
            self.logger.debug(f"Sent CAN message: ID=0x{can_id:03X}, Data={data.hex()}")
            return True
        except can.CanError as e:
            self.logger.error(f"CAN send error: {e}")
            return False
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.bus is not None
    
    def get_info(self) -> Dict[str, Any]:
        """CAN 설정 정보 반환"""
        return {
            'interface_type': self.interface_type,
            'channel': self.channel,
            'bitrate': self.bitrate,
            'connected': self.is_connected(),
            'description': self.CAN_INTERFACES.get(self.interface_type, {}).get('description', 'Unknown')
        }
    
    @classmethod
    def get_available_interfaces(cls) -> Dict[str, Dict[str, Any]]:
        """사용 가능한 CAN 인터페이스 목록 반환"""
        return cls.CAN_INTERFACES.copy()
    
    @classmethod
    def auto_detect_interface(cls) -> str:
        """자동으로 사용 가능한 CAN 인터페이스 감지"""
        import platform
        
        system = platform.system().lower()
        
        if system == 'linux':
            return 'socketcan'
        elif system == 'windows':
            # Windows에서는 serial 인터페이스 (USB-CAN) 우선
            return 'serial'
        else:
            return 'serial'  # 기본값