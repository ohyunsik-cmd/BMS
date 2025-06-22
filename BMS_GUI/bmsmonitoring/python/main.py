import asyncio
import logging
import signal
import sys
import argparse
from typing import Optional

from can_processor import CANDataProcessor
from can_config import CANConfig
from websocket_client import WebSocketClient
from can_simulator import CANDataSimulator

class CANToWebSocketBridge:
    """CAN 데이터를 WebSocket으로 전송하는 메인 브리지 클래스"""
    
    def __init__(self, 
                 can_interface: str = 'serial',
                 can_channel: str = None,
                 websocket_uri: str = 'ws://localhost:8765',
                 simulation_mode: bool = False):
        
        self.can_config = CANConfig(can_interface, can_channel)
        self.can_processor = CANDataProcessor()
        self.websocket_client = WebSocketClient(websocket_uri)
        self.can_simulator = CANDataSimulator()
        
        self.simulation_mode = simulation_mode
        self.is_running = False
        self.logger = logging.getLogger(__name__)
        
        # 신호 처리 설정
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """신호 처리기"""
        self.logger.info(f"Received signal {signum}, shutting down...")
        self.stop()
    
    async def start(self):
        """브리지 시작"""
        self.logger.info("Starting CAN to WebSocket bridge...")
        self.is_running = True
        
        # WebSocket 연결
        if not await self.websocket_client.connect():
            self.logger.error("Failed to connect to WebSocket server")
            return False
        
        if self.simulation_mode:
            await self._run_simulation_mode()
        else:
            await self._run_can_mode()
        
        return True
    
    async def _run_can_mode(self):
        """실제 CAN 모드 실행"""
        self.logger.info("Running in CAN mode...")
        
        # CAN 버스 연결
        if not self.can_config.connect():
            self.logger.warning("Failed to connect to CAN bus, switching to simulation mode...")
            await self._run_simulation_mode()
            return
        
        self.logger.info(f"CAN bus connected: {self.can_config.get_info()}")
        
        try:
            while self.is_running:
                # CAN 메시지 수신
                msg = self.can_config.receive_message(timeout=1.0)
                
                if msg:
                    self.logger.debug(f"Received CAN message: ID=0x{msg.arbitration_id:03X}, Data={msg.data.hex()}")
                    
                    # CAN 메시지 처리
                    if self.can_processor.process_can_message(msg):
                        # 처리된 BMS 데이터를 WebSocket으로 전송
                        bms_data = self.can_processor.get_bms_data()
                        if await self.websocket_client.send_data(bms_data):
                            self.logger.info(f"BMS data sent: {bms_data['deviceId']}")
                        else:
                            self.logger.warning("Failed to send BMS data")
                
                await asyncio.sleep(0.01)  # CPU 사용률 조절
                
        except Exception as e:
            self.logger.error(f"Error in CAN mode: {e}")
        finally:
            self.can_config.disconnect()
    
    async def _run_simulation_mode(self):
        """시뮬레이션 모드 실행"""
        self.logger.info("Running in simulation mode...")
        
        def handle_simulated_message(msg):
            """시뮬레이션 메시지 처리"""
            if self.can_processor.process_can_message(msg):
                # 비동기 태스크로 WebSocket 전송
                asyncio.create_task(self._send_bms_data())
        
        # 시뮬레이션 실행
        await self.can_simulator.run_simulation(handle_simulated_message)
    
    async def _send_bms_data(self):
        """BMS 데이터 WebSocket 전송"""
        try:
            bms_data = self.can_processor.get_bms_data()
            if await self.websocket_client.send_data(bms_data):
                self.logger.debug(f"Simulation BMS data sent: {bms_data['deviceId']}")
        except Exception as e:
            self.logger.error(f"Error sending simulation data: {e}")
    
    def stop(self):
        """브리지 중지"""
        self.logger.info("Stopping CAN to WebSocket bridge...")
        self.is_running = False
        self.can_simulator.stop_simulation()
    
    async def cleanup(self):
        """정리 작업"""
        self.stop()
        await self.websocket_client.cleanup()
        self.can_config.disconnect()

def setup_logging(level: str = 'INFO'):
    """로깅 설정"""
    log_level = getattr(logging, level.upper())
    
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('can_bridge.log')
        ]
    )

def parse_arguments():
    """명령행 인수 파싱"""
    parser = argparse.ArgumentParser(description='CAN to WebSocket Bridge')
    
    parser.add_argument('--interface', '-i', 
                        choices=['serial', 'socketcan', 'pcan', 'seeedstudio'],
                        default='seeedstudio',
                        help='CAN interface type (default: seeedstudio)')
    
    parser.add_argument('--channel', '-c',
                        help='CAN channel (e.g., COM3, can0, 0)')
    
    parser.add_argument('--websocket-uri', '-w',
                        default='ws://localhost:8765',
                        help='WebSocket server URI (default: ws://localhost:8765)')
    
    parser.add_argument('--simulation', '-s',
                        action='store_true',
                        help='Run in simulation mode')
    
    parser.add_argument('--log-level', '-l',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                        default='INFO',
                        help='Log level (default: INFO)')
    
    return parser.parse_args()

async def main():
    """메인 함수"""
    args = parse_arguments()
    
    # 로깅 설정
    setup_logging(args.log_level)
    logger = logging.getLogger(__name__)
    
    logger.info("=== CAN to WebSocket Bridge Starting ===")
    logger.info(f"Interface: {args.interface}")
    logger.info(f"Channel: {args.channel}")
    logger.info(f"WebSocket URI: {args.websocket_uri}")
    logger.info(f"Simulation Mode: {args.simulation}")
    logger.info("=" * 45)
    
    # 브리지 생성 및 실행
    bridge = CANToWebSocketBridge(
        can_interface=args.interface,
        can_channel=args.channel,
        websocket_uri=args.websocket_uri,
        simulation_mode=args.simulation
    )
    
    try:
        await bridge.start()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        await bridge.cleanup()
        logger.info("Bridge stopped")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Program error: {e}")
        sys.exit(1)