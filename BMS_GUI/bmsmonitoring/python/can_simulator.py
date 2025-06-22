import can
import struct
import random
import asyncio
import logging
from typing import List, Dict, Any

class CANDataSimulator:
    """CAN 데이터 시뮬레이션 클래스"""
    
    def __init__(self, update_interval: float = 1.0):
        self.update_interval = update_interval
        self.is_running = False
        self.logger = logging.getLogger(__name__)
        
        # 시뮬레이션 데이터 범위 설정
        self.simulation_config = {
            'pack_voltage_range': (36.0, 42.0),  # V
            'current_range': (-50.0, 50.0),     # A
            'cell_voltage_range': (3.0, 4.5),   # V
            'temperature_range': (20.0, 80.0),  # °C
            'protection_probability': 0.02,      # 2% 확률로 보호 상태 활성화
            'balancing_probability': 0.3         # 30% 확률로 셀 밸런싱 활성화
        }
    
    def generate_pack_voltage_current_message(self) -> can.Message:
        """팩 전압/전류 메시지 생성 (ID: 0x100)"""
        pack_voltage = random.uniform(*self.simulation_config['pack_voltage_range'])
        current = random.uniform(*self.simulation_config['current_range'])
        
        # 0.1V, 0.1A 단위로 변환
        pack_voltage_raw = int(pack_voltage * 10)
        current_raw = int(current * 10)
        
        data = struct.pack('>Hh', pack_voltage_raw, current_raw)
        return can.Message(arbitration_id=0x100, data=data)
    
    def generate_cell_voltages_1_4_message(self) -> can.Message:
        """셀 전압 1~4번 메시지 생성 (ID: 0x101)"""
        voltages = [
            random.uniform(*self.simulation_config['cell_voltage_range'])
            for _ in range(4)
        ]
        
        # mV 단위로 변환
        voltages_raw = [int(v * 1000) for v in voltages]
        
        data = struct.pack('>HHHH', *voltages_raw)
        return can.Message(arbitration_id=0x101, data=data)
    
    def generate_cell_voltages_5_8_message(self) -> can.Message:
        """셀 전압 5~8번 메시지 생성 (ID: 0x102)"""
        voltages = [
            random.uniform(*self.simulation_config['cell_voltage_range'])
            for _ in range(4)
        ]
        
        # mV 단위로 변환
        voltages_raw = [int(v * 1000) for v in voltages]
        
        data = struct.pack('>HHHH', *voltages_raw)
        return can.Message(arbitration_id=0x102, data=data)
    
    def generate_temperature_message(self) -> can.Message:
        """온도 메시지 생성 (ID: 0x103)"""
        ts1 = random.uniform(*self.simulation_config['temperature_range'])
        ts3 = random.uniform(*self.simulation_config['temperature_range'])
        int_temp = random.uniform(*self.simulation_config['temperature_range'])
        
        # 0.1도 단위로 변환
        ts1_raw = int(ts1 * 10)
        ts3_raw = int(ts3 * 10)
        int_temp_raw = int(int_temp * 10)
        
        data = struct.pack('>hhh', ts1_raw, ts3_raw, int_temp_raw)
        return can.Message(arbitration_id=0x103, data=data)
    
    def generate_protection_status_message(self) -> can.Message:
        """보호 상태 메시지 생성 (ID: 0x104)"""
        status_byte = 0
        
        # 각 보호 기능별로 확률적으로 활성화
        if random.random() < self.simulation_config['protection_probability']:
            status_byte |= 0x01  # OVP
        if random.random() < self.simulation_config['protection_probability']:
            status_byte |= 0x02  # UVP
        if random.random() < self.simulation_config['protection_probability']:
            status_byte |= 0x04  # OCP
        if random.random() < self.simulation_config['protection_probability']:
            status_byte |= 0x08  # OTP
        if random.random() < self.simulation_config['protection_probability']:
            status_byte |= 0x10  # UTP
        
        data = bytes([status_byte])
        return can.Message(arbitration_id=0x104, data=data)
    
    def generate_cell_balancing_message(self) -> can.Message:
        """셀 밸런싱 상태 메시지 생성 (ID: 0x105)"""
        balancing_status = 0
        
        # 각 셀별로 확률적으로 밸런싱 활성화
        if random.random() < self.simulation_config['balancing_probability']:
            for i in range(10):  # 10개 셀
                if random.random() < 0.3:  # 각 셀별 30% 확률
                    balancing_status |= (1 << i)
        
        data = struct.pack('>H', balancing_status)
        return can.Message(arbitration_id=0x105, data=data)
    
    def generate_all_messages(self) -> List[can.Message]:
        """모든 시뮬레이션 메시지 생성"""
        return [
            self.generate_pack_voltage_current_message(),
            self.generate_cell_voltages_1_4_message(),
            self.generate_cell_voltages_5_8_message(),
            self.generate_temperature_message(),
            self.generate_protection_status_message(),
            self.generate_cell_balancing_message()
        ]
    
    async def run_simulation(self, message_handler) -> None:
        """시뮬레이션 실행"""
        self.is_running = True
        self.logger.info("Starting CAN simulation...")
        
        while self.is_running:
            try:
                messages = self.generate_all_messages()
                
                for msg in messages:
                    if not self.is_running:
                        break
                    
                    self.logger.debug(f"Generated CAN message: ID=0x{msg.arbitration_id:03X}, Data={msg.data.hex()}")
                    message_handler(msg)
                    
                    # 메시지 간 짧은 지연
                    await asyncio.sleep(0.1)
                
                # 다음 시뮬레이션 사이클까지 대기
                await asyncio.sleep(self.update_interval)
                
            except Exception as e:
                self.logger.error(f"Simulation error: {e}")
                await asyncio.sleep(1.0)
    
    def stop_simulation(self) -> None:
        """시뮬레이션 중지"""
        self.is_running = False
        self.logger.info("Stopping CAN simulation...")
    
    def update_config(self, config: Dict[str, Any]) -> None:
        """시뮬레이션 설정 업데이트"""
        self.simulation_config.update(config)
        self.logger.info(f"Simulation config updated: {config}")
    
    def get_config(self) -> Dict[str, Any]:
        """현재 시뮬레이션 설정 반환"""
        return self.simulation_config.copy()