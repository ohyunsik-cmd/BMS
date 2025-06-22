import struct
import logging
from typing import Dict, Any, Optional

class CANDataProcessor:
    """CAN 데이터를 파싱하고 BMS 데이터로 변환하는 클래스"""
    
    def __init__(self):
        # CAN ID별 데이터 매핑 정의
        self.can_id_mapping = {
            0x100: self.parse_pack_voltage_current,
            0x101: self.parse_cell_voltages_1_5,
            0x102: self.parse_cell_voltages_6_10,
            0x103: self.parse_temperatures,
            0x104: self.parse_protection_status,
            0x105: self.parse_cell_balancing
        }
        
        # BMS 데이터 저장소
        self.bms_data = {
            "deviceId": "BMS_01",
            "packVoltage": 75,
            "cellVoltages": [3.7, 4.0, 3.7, 3.9, 4.3, 3.7, 4.0, 3.7, 3.9, 4.3],
            "current": 12.5,
            "temperatures": {
                "ts1": 45,
                "ts3": 47,
                "intTemp": 48
            },
            "protection": {
                "ovp": False,
                "uvp": False,
                "ocp": False,
                "otp": False,
                "utp": False
            },
            "cellBalancing": 0
        }
        
        # 로깅 설정
        self.logger = logging.getLogger(__name__)

    def parse_pack_voltage_current(self, data: bytes) -> None:
        """팩 전압과 전류 파싱"""
        try:
            if len(data) >= 6:
                # 2바이트 팩 전압 (0.1V 단위), 2바이트 전류 (0.1A 단위, signed)
                pack_voltage_raw = struct.unpack('>H', data[0:2])[0]  # Big-endian 16bit
                current_raw = struct.unpack('>h', data[2:4])[0]       # Big-endian signed 16bit
                
                pack_voltage = pack_voltage_raw / 10.0  # 실제 전압
                current = current_raw / 10.0            # 실제 전류
                
                # 팩 전압을 퍼센트로 변환 (예: 36V~42V를 0~100%로)
                voltage_percent = max(0, min(100, ((pack_voltage - 36.0) / 6.0) * 100))
                
                self.bms_data["packVoltage"] = round(voltage_percent, 1)
                self.bms_data["current"] = round(current, 1)
                
                self.logger.debug(f"Pack voltage: {pack_voltage}V ({voltage_percent}%), Current: {current}A")
        except Exception as e:
            self.logger.error(f"Error parsing pack voltage/current: {e}")

    def parse_cell_voltages_1_5(self, data: bytes) -> None:
        """셀 전압 1~5번 파싱"""
        try:
            if len(data) >= 8:
                for i in range(4):  # 4개 셀 (각 2바이트)
                    if i * 2 + 1 < len(data):
                        voltage_raw = struct.unpack('>H', data[i*2:i*2+2])[0]
                        voltage = voltage_raw / 1000.0  # mV to V
                        if i < len(self.bms_data["cellVoltages"]):
                            self.bms_data["cellVoltages"][i] = round(voltage, 2)
                            
                self.logger.debug(f"Cell voltages 1-4: {self.bms_data['cellVoltages'][:4]}")
        except Exception as e:
            self.logger.error(f"Error parsing cell voltages 1-5: {e}")

    def parse_cell_voltages_6_10(self, data: bytes) -> None:
        """셀 전압 6~10번 파싱"""
        try:
            if len(data) >= 8:
                for i in range(4):  # 4개 셀 (각 2바이트)
                    if i * 2 + 1 < len(data):
                        voltage_raw = struct.unpack('>H', data[i*2:i*2+2])[0]
                        voltage = voltage_raw / 1000.0  # mV to V
                        cell_index = i + 5  # 6~9번 셀 (인덱스 5~8)
                        if cell_index < len(self.bms_data["cellVoltages"]):
                            self.bms_data["cellVoltages"][cell_index] = round(voltage, 2)
                            
                self.logger.debug(f"Cell voltages 6-9: {self.bms_data['cellVoltages'][5:9]}")
        except Exception as e:
            self.logger.error(f"Error parsing cell voltages 6-10: {e}")

    def parse_temperatures(self, data: bytes) -> None:
        """온도 데이터 파싱"""
        try:
            if len(data) >= 6:
                # 각 온도는 2바이트 (0.1도 단위)
                ts1_raw = struct.unpack('>h', data[0:2])[0]  # signed
                ts3_raw = struct.unpack('>h', data[2:4])[0]  # signed
                int_temp_raw = struct.unpack('>h', data[4:6])[0]  # signed
                
                self.bms_data["temperatures"]["ts1"] = round(ts1_raw / 10.0, 1)
                self.bms_data["temperatures"]["ts3"] = round(ts3_raw / 10.0, 1)
                self.bms_data["temperatures"]["intTemp"] = round(int_temp_raw / 10.0, 1)
                
                self.logger.debug(f"Temperatures: {self.bms_data['temperatures']}")
        except Exception as e:
            self.logger.error(f"Error parsing temperatures: {e}")

    def parse_protection_status(self, data: bytes) -> None:
        """보호 상태 파싱"""
        try:
            if len(data) >= 1:
                status_byte = data[0]
                self.bms_data["protection"]["ovp"] = bool(status_byte & 0x01)
                self.bms_data["protection"]["uvp"] = bool(status_byte & 0x02)
                self.bms_data["protection"]["ocp"] = bool(status_byte & 0x04)
                self.bms_data["protection"]["otp"] = bool(status_byte & 0x08)
                self.bms_data["protection"]["utp"] = bool(status_byte & 0x10)
                
                self.logger.debug(f"Protection status: {self.bms_data['protection']}")
        except Exception as e:
            self.logger.error(f"Error parsing protection status: {e}")

    def parse_cell_balancing(self, data: bytes) -> None:
        """셀 밸런싱 상태 파싱"""
        try:
            if len(data) >= 2:
                balancing_status = struct.unpack('>H', data[0:2])[0]
                self.bms_data["cellBalancing"] = balancing_status & 0x3FF  # 하위 10비트만 사용
                
                self.logger.debug(f"Cell balancing: {self.bms_data['cellBalancing']}")
        except Exception as e:
            self.logger.error(f"Error parsing cell balancing: {e}")

    def process_can_message(self, msg) -> bool:
        """CAN 메시지 처리"""
        can_id = msg.arbitration_id
        data = msg.data
        
        if can_id in self.can_id_mapping:
            try:
                self.can_id_mapping[can_id](data)
                self.logger.info(f"Processed CAN ID: 0x{can_id:03X}, Data: {data.hex()}")
                return True
            except Exception as e:
                self.logger.error(f"CAN message parsing error (ID: 0x{can_id:03X}): {e}")
        else:
            self.logger.warning(f"Unknown CAN ID: 0x{can_id:03X}")
        return False

    def get_bms_data(self) -> Dict[str, Any]:
        """현재 BMS 데이터 반환"""
        return self.bms_data.copy()
    
    def update_device_id(self, device_id: str) -> None:
        """디바이스 ID 업데이트"""
        self.bms_data["deviceId"] = device_id
        
    def reset_data(self) -> None:
        """BMS 데이터 초기화"""
        self.bms_data.update({
            "packVoltage": 0,
            "cellVoltages": [0.0] * 10,
            "current": 0.0,
            "temperatures": {"ts1": 0, "ts3": 0, "intTemp": 0},
            "protection": {"ovp": False, "uvp": False, "ocp": False, "otp": False, "utp": False},
            "cellBalancing": 0
        })