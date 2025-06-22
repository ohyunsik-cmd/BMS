import serial.tools.list_ports
import time

print("=== 간단한 CAN 테스트 ===")

# 1. COM 포트 찾기
print("\n1. COM 포트 검색 중...")
ports = serial.tools.list_ports.comports()

if not ports:
    print("❌ COM 포트를 찾을 수 없습니다!")
    input("Enter 키를 눌러 종료...")
    exit()

print(f"✅ 총 {len(ports)}개의 포트 발견:")
for i, port in enumerate(ports):
    print(f"   {i+1}. {port.device} - {port.description}")

# 2. CAN 라이브러리 테스트
print("\n2. python-can 라이브러리 테스트...")
try:
    import can
    print("✅ python-can 라이브러리 설치됨")
except ImportError:
    print("❌ python-can 라이브러리가 설치되지 않음")
    print("다음 명령어로 설치하세요:")
    print("pip install python-can")
    input("Enter 키를 눌러 종료...")
    exit()

# 3. 포트 선택
while True:
    try:
        choice = input(f"\n테스트할 포트 번호를 선택하세요 (1-{len(ports)}): ")
        port_index = int(choice) - 1
        if 0 <= port_index < len(ports):
            selected_port = ports[port_index].device
            break
        else:
            print("잘못된 번호입니다.")
    except ValueError:
        print("숫자를 입력해주세요.")

print(f"\n선택된 포트: {selected_port}")

# 4. CAN 연결 테스트
print("\n3. CAN 연결 테스트...")
try:
    bus = can.interface.Bus(
        bustype='serial',
        channel=selected_port,
        bitrate=250000,
        timeout=1.0
    )
    print("✅ CAN 연결 성공!")
    
    # 5. 메시지 수신 테스트
    print("\n4. 메시지 수신 테스트 (10초간)...")
    print("STM32에서 메시지를 보내고 있는지 확인하세요...")
    
    for i in range(100):  # 10초간 0.1초마다 체크
        message = bus.recv(timeout=0.1)
        if message:
            print(f"🎉 메시지 수신!")
            print(f"   ID: 0x{message.arbitration_id:03X}")
            print(f"   Data: {message.data.hex().upper()}")
            break
        
        if i % 10 == 0:  # 1초마다 진행 상황
            print(f"   대기 중... {i//10 + 1}/10초")
    else:
        print("❌ 10초간 메시지 없음")
        print("STM32가 메시지를 보내고 있는지 확인하세요")
    
    bus.shutdown()
    
except Exception as e:
    print(f"❌ CAN 연결 실패: {e}")

print("\n테스트 완료!")
input("Enter 키를 눌러 종료...")