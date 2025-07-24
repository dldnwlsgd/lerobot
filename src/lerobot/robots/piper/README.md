# Piper Robot

Piper 로봇 팔을 위한 LeRobot 구현입니다.

## 개요

Piper는 7개 관절을 가진 로봇 팔로, CAN 통신과 ROS2를 통해 실시간으로 제어됩니다.

## Connection Modes

Piper 로봇은 두 가지 연결 모드를 지원합니다:

### 1. Direct Mode (기본값, 권장)
- **설정**: `use_direct_mode=True`
- **실행 방법**: `joint_state_loop_pub.py`를 직접 실행
- **특징**:
  - RViz 없이 독립적으로 실행 가능
  - Unity나 TCP를 통한 실시간 데이터 수신
  - 더 가벼운 실행 환경

```bash
# Direct mode로 joint_state_loop_pub.py 실행
cd Piper_Ros-Tcp
python3 joint_state_loop_pub.py
```

### 2. RViz Mode (레거시)
- **설정**: `use_direct_mode=False`  
- **실행 방법**: `start_single_piper_rviz.launch.py` 실행 필요
- **특징**:
  - RViz 시각화 포함
  - 전체 launch 시스템 필요
  - 개발 및 디버깅에 유용

```bash
# RViz mode로 launch 파일 실행
ros2 launch piper start_single_piper_rviz.launch.py can_port:=can0 auto_enable:=true gripper_exist:=true
```

### 관절 구성

- **joint_1**: 베이스 회전 관절
- **joint_2**: 어깨 피치 관절  
- **joint_3**: 어깨 롤 관절
- **joint_4**: 팔꿈치 피치 관절
- **joint_5**: 손목 피치 관절
- **joint_6**: 손목 롤 관절
- **joint_7**: 그리퍼

## 설정

### 기본 설정 예시

```python
from lerobot.robots.piper import PiperConfig, Piper

config = PiperConfig(
    id="piper_robot",
    can_interface="can0",  # CAN 인터페이스
    use_direct_mode=True,  # True: joint_state_loop_pub.py, False: RViz launch
    max_joint_velocity=2.0,  # 최대 관절 속도 (rad/s)
    max_relative_target=0.5,  # 안전을 위한 최대 상대 이동량 (rad)
    joint_read_frequency=100.0,  # 관절 상태 읽기 주파수 (Hz)
    action_frequency=50.0,  # 액션 전송 주파수 (Hz)
)
```

### 카메라 포함 설정 (기본 설정 포함)

```python
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

config = PiperConfig(
    id="piper_robot",
    can_interface="can0",
    cameras={
        # 손목 Intel RealSense 카메라 (기본 포함)
        "wrist": RealSenseCameraConfig(
            name="Intel RealSense D405",
            fps=30,
            width=640,
            height=480,
        ),
        # 핸드폰 카메라 (기본 포함)
        "phone": OpenCVCameraConfig(
            fps=30,
            width=640,
            height=480,
            index_or_path=0,  # 실제 연결된 카메라 인덱스
        ),
        # 추가 외부 카메라 (옵션)
        "external": OpenCVCameraConfig(
            fps=30,
            width=1280,
            height=720,
            index_or_path=1,
        ),
    }
)
```

### SDK 설정 파일 포함

```python
config = PiperConfig(
    id="piper_robot",
    can_interface="can0",
    sdk_config_path="/path/to/piper_sdk_config.yaml",  # SDK 설정 파일 경로
    # 관절 제한 커스터마이징 (옵션)
    joint_limits={
        "joint_1": [-3.14, 3.14],
        "joint_2": [-2.5, 2.5],
        "joint_3": [-2.0, 2.0],
        "joint_4": [-3.0, 3.0],
        "joint_5": [-2.5, 2.5],
        "joint_6": [-3.14, 3.14],
        "joint_7": [-1.0, 1.0],  # 그리퍼
    }
)
```

## 사용법

### 기본 사용

```python
from lerobot.robots.piper import PiperConfig, Piper

# 설정 생성
config = PiperConfig(id="my_piper", can_interface="can0")

# 로봇 인스턴스 생성
robot = Piper(config)

# 연결 (piper는 공장 캘리브레이션 사용)
robot.connect()

# 관찰 데이터 읽기 (실시간 관절 상태)
observation = robot.get_observation()
print(f"현재 위치: {observation}")

# 액션 전송
action = {
    "joint_1.pos": 0.1,    # 베이스 회전
    "joint_2.pos": -0.2,   # 어깨 피치
    "joint_3.pos": 0.3,    # 어깨 롤
    "joint_4.pos": 0.0,    # 팔꿈치 피치
    "joint_5.pos": 0.0,    # 손목 피치
    "joint_6.pos": 0.0,    # 손목 롤
    "joint_7.pos": 0.5,    # 그리퍼 (0.5 라디안 열림)
}

sent_action = robot.send_action(action)
print(f"전송된 액션: {sent_action}")

# 로봇 상태 확인
status = robot.get_status()
print(f"모터 토크: {[motor['load'] for motor in status['motors'].values()]}")

# 연결 해제
robot.disconnect()
```

### CAN 인터페이스 설정

CAN 인터페이스를 사용하기 전에 시스템 설정이 필요합니다:

```bash
# CAN 인터페이스 활성화
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# CAN 통신 확인
candump can0
```

### 실시간 관절 모니터링

```python
# 실시간으로 관절 상태 모니터링
robot.connect()

for i in range(100):  # 10초간 모니터링 (100Hz)
    observation = robot.get_observation()
    joint_positions = [observation[f"joint_{j}.pos"] for j in range(1, 8)]
    print(f"관절 위치: {joint_positions}")
    time.sleep(0.01)  # 100Hz
```

### 안전 기능 활용

```python
# 응급 정지
robot.sdk.emergency_stop()

# 응급 정지 상태 해제
robot.reset_emergency_stop()

# 통신 타임아웃 확인
if robot.sdk.check_communication_timeout(1.0):
    print("통신 타임아웃 발생!")
```

### 상태 확인

```python
status = robot.get_status()
print(f"로봇 상태: {status}")
```

## 설정 옵션

### 통신 설정

- `can_interface`: CAN 인터페이스 이름 (예: "can0")
- `sdk_config_path`: Piper SDK 설정 파일 경로
- `communication_timeout`: 통신 타임아웃 (초)

### 주파수 설정

- `joint_read_frequency`: 관절 상태 읽기 주파수 (Hz)
- `action_frequency`: 액션 전송 주파수 (Hz)

### 안전 설정

- `max_joint_velocity`: 최대 관절 속도 (rad/s)
- `max_joint_acceleration`: 최대 관절 가속도 (rad/s²)
- `max_relative_target`: 안전을 위한 최대 상대 이동량 (rad)
- `max_joint_torque`: 최대 관절 토크 (Nm)
- `enable_collision_detection`: 충돌 감지 활성화
- `emergency_stop_acceleration`: 응급 정지 가속도 (rad/s²)

### 관절 설정

- `joint_names`: 관절 이름 리스트
- `joint_limits`: 각 관절의 위치 제한 (rad)

## 문제 해결

### CAN 통신 문제

1. CAN 인터페이스가 활성화되어 있는지 확인:
   ```bash
   ip link show can0
   ```

2. 권한 문제 해결:
   ```bash
   sudo usermod -a -G dialout $USER
   # 재로그인 필요
   ```

3. CAN 비트레이트 확인:
   ```bash
   sudo ip link set can0 type can bitrate 500000
   ```

### piper_sdk 문제

1. piper_sdk가 설치되어 있는지 확인:
   ```bash
   pip list | grep piper
   ```

2. SDK 설정 파일 경로가 올바른지 확인
3. 로봇 전원이 켜져 있고 CAN 연결이 정상인지 확인

### 실시간 성능 문제

1. 시스템 로드 확인:
   ```bash
   top
   htop
   ```

2. `joint_read_frequency`와 `action_frequency` 조정
3. 카메라 해상도를 줄여 성능 향상:
   ```python
   cameras={
       "wrist": RealSenseCameraConfig(
           width=320,  # 640에서 320으로 감소
           height=240,  # 480에서 240으로 감소
       )
   }
   ```

### 안전 문제

1. 응급 정지가 자주 발생하는 경우:
   - `max_joint_velocity` 값을 낮춤
   - `max_relative_target` 값을 낮춤
   - `communication_timeout` 값을 늘림

2. 관절 제한 오류:
   - `joint_limits` 설정 확인
   - 현재 관절 위치가 제한 범위 내에 있는지 확인

### 카메라 문제

1. Intel RealSense 카메라 문제:
   ```bash
   # RealSense SDK 설치 확인
   rs-enumerate-devices
   ```

2. 핸드폰 카메라 연결 문제:
   - USB 연결 확인
   - 카메라 인덱스 확인: `ls /dev/video*`
   - 다른 프로그램에서 카메라를 사용 중인지 확인

### 디버깅

실시간 상태 모니터링으로 문제 진단:

```python
robot.connect()
status = robot.get_status()

print("통신 상태:", not status["communication_timeout"])
print("응급 정지:", status["emergency_stop"])
print("관절 토크:", [j["torque"] for j in status["joints"].values()])
``` 