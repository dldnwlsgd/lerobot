# Piper Robot Inference Guide

이 가이드는 학습된 모델을 사용하여 Piper 로봇에서 inference를 수행하는 방법을 설명합니다.

## 사전 요구사항

1. **ROS2 Humble 설치**
   ```bash
   # ROS2 Humble이 설치되어 있어야 합니다
   source /opt/ros/humble/setup.bash
   ```

2. **Piper ROS 패키지**
   ```bash
   # Piper ROS 워크스페이스가 빌드되어 있어야 합니다
   cd /home/rosota/Github/Piper_Ros-Tcp
   colcon build
   source install/setup.bash
   ```

3. **Python 환경**
   ```bash
   # lerobot 환경 활성화
   conda activate lerobot
   ```

## 빠른 시작

### 1. 기본 inference 실행

```bash
# ROS2 환경 설정 및 inference 실행 (모든 과정 자동화)
./src/lerobot/scripts/start_piper_inference.sh --start-piper-node
```

### 2. 수동으로 단계별 실행

#### 단계 1: Piper ROS 노드 시작
```bash
# 터미널 1에서 Piper ROS 노드 실행
source /opt/ros/humble/setup.bash
cd /home/rosota/Github/Piper_Ros-Tcp
source install/setup.bash
ros2 launch piper start_single_piper.launch.py
```

#### 단계 2: Inference 실행
```bash
# 터미널 2에서 inference 실행
source /opt/ros/humble/setup.bash
conda activate lerobot
python -m lerobot.infer_piper \
    --policy_path=outputs/train/act_piper_xr_test/checkpoints/100000/pretrained_model \
    --dataset_repo_id=ujin/piper-direct-mode-test_xr_2 \
    --episode_time_s=30 \
    --fps=30
```

## 설정 옵션

### 명령줄 옵션

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--policy_path` | 학습된 모델 경로 | `outputs/train/act_piper_xr_test/checkpoints/100000/pretrained_model` |
| `--dataset_repo_id` | 데이터셋 repo ID (features 구조 참조용) | `ujin/piper-direct-mode-test_xr_2` |
| `--episode_time_s` | inference 실행 시간 (초) | 30 |
| `--fps` | 제어 주파수 | 30 |
| `--robot_use_cameras` | 카메라 사용 여부 | true |
| `--verbose` | 상세 출력 | true |
| `--device` | 연산 장치 (cuda/cpu) | cuda |

### 스크립트 옵션

`start_piper_inference.sh` 스크립트 옵션:

| 옵션 | 설명 |
|------|------|
| `--start-piper-node` | Piper ROS 노드도 함께 시작 |
| `--no-cameras` | 카메라 없이 실행 |
| `--quiet` | 출력 최소화 |
| `-h, --help` | 도움말 표시 |

## ROS2 토픽

### 구독하는 토픽
- `/joint_states` - 현재 joint 상태를 받아옴

### 발행하는 토픽  
- `/joint_ctrl_single` - joint 명령을 전송

### 토픽 메시지 형식
```bash
# Joint state message 형식
sensor_msgs/msg/JointState:
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## 문제 해결

### 1. ROS2 관련 오류
```bash
# ROS2가 제대로 소싱되었는지 확인
source /opt/ros/humble/setup.bash
ros2 topic list
```

### 2. 카메라 연결 오류
```bash
# 카메라 없이 실행
python -m lerobot.infer_piper --robot_use_cameras=false
```

### 3. 모델 로드 오류
```bash
# 모델 경로 확인
ls -la outputs/train/act_piper_xr_test/checkpoints/100000/pretrained_model/
```

### 4. 데이터셋 오류
```bash
# 데이터셋 경로 확인
ls -la ~/.cache/huggingface/lerobot/ujin/piper-direct-mode-test_xr_2/
```

## 개발자 정보

### 코드 구조
```
src/lerobot/
├── infer_piper.py                    # 메인 inference 코드
└── scripts/
    ├── start_piper_inference.sh      # 시작 스크립트
    └── PIPER_INFERENCE_README.md     # 이 문서
```

### 주요 클래스
- `InferConfig` - inference 설정
- `PiperInferenceNode` - ROS2 노드 (joint state 구독/발행)
- `PiperInferenceSystem` - 메인 inference 시스템

### 처리 흐름
1. 데이터셋에서 features 구조 로드
2. 학습된 모델 로드
3. ROS2 노드 초기화 및 joint state 구독 시작
4. 카메라 연결 (옵션)
5. Inference 루프:
   - 현재 joint state 및 카메라 이미지 획득
   - 모델을 통해 다음 action 예측
   - ROS2를 통해 joint 명령 전송

## 예제

### 기본 사용법
```bash
./src/lerobot/scripts/start_piper_inference.sh
```

### 카메라 없이 실행
```bash
./src/lerobot/scripts/start_piper_inference.sh --no-cameras
```

### 다른 모델 사용
```bash
./src/lerobot/scripts/start_piper_inference.sh \
    --policy_path=outputs/train/my_other_model/checkpoints/050000/pretrained_model
```

### 긴 시간 실행
```bash
./src/lerobot/scripts/start_piper_inference.sh \
    --episode_time_s=120 \
    --fps=20
``` 