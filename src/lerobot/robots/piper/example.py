#!/usr/bin/env python

"""
Piper 로봇 사용 예시

이 스크립트는 Piper 로봇의 기본적인 사용법을 보여줍니다.
"""

import time
import logging
import numpy as np
from lerobot.robots.piper import PiperConfig, Piper
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def basic_example():
    """기본 사용 예시"""
    logger.info("=== Piper 로봇 기본 사용 예시 ===")
    
    # 설정 생성
    config = PiperConfig(
        id="piper_example",
        can_interface="can0",  # CAN 인터페이스
        max_joint_velocity=1.0,  # 안전을 위해 낮은 속도 사용
        max_relative_target=0.3,  # 안전을 위한 제한 (라디안)
        joint_read_frequency=50.0,  # 50Hz로 줄여서 안정성 향상
        action_frequency=25.0,  # 25Hz 액션 주파수
    )
    
    # 로봇 인스턴스 생성
    robot = Piper(config)
    
    try:
        # 연결
        logger.info("로봇에 연결 중...")
        robot.connect()  # Piper는 공장 캘리브레이션 사용
        logger.info("연결 완료!")
        
        # 현재 상태 읽기
        observation = robot.get_observation()
        logger.info("현재 로봇 위치:")
        for joint, pos in observation.items():
            if joint.endswith('.pos'):
                logger.info(f"  {joint}: {pos:.3f}")
        
        # 기본 위치로 이동
        logger.info("기본 위치로 이동...")
        home_action = {
            "joint_1.pos": 0.0,  # 베이스
            "joint_2.pos": 0.0,  # 어깨 피치
            "joint_3.pos": 0.0,  # 어깨 롤
            "joint_4.pos": 0.0,  # 팔꿈치
            "joint_5.pos": 0.0,  # 손목 피치
            "joint_6.pos": 0.0,  # 손목 롤
            "joint_7.pos": 0.0,  # 그리퍼 닫기
        }
        robot.send_action(home_action)
        time.sleep(2)
        
        # 간단한 동작 시퀀스
        logger.info("동작 시퀀스 시작...")
        
        movements = [
            # 베이스 회전
            {"joint_1.pos": 0.5, "joint_7.pos": 0.2},  # 베이스 회전, 그리퍼 약간 열기
            {"joint_1.pos": -0.5, "joint_7.pos": 0.4},
            {"joint_1.pos": 0.0, "joint_7.pos": 0.6},
            
            # 어깨와 팔꿈치 동작
            {"joint_2.pos": 0.3, "joint_4.pos": 0.8, "joint_7.pos": 0.8},  # 어깨와 팔꿈치
            {"joint_2.pos": -0.3, "joint_4.pos": -0.8, "joint_7.pos": 1.0},
            {"joint_2.pos": 0.0, "joint_4.pos": 0.0, "joint_7.pos": 0.5},
            
            # 손목 동작
            {"joint_5.pos": 0.5, "joint_6.pos": 0.8},  # 손목 피치와 롤
            {"joint_5.pos": -0.5, "joint_6.pos": -0.8},
            
            # 원래 위치로 복귀
            {"joint_5.pos": 0.0, "joint_6.pos": 0.0, "joint_7.pos": 0.0},
        ]
        
        for i, movement in enumerate(movements):
            logger.info(f"동작 {i+1}/{len(movements)}: {movement}")
            
            # 현재 위치 기반으로 액션 생성
            current_obs = robot.get_observation()
            action = {}
            
            # 현재 위치 유지
            for motor_name in robot.bus.motors:
                action[f"{motor_name}.pos"] = current_obs[f"{motor_name}.pos"]
            
            # 지정된 동작만 변경
            action.update(movement)
            
            robot.send_action(action)
            time.sleep(1.5)
        
        logger.info("동작 시퀀스 완료!")
        
    except Exception as e:
        logger.error(f"오류 발생: {e}")
    finally:
        # 연결 해제
        if robot.is_connected:
            robot.disconnect()
            logger.info("로봇 연결 해제 완료")


def camera_example():
    """카메라 포함 예시"""
    logger.info("=== Piper 로봇 카메라 예시 ===")
    
    # 기본 카메라 설정 사용 (wrist + phone)
    config = PiperConfig(
        id="piper_camera_example",
        can_interface="can0",
        # cameras는 기본 설정 사용 (wrist: RealSense, phone: OpenCV)
    )
    
    robot = Piper(config)
    
    try:
        robot.connect()
        
        # 관찰 데이터 (관절 + 카메라)
        observation = robot.get_observation()
        
        logger.info("관찰 데이터:")
        for key, value in observation.items():
            if key.endswith('.pos'):
                logger.info(f"  {key}: {value:.3f}")
            else:
                # 카메라 이미지
                if hasattr(value, 'shape'):
                    logger.info(f"  {key}: 이미지 크기 {value.shape}")
                    
        # 카메라별 상세 정보
        logger.info("카메라 상세 정보:")
        for cam_name, cam in robot.cameras.items():
            logger.info(f"  {cam_name}: 연결됨 = {cam.is_connected}")
            if cam.is_connected:
                logger.info(f"    해상도: {cam.width}x{cam.height}")
                logger.info(f"    FPS: {cam.fps}")
        
    except Exception as e:
        logger.error(f"오류 발생: {e}")
    finally:
        if robot.is_connected:
            robot.disconnect()


def status_monitoring_example():
    """상태 모니터링 예시"""
    logger.info("=== Piper 로봇 상태 모니터링 예시 ===")
    
    config = PiperConfig(
        id="piper_monitoring",
        can_interface="can0",
        joint_read_frequency=50.0,  # 모니터링을 위해 중간 주파수 사용
    )
    
    robot = Piper(config)
    
    try:
        robot.connect()
        
        # 10초 동안 상태 모니터링
        logger.info("10초 동안 상태 모니터링...")
        for i in range(10):
            status = robot.get_status()
            
            logger.info(f"상태 체크 {i+1}/10:")
            logger.info(f"  연결됨: {status['connected']}")
            logger.info(f"  캘리브레이션됨: {status['calibrated']}")
            logger.info(f"  응급 정지: {status['emergency_stop']}")
            logger.info(f"  통신 타임아웃: {status['communication_timeout']}")
            
            # 모터 상태 체크
            logger.info("  모터 상태:")
            for motor, info in status['motors'].items():
                pos = info['position']
                vel = info['velocity'] 
                load = info['load']
                temp = info['temperature']
                
                # 이상 상태 체크
                if abs(load) > 5.0:  # 5Nm 이상이면 경고
                    logger.warning(f"    {motor} - 위치: {pos:.3f}, 속도: {vel:.3f}, 로드: {load:.3f}, 온도: {temp:.1f}°C (로드 높음!)")
                elif temp > 60.0:  # 60도 이상이면 경고
                    logger.warning(f"    {motor} - 위치: {pos:.3f}, 속도: {vel:.3f}, 로드: {load:.3f}, 온도: {temp:.1f}°C (온도 높음!)")
                else:
                    logger.info(f"    {motor} - 위치: {pos:.3f}, 속도: {vel:.3f}, 로드: {load:.3f}, 온도: {temp:.1f}°C")
            
            time.sleep(1)
        
    except Exception as e:
        logger.error(f"오류 발생: {e}")
    finally:
        if robot.is_connected:
            robot.disconnect()


def main():
    """메인 함수"""
    logger.info("Piper 로봇 예시 프로그램 (CAN 통신 버전)")
    logger.info("실행 전 확인사항:")
    logger.info("1. CAN 인터페이스가 활성화되어 있는지 확인: sudo ip link set can0 up")
    logger.info("2. piper_sdk가 설치되어 있는지 확인")
    logger.info("3. 로봇 전원이 켜져 있는지 확인\n")
    
    while True:
        print("\n어떤 예시를 실행하시겠습니까?")
        print("1. 기본 사용 예시 (관절 움직임)")
        print("2. 카메라 포함 예시 (wrist + phone)")
        print("3. 상태 모니터링 예시 (실시간 관절 상태)")
        print("4. 종료")
        
        choice = input("선택 (1-4): ").strip()
        
        if choice == "1":
            try:
                basic_example()
            except Exception as e:
                logger.error(f"기본 예시 실행 중 오류: {e}")
        elif choice == "2":
            try:
                camera_example()
            except Exception as e:
                logger.error(f"카메라 예시 실행 중 오류: {e}")
        elif choice == "3":
            try:
                status_monitoring_example()
            except Exception as e:
                logger.error(f"상태 모니터링 예시 실행 중 오류: {e}")
        elif choice == "4":
            logger.info("프로그램을 종료합니다.")
            break
        else:
            print("잘못된 선택입니다. 다시 선택해주세요.")


if __name__ == "__main__":
    main() 