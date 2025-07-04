# Robot Arm Simulation with Box2D

3-link 로봇 팔 시뮬레이션 프로젝트입니다. Box2D 물리 엔진과 pygame을 사용하여 로봇이 목표 지점으로 이동하는 시뮬레이션을 제공합니다.

## 프로젝트 구조

```
robot_sim_v1/
├── env.py           # 로봇 환경 설정 (월드, 링크, 장애물 생성)
├── policy.py        # 제어 정책 (potential field, RMP)
├── render.py        # 시각화 렌더링
├── main.py          # 메인 시뮬레이션
├── joint_test.py    # 조인트 회전 테스트
├── simple_test.py   # 간단한 토크 테스트
└── record_video.py  # 비디오 녹화 기능
```

## 설치 및 설정

### 1. 가상환경 생성 (uv 사용)
```bash
cd pybox
uv venv pybox2d
source pybox2d/bin/activate  # macOS/Linux
```

### 2. 필수 패키지 설치
```bash
# Box2D 물리 엔진
uv pip install Box2D

# Jupyter 노트북 지원
pip install ipykernel
python -m ipykernel install --user --name pybox2d --display-name "pybox2d (Python 3.8.20)"

# 비디오 녹화용 (선택사항)
pip install opencv-python
```

## 사용법

### 1. 기본 시뮬레이션 실행

**기본 설정으로 실행 (target: 5.0, 5.0):**
```bash
python main.py
```

**특정 목표 위치 설정:**
```bash
python main.py --target 6.0 2.0
python main.py --target 4.0 3.0
python main.py --target 8.0 -1.0
```

### 2. 테스트 프로그램

**조인트 회전 테스트 (각 조인트가 서로 다른 속도로 회전):**
```bash
python joint_test.py
```

**간단한 토크 테스트:**
```bash
python simple_test.py
```

### 3. 비디오 녹화

**기본 10초 녹화:**
```bash
python record_video.py --target 5.0 5.0 --duration 10 --output robot_target_5_5.mp4
```

**다양한 옵션으로 녹화:**
```bash
# 특정 목표로 15초 녹화
python record_video.py --target 7.0 3.0 --duration 15 --output robot_reach_7_3.mp4

# 높은 품질로 녹화 (60 FPS)
python record_video.py --target 4.0 4.0 --duration 8 --fps 60 --output high_quality.mp4
```

## 명령어 옵션

### main.py 옵션
- `--target X Y`: 목표 위치 설정 (기본값: 5.0 5.0)

### record_video.py 옵션
- `--target X Y`: 목표 위치 설정 (기본값: 5.0 5.0)
- `--duration SECONDS`: 녹화 시간 (기본값: 10.0초)
- `--output FILENAME`: 출력 파일명 (기본값: robot_simulation.mp4)
- `--fps FPS`: 비디오 FPS (기본값: 60)

## 시뮬레이션 특징

### 로봇 사양
- **링크 수**: 3개
- **링크 길이**: [3.0m, 2.5m, 2.0m]
- **총 도달 범위**: 최대 7.5m
- **조인트**: 회전 조인트 3개

### 제어 방식
- **Potential Field**: 목표점으로의 attractive force + 장애물 회피 repulsive force
- **Jacobian**: 해석적 계산으로 end-effector force를 조인트 토크로 변환
- **물리 시뮬레이션**: Box2D 엔진 사용

### 시각화
- **로봇**: 회색 사각형 링크
- **목표점**: 빨간 원 (중심에 흰 점)
- **장애물**: 회색 사각형
- **실시간 정보**: 목표 위치, 프레임 정보 표시

## 예시 실행 명령어

```bash
# 다양한 목표 위치로 테스트
python main.py --target 3.0 3.0    # 가까운 목표
python main.py --target 7.0 0.0    # 수평 목표  
python main.py --target 5.0 5.0    # 대각선 목표
python main.py --target 2.0 -2.0   # 아래쪽 목표

# 비디오 녹화 예시
python record_video.py --target 6.0 4.0 --duration 12 --output demo.mp4
```

## 키보드 단축키

- **ESC**: 프로그램 종료 (joint_test.py)
- **창 닫기**: 시뮬레이션 종료

## 문제 해결

### 로봇이 움직이지 않는 경우
1. Force 값 확인: 목표가 너무 가까우면 force가 작을 수 있음
2. Jacobian 계산 확인: 특이점(singularity) 근처에서는 움직임이 제한됨
3. 목표 위치를 로봇 도달 범위 내로 설정 (최대 7.5m)

### 비디오 녹화 오류
```bash
# OpenCV 재설치
pip install --upgrade opencv-python
```

## 개발자 정보

이 프로젝트는 Box2D 물리 엔진을 사용한 로봇 팔 시뮬레이션 데모입니다.
- **물리 엔진**: Box2D
- **렌더링**: pygame
- **제어**: Potential Field + Jacobian Transpose
- **비디오**: OpenCV
