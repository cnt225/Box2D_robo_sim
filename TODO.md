# TODO List - Robot Simulation Project

## 📋 현재 완료 상태 (2025-01-2025)

### ✅ **주요 완성 시스템들**

#### 🏗️ **포인트클라우드 & 환경 시스템 (완료)**
- ✅ 이중 모듈 아키텍처 구현 완료
- ✅ 8개 사전 생성 환경 라이브러리 구축
- ✅ Advanced Shape Detection (circularity 분석, 85th percentile 반지름)
- ✅ 조직화된 폴더 구조 (data/env_name/)
- ✅ 통합 메타데이터 시스템 (PLY + JSON + 시각화)

#### 🎯 **Quick Visualization 시스템 (완료)**
- ✅ `utils/quick_visualize.py` - 빠른 환경 조회 도구
- ✅ 특정 환경 번호 지정 조회 (예: #65번 환경)
- ✅ 환경 정보 표시 및 이미지 저장
- ✅ 대용량 데이터셋 (10K+ 환경) 빠른 접근

#### 🔷 **Concave 장애물 생성 시스템 (완료)**
- ✅ `pointcloud/concave_shape_generator/` 모듈 완성
- ✅ L-shape, U-shape 등 복잡한 오목 형태 생성
- ✅ Box2D 자동 Convex Decomposition 통합
- ✅ SVG → JSON → Box2D Bodies 완전 파이프라인
- ✅ Multiple Methods (Triangulation, Convex Hull, Multiple Fixtures)

#### 🤖 **Robot Pose 생성 시스템 (완료)**
- ✅ `pose/` 모듈 완성 (3개 핵심 파일)
- ✅ Random Pose Generator (6가지 로봇 구성 지원)
- ✅ Collision Detector (Rectangle/Ellipse 링크 정밀 검사)
- ✅ Complete Pipeline (392+ poses/sec 성능)
- ✅ JSON 결과 저장 (통계 포함)

#### 📁 **파일 구조 & 데이터 관리 (완료)**
- ✅ .gitignore 대폭 업데이트 (557MB 데이터 제외)
- ✅ 기능별 모듈화 (pointcloud/, pose/, 메인 시뮬레이션)
- ✅ 레거시 파일 정리 (utils/svg_to_ply_converter, test_adapter.py 등)
- ✅ 대용량 데이터셋 관리 (circle_envs_10k: 4,238 files, 205MB)

---

## 🚀 다음 우선순위 작업

### 1. **Pose Dataset 대량 생산 파이프라인** (즉시 시작 가능)
현재 pose 생성 시스템이 완성되었으므로 대규모 데이터셋 생성:

#### 1.1 Batch Pose Generation System
- [ ] **`pose/batch_generate_poses.py`** 생성
  - 모든 환경(8개 기본 + circle_envs_10k)에 대해 일괄 pose 생성
  - 로봇별(0-5) × 환경별 pose 데이터 매트릭스 구축
  - 진행상황 추적 및 재시작 기능

#### 1.2 Pose Pair Generation
- [ ] **{init_pose, target_pose} 쌍 생성 시스템**
  - 유효한 시작-목표 포즈 쌍 생성
  - 경로 존재성 검증 (간단한 직선 경로 체크)
  - 난이도별 분류 (거리, 장애물 수)

#### 1.3 Dataset Management
- [ ] **구조화된 데이터 저장**
  ```
  pose_datasets/
  ├── robot_0/
  │   ├── circles_only_poses.json
  │   ├── random_hard_01_poses.json
  │   └── circle_envs_10k/
  │       ├── env_0001_poses.json
  │       └── env_0002_poses.json
  └── robot_1/
  ```
- [ ] **메타데이터 통합 관리**
- [ ] **품질 검증 시스템**

### 2. **궤적 계획 및 검증 시스템**
Pose 데이터를 활용한 궤적 생성:

#### 2.1 Path Planning Integration
- [ ] **SE3_utils.SE3_update 기반 궤적 생성**
- [ ] **V 기반 trajectory 생성 시스템**
- [ ] **충돌 없는 경로 검증**

#### 2.2 Trajectory Dataset
- [ ] **pose pair → trajectory 데이터 생성**
- [ ] **궤적 품질 평가 메트릭**
- [ ] **학습용 궤적 데이터셋 구축**

### 3. **성능 최적화 및 확장**
현재 시스템의 성능 개선:

#### 3.1 Collision Detection 최적화
- [ ] **Spatial indexing 활용 (현재 취소된 작업 재검토)**
- [ ] **대용량 환경 (10K+ 장애물) 최적화**
- [ ] **GPU 기반 병렬 처리 검토**

#### 3.2 Memory 및 Storage 최적화
- [ ] **포즈 데이터 압축 저장**
- [ ] **메모리 효율적 배치 처리**
- [ ] **점진적 로딩 시스템**

---

## 📊 현재 프로젝트 상태

### 🎯 **완성도**
- **환경 생성 시스템**: 100% 완료
- **Pose 생성 시스템**: 100% 완료
- **시각화 시스템**: 100% 완료
- **기본 시뮬레이션**: 100% 완료
- **Concave 장애물**: 100% 완료

### 📈 **성능 지표**
- **Pose 생성**: 392+ poses/sec
- **환경 로딩**: <0.1s (일반 환경)
- **충돌 검사**: 실시간 (10K+ 점 환경)
- **데이터 규모**: 216MB (pointcloud 205MB + 코드 11MB)

### 🎮 **사용 가능한 기능들**
1. **빠른 환경 조회**: `utils/quick_visualize.py env_name env_number`
2. **Pose 생성**: `pose_pipeline.py env.ply robot_id --num_poses N`
3. **Concave 장애물**: `concave_shape_generator/` 전체 시스템
4. **실시간 시뮬레이션**: `main.py --env env_name --geometry N`
5. **비디오 녹화**: `record_video.py --env env_name --output video.mp4`

---

## 🔧 기술적 고려사항

### Batch Pose Generation 설계
```python
# pose/batch_generate_poses.py 예상 구조
class BatchPoseGenerator:
    def __init__(self):
        self.environments = self.discover_environments()
        self.robots = list(range(6))  # Robot IDs 0-5
        
    def generate_all_combinations(self):
        """모든 환경 × 로봇 조합에 대해 pose 생성"""
        
    def generate_pose_pairs(self):
        """유효한 {init, target} 쌍 생성"""
        
    def save_structured_dataset(self):
        """구조화된 형태로 데이터셋 저장"""
```

### 우선순위 이유
1. **Pose Dataset**: 현재 시스템이 완성되어 즉시 시작 가능
2. **실용적 가치**: 생성된 pose 데이터는 강화학습, 경로계획 등에 바로 활용
3. **확장성**: 대량 데이터 생성 후 trajectory planning 등으로 자연스럽게 확장

---

## ⚠️ 보류/취소된 작업들

### 취소된 이유가 있는 작업들
- ~~메모리 효율적인 겹침 검사 알고리즘~~ (현재 성능으로 충분)
- ~~대용량 환경 포인트클라우드 추출 최적화~~ (현재 성능으로 충분)
- ~~GUI 인터페이스~~ (CLI가 더 효율적)

---

**현재 상태**: 모든 핵심 시스템 완성, 다음 단계는 **Pose Dataset 대량 생산**
