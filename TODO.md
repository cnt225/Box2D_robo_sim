# TODO List - Robot Simulation Project

## 프로젝트 개요
Box2D 기반 3-link 로봇 팔 2D 시뮬레이션 프로젝트. 목표 위치로 이동하면서 장애물을 회피하는 로봇 제어 시뮬레이션.

## 완료된 주요 리팩토링 (2025-07-05)

### 🏗️ **이중 모듈 아키텍처 구현 완료**
**이전**: 단일 모듈에서 모든 기능 처리 (복잡한 CLI, 혼재된 책임)
**현재**: 깔끔한 분리 구조

#### Module 1: pointcloud/ (환경 데이터 생성 및 관리)
```
pointcloud/
├── create_pointcloud.py       # 모든 pointcloud 생성 옵션 집중
├── pointcloud_extractor.py    # 물리 환경에서 pointcloud 추출
├── pointcloud_loader.py       # PLY 파일 로딩 및 환경 재구성
└── data/                      # PLY 형식 pointcloud 파일들
```

#### Module 2: main/ (시뮬레이션 실행)
```
robot_sim_v1/
├── main.py                    # 실시간 시뮬레이션 (간소화된 CLI)
├── record_video.py            # 비디오 녹화 (간소화된 CLI)
├── env.py                     # 환경 생성 (geometry config 기반)
├── robot_config.py            # 로봇 geometry 설정 (6가지 미리 정의)
├── policy.py                  # 제어 정책들
├── simulation.py              # 핵심 시뮬레이션 로직
└── render.py                  # 시각화
```

### 🎯 **CLI 대폭 간소화 완료**
**이전 복잡한 CLI** (15+ 옵션):
```bash
python main.py --target 5 5 --link-shape rectangle --clustering-eps 0.3 --min-samples 5 --obstacle-type polygon --noise-level 0.01 --resolution 0.05 [...]
```

**현재 간소화된 CLI** (4개 핵심 옵션):
```bash
# 정적 환경
python main.py --target 5.0 5.0 --env static --geometry 2 --policy potential_field_pd

# pointcloud 환경  
python main.py --target 6.0 4.0 --env clean_env.ply --geometry 1 --policy rmp

# 비디오 녹화
python record_video.py --target 5.0 5.0 --env static --geometry 2 --duration 10 --output demo.mp4
```

### 🔧 **핵심 기능 검증 완료**
- ✅ **정적 환경**: 미리 정의된 장애물 배치로 즉시 시뮬레이션 가능
- ✅ **pointcloud 환경**: PLY 파일에서 환경 로딩 및 장애물 재구성
- ✅ **Geometry 선택**: `--list-geometries`로 6가지 로봇 설정 선택 가능
- ✅ **정책 선택**: potential_field, potential_field_pd, rmp 중 선택
- ✅ **비디오 녹화**: MP4 형식으로 시뮬레이션 결과 저장

### 🗂️ **파일 정리 및 표준화**
- ✅ **레거시 파일 제거**: `env_new.py`, 구형 pointcloud 형식들
- ✅ **PLY 형식 표준화**: 모든 pointcloud 데이터를 PLY 형식으로 통일
- ✅ **메타데이터 임베딩**: clustering/obstacle 설정을 PLY 헤더에 포함
- ✅ **Gitignore 업데이트**: 불필요한 파일들 제외

### 📚 **문서화 완료**
- ✅ **README.md 완전 재작성**: 새로운 아키텍처와 워크플로우 반영
- ✅ **이중 모듈 워크플로우 가이드**: pointcloud 생성 → 시뮬레이션 실행
- ✅ **명령어 레퍼런스**: 간소화된 CLI 옵션들 설명
- ✅ **문제 해결 가이드**: 새로운 구조에 맞는 트러블슈팅

### 🐛 **주요 버그 수정**
- ✅ **env_type 오류**: `args.env_type` → `args.env` 수정
- ✅ **정적 환경 인식**: 'static' 값을 pointcloud 파일로 잘못 인식하던 문제 해결
- ✅ **geometry config 적용**: robot_config.py 시스템을 환경 생성에 완전 통합
- ✅ **타원형 링크 계산**: geometry config 기반 동적 크기 계산

## 현재 상태 (2025-07-05)
✅ 이중 모듈 아키텍처 구현 완료
✅ 간소화된 CLI 시스템 완료
✅ 기본 기능 검증 완료 (정적/pointcloud 환경, 비디오 녹화)
✅ 문서화 완료

## 우선순위 높음 🔥

### 1. Configuration 시스템 개선
- ✅ **robot_config.py → config.yaml 이전**
  - ✅ 현재 코드에 하드코딩된 robot geometry 설정들을 config.yaml로 이동
  - ✅ 6개 기본 geometry 외에 더 다양한 설정 추가 (max_reach 포함)
  - ✅ Link lengths, widths, shapes 등 모든 robot 파라미터 통합 관리
  - ✅ ConfigLoader 클래스 구현으로 yaml 기반 설정 로딩
  - ✅ robot_config.py를 config.yaml 기반으로 재작성
  - ✅ 기능 검증 완료 (--list-geometries, 시뮬레이션 실행)
  
- [ ] **분산된 설정 요소들 통합**
  - [ ] main.py, record_video.py의 하드코딩 파라미터들 config.yaml로 이전
  - [ ] pointcloud 생성 관련 기본값들 (resolution, noise_level, clustering 등)
  - [ ] 시뮬레이션 파라미터들 (FPS, screen size, physics 설정 등)
  - [ ] Control policy 관련 파라미터들
  - [ ] 모든 설정을 config.yaml에 중앙 집중식 관리

### 2. Robot Configuration 확장
- [ ] **적절한 robot geometry 생성**
  - 다양한 작업 영역을 커버하는 geometry 설정
  - 소형 (precision work), 중형 (general purpose), 대형 (extended reach) 로봇
  - Rectangle/Ellipse 형태별 최적화된 설정
  - 각 geometry의 reach, workspace, 특성 문서화

### 3. Policy 시스템 개선
- [ ] **Control policy 적용 방안 검토**
  - 현재 potential_field, potential_field_pd, rmp 정책 성능 분석
  - 각 정책별 최적 파라미터 튜닝
  - Geometry별 최적 정책 매칭
  - 새로운 정책 추가 가능성 검토 (A*, RRT 등)

## 우선순위 중간 📋

### 4. Environment 데이터 대량 생산 시스템
- [ ] **자동 환경 생성 시스템 구축**
  - 사각형, 원, 다각형 등 다양한 형태 장애물 자동 배치
  - 장애물 개수, 크기, 위치를 랜덤하게 생성하는 시스템
  - 시드 기반 재현 가능한 환경 생성
  - 난이도별 환경 생성 (simple, medium, complex)

- [ ] **Batch 생성 도구 개발**
  - `pointcloud/batch_generate.py` 스크립트 생성
  - 지정된 개수만큼 다양한 환경 자동 생성
  - 환경별 메타데이터 관리 (난이도, 장애물 정보 등)
  - 생성된 환경들의 품질 검증 시스템

### 5. 코드 품질 개선
- [ ] **코드 간소화 및 정리**
  - 중복 코드 제거
  - 함수/클래스 책임 분리 명확화
  - 타입 힌트 추가
  - Docstring 보완

- [ ] **에러 처리 강화**
  - 파일 로딩 실패 시 graceful fallback
  - 잘못된 geometry ID 처리 개선
  - 시뮬레이션 오류 시 복구 메커니즘

## 우선순위 낮음 📝

### 6. 기능 확장
- [ ] **성능 최적화**
  - pointcloud 로딩 속도 개선
  - 렌더링 성능 최적화
  - 대용량 환경 데이터 처리 개선

- [ ] **사용성 개선**
  - GUI 인터페이스 추가 고려
  - 실시간 파라미터 조정 기능
  - 시뮬레이션 결과 분석 도구

- [ ] **문서화 개선**
  - API 문서 자동 생성
  - 튜토리얼 비디오 제작
  - 사용 사례별 가이드 작성

## 기술적 고려사항 🔧

### Configuration 시스템 설계
```yaml
# config.yaml 예상 구조
robot_geometries:
  - id: 1
    name: "Compact Rectangle Robot"
    link_lengths: [2.5, 2.0, 1.5]
    link_widths: [0.25, 0.2, 0.15]
    link_shape: "rectangle"
    max_reach: 6.0
    
simulation:
  fps: 60
  screen_width: 800
  screen_height: 600
  time_step: 0.0167
  
pointcloud:
  default_resolution: 0.05
  default_noise_level: 0.01
  default_clustering_eps: 0.3
  default_min_samples: 5
  
policies:
  potential_field:
    attractive_gain: 1.0
    repulsive_gain: 2.0
    repulsive_range: 1.0
```

### Environment 생성 시스템 설계
```python
# batch_generate.py 예상 인터페이스
python pointcloud/batch_generate.py \
  --count 100 \
  --difficulty easy,medium,hard \
  --obstacle_types rectangle,circle,polygon \
  --min_obstacles 2 \
  --max_obstacles 8 \
  --output_prefix batch_env
```

## 현재 코드 구조 및 핵심 파일들

### 핵심 설정 파일
- **`robot_config.py`**: 6가지 로봇 geometry 정의 (현재 하드코딩, config.yaml로 이전 필요)
- **`env.py`**: 환경 생성 통합 모듈 (정적/pointcloud 환경 선택적 로딩)
- **`policy.py`**: 3가지 제어 정책 (potential_field, potential_field_pd, rmp)

### 현재 Robot Geometry 설정 (robot_config.py)
```python
ROBOT_GEOMETRIES = {
    0: {"name": "Compact Rectangle Robot", "link_lengths": [2.5, 2.0, 1.5], "link_widths": [0.25, 0.2, 0.15], "link_shape": "rectangle"},
    1: {"name": "Standard Rectangle Robot", "link_lengths": [3.0, 2.5, 2.0], "link_widths": [0.3, 0.25, 0.2], "link_shape": "rectangle"},
    2: {"name": "Extended Rectangle Robot", "link_lengths": [3.5, 3.0, 2.5], "link_widths": [0.35, 0.3, 0.25], "link_shape": "rectangle"},
    3: {"name": "Compact Ellipse Robot", "link_lengths": [2.5, 2.0, 1.5], "link_widths": [0.25, 0.2, 0.15], "link_shape": "ellipse"},
    4: {"name": "Standard Ellipse Robot", "link_lengths": [3.0, 2.5, 2.0], "link_widths": [0.3, 0.25, 0.2], "link_shape": "ellipse"},
    5: {"name": "Extended Ellipse Robot", "link_lengths": [3.5, 3.0, 2.5], "link_widths": [0.35, 0.3, 0.25], "link_shape": "ellipse"}
}
```

### 분산된 설정 요소들 (통합 필요)
```python
# main.py에 하드코딩
SCREEN_W, SCREEN_H = 800, 600
FPS = 60
TIME_STEP = 1.0 / FPS

# create_pointcloud.py 기본값들
default_resolution = 0.05
default_noise_level = 0.01
default_clustering_eps = 0.3
default_min_samples = 5

# policy.py 파라미터들 (하드코딩)
attractive_gain = 1.0
repulsive_gain = 2.0
# 등등...
```

### 현재 워크플로우
1. **pointcloud 생성** (선택사항):
   ```bash
   cd pointcloud
   python create_pointcloud.py --output_name my_env --clustering_eps 0.25 --obstacle_type polygon
   ```

2. **시뮬레이션 실행**:
   ```bash
   python main.py --target 5.0 5.0 --env my_env.ply --geometry 2 --policy potential_field_pd
   # 또는 정적 환경: --env static
   ```

3. **비디오 녹화**:
   ```bash
   python record_video.py --target 5.0 5.0 --env my_env.ply --geometry 2 --duration 10 --output demo.mp4
   ```

### 테스트된 기능들
- ✅ `python main.py --target 5.0 5.0 --geometry 2 --env static --policy potential_field_pd`
- ✅ `python main.py --target 6.0 4.0 --geometry 1 --env clean_env --policy rmp`  
- ✅ `python record_video.py --target 5.0 5.0 --geometry 2 --env static --duration 5 --output test_static.mp4`
- ✅ `python main.py --list-geometries` (geometry 목록 출력)

### 알려진 제한사항
- Robot geometry가 코드에 하드코딩되어 있어 설정 변경이 어려움
- 환경 생성이 수동적 (대량 생산 시스템 없음)
- Policy 파라미터 튜닝이 코드 수정을 통해서만 가능
- 설정들이 여러 파일에 분산되어 관리 복잡

---

**마지막 업데이트**: 2025-07-05  
**현재 브랜치**: main  
**작업자**: GitHub Copilot + User

## 다음 작업 세션 준비사항

### 🎯 **즉시 시작 가능한 작업들**

#### 1. Configuration 통합 (우선순위 1)
**목표**: 모든 설정을 config.yaml로 중앙 집중화
**현재 문제**: 설정들이 robot_config.py, main.py, create_pointcloud.py 등에 분산

**구체적 작업**:
1. `config.yaml` 파일 생성 및 구조 설계
2. `robot_config.py`의 ROBOT_GEOMETRIES를 config.yaml로 이전
3. 각 모듈에서 config.yaml을 읽어오는 Config 클래스 구현
4. 하드코딩된 시뮬레이션 파라미터들 config.yaml로 이전

#### 2. Robot Geometry 확장 (우선순위 2)  
**목표**: 다양한 용도의 로봇 설정 추가
**현재 상태**: 6개 기본 설정 (compact/standard/extended × rectangle/ellipse)

**구체적 작업**:
1. 정밀 작업용 소형 로봇 (짧은 링크, 높은 정확도)
2. 중거리 작업용 표준 로봇 (균형잡힌 설정)  
3. 장거리 작업용 대형 로봇 (긴 링크, 넓은 작업영역)
4. 각 geometry별 최적 policy 매칭 연구

#### 3. 환경 대량 생산 시스템 (우선순위 3)
**목표**: 다양한 환경을 자동으로 대량 생성
**현재 상태**: 수동으로 create_pointcloud.py 실행

**구체적 작업**:
1. `pointcloud/batch_generate.py` 스크립트 개발
2. 장애물 형태별 (사각형, 원, 다각형) 랜덤 배치 알고리즘
3. 난이도별 환경 생성 (장애물 개수/크기/밀도 기반)
4. 시드 기반 재현 가능한 생성 시스템

### 📋 **필요한 파일 목록**
새로 만들어야 할 파일들:
- `config.yaml` - 모든 설정 통합
- `config_loader.py` - 설정 파일 로딩 유틸리티
- `pointcloud/batch_generate.py` - 환경 대량 생성
- `pointcloud/environment_validator.py` - 생성된 환경 품질 검증

수정해야 할 파일들:
- `robot_config.py` → config.yaml 기반으로 재작성
- `main.py`, `record_video.py` → config 파라미터 적용
- `create_pointcloud.py` → config 기반 기본값 사용
- `policy.py` → config 기반 파라미터 튜닝

### 🚀 **빠른 시작 가이드** (다음 세션용)
1. **README.md**와 이 **TODO.md** 파일을 먼저 읽어서 전체 상황 파악
2. 현재 작업 중인 파일들 확인: `robot_config.py`, `env.py`, `main.py`
3. 테스트 명령어로 현재 상태 확인:
   ```bash
   python main.py --list-geometries
   python main.py --target 5.0 5.0 --env static --geometry 1
   ```
4. 첫 번째 작업: `config.yaml` 설계부터 시작
