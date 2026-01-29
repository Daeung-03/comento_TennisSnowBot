# TennisCourt_SnowRemoval

![Python](https://img.shields.io/badge/python-3.11+-blue.svg)
![Simulator](https://img.shields.io/badge/simulator-AutoNavSim2D-orange.svg)

**Autonomous Snow Removal Robot Simulation for Tennis Courts**  
테니스장 전용 자율주행 제설 로봇 시뮬레이션 프로젝트입니다.

---

## 📖 Table of Contents
- [About](#-about)
- [Features](#-features)
- [Project Structure](#-project-structure)
- [Installation](#-installation)
- [Usage](#-usage)
- [Algorithm Details](#-algorithm-details)
- [Custom Map Generation](#-custom-map-generation)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🧐 About

이 프로젝트는 **AutoNavSim2D** 시뮬레이터를 기반으로, 테니스 코트 환경에서 자율주행 로봇이 눈(Snow)을 감지하고 효율적으로 제거하는 과정을 시뮬레이션합니다.

단순히 전체 영역을 cover하는 것이 아닌, **제설이 필요한 구역** 감지하여, 효율적으로 주행하는 제설 로봇의 동작을 구현했습니다.

### 🎯 Objective
- **Perception**: 테니스 코트 내 무작위로 쌓인 눈 영역(Cluster) 인식
- **Planning**: 다수의 눈 영역을 최단 거리로 순회하는 전역 경로 생성
- **Coverage**: 각 눈 영역 내부를 boustrophedon 패턴으로 빈틈없이 제설
- **Control**: 생성된 경로를 정밀하게 추종하는 모션 제어

---

## ✨ Features

- **Dynamic Snow Detection**: DBSCAN 클러스터링을 이용한 실시간 눈 영역 인식
- **Smart Path Planning**:
  - **A\***: 장애물(네트, 라인)을 회피하여 클러스터 간 이동
  - **Boustrophedon Coverage**: 효율적인 제설을 위한 영역 채우기 패턴
- **Realistic Environment**: 실제 테니스 코트 규격(23.77m x 10.97m) 비율을 반영한 맵
- **Interactive Simulation**:
  - 시작 위치 자유 선택
  - 실시간 경로 시각화 및 로봇 모니터링

---

## 📂 Project Structure

```text
.
├── main.py                 # 🚀 메인 실행 파일
├── requirements.txt        # 의존성 패키지
├── maps/                   # 맵 데이터 저장소 (.pkl)
├── src/
│   ├── perception/         # [인식] 눈 감지 (DBSCAN)
│   ├── control/            # [제어] 경로 계획 (A* + Zigzag)
│   └── launch/             # [실행] 통합 래퍼 (Simulator Wrapper)
├── tools/                  # 유틸리티 (맵 생성기)
└── examples/               # 기능별 테스트 예제
```

---

## ⚙️ Installation

### 1. Clone Repository
```bash
git clone https://github.com/Daeung-03/TennisCourt_SnowRemoval.git
cd TennisCourt_SnowRemoval
```

### 2. Set up Virtual Environment (Recommended)
```bash
python -m venv .venv
source .venv/bin/activate  # Mac/Linux
# .venv\Scripts\activate   # Windows
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```
> **Requirements**: `autonavsim2d`, `pygame`, `numpy`, `scikit-learn`

---

## 🚀 Usage

### 1. Run Simulation
가장 기본적인 실행 방법입니다.

```bash
python main.py
```

1. 시뮬레이터 창이 열립니다.
2. 맵에서 **로봇의 시작 위치(빨간 점)**를 클릭합니다.
3. 맵의 아무 곳이나 클릭합니다(초록 점).
4. **`Plan Path`** 버튼을 클릭하면 경로를 생성합니다(로딩이 걸립니다).
5. 우측 메뉴의 **`Navigate`** 버튼을 클릭하면 제설이 시작됩니다.

### 2. Generate New Map
새로운 눈 배치를 가진 맵을 생성하려면 맵 생성기를 실행하세요.

```bash
python tools/tenniscourt_map_gen.py
```
> 생성된 맵은 `maps/TennisCourt_Snow.pkl`로 저장됩니다.

### 3. Test Modules
각 기능별로 독립적인 테스트가 가능합니다.
- **인식(Perception) 테스트**: `python examples/perception_ex.py`
- **제어(Control) 테스트**: `python examples/control_ex.py`

---

## 🧠 Algorithm Details

### 1. Perception (Snow Detection)
- **Algorithm**: DBSCAN (Density-Based Spatial Clustering of Applications with Noise)
- **Process**: 맵의 파란색 픽셀(눈)을 밀도 기반으로 군집화하여 Bounding Box를 추출합니다.

### 2. Global Planning (TSP-like)
- **Algorithm**: Greedy Approach + A*
- **Process**: 현재 로봇 위치에서 가장 가까운 눈 클러스터를 탐색하여 방문 순서를 결정합니다.

### 3. Local Planning (Coverage)
- **Algorithm**: Boustrophedon (Ox-turning) Decomposition
- **Process**: 클러스터 내부를 ‘ㄹ’자 형태(boustrophedon)로 주행하여 영역을 완전히 커버합니다.

## 📝 License

This project is a personal project(for learning) utilizing the `AutoNavSim2D` simulator.
All rights to `AutoNavSim2D` belong to its original creator.

## 🙏 Acknowledgments

- **Simulator**: [AutoNavSim2D](https://github.com/yendiDev/autonavsim2d) by yendiDev
- **Library**: Pygame Community
```