# Airport Docking Project

## 📌 프로젝트 개요
ROS 기반 모바일 로봇 **TurtleBot3**를 활용하여 **충전 스테이션 자동 도킹(Docking) 알고리즘**을 구현하는 프로젝트입니다.  
LiDAR 기반 Point Cloud 데이터를 활용하여 충전 스테이션을 인식하고 로봇을 도킹 위치로 이동시키는 시스템을 개발합니다.

---

# 🚀 이전 진행 상황

## Simulation
- ROS 기반 모바일 로봇 **TurtleBot3**를 **Gazebo 시뮬레이터** 환경에서 테스트
- Gazebo 환경에 **Charging Station 모델을 임시 설치**
- LiDAR 데이터와 Docking 알고리즘 결과를 **RViz에서 실시간 시각화**

![Gazebo Simulation](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B0%80%EC%A0%9C%EB%B3%B4.png?raw=true)

---

## ROI Filtering
센서에서 측정되는 **Point Cloud 데이터의 양을 줄여 불필요한 연산을 감소**시키기 위해 사용

### ROI Filtering 전
![ROI Before](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/ROI%201.png?raw=true)

### ROI Filtering 후
![ROI After](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/ROI2.png?raw=true)

---

## Euclidean Clustering
가장 기본적인 **군집화(Clustering) 알고리즘**

- 두 점 사이의 **유클리드 거리(Euclidean Distance)** 계산
- 특정 거리 이하일 경우 동일한 군집으로 분류

![Euclidean Clustering](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EC%97%90%EB%93%80%EC%8B%9C%EC%95%88.png?raw=true)

---

## RANSAC
**RANSAC(Random Sample Consensus)** 알고리즘을 활용하여  
Point Cloud에서 **Inlier와 Outlier를 분리**

- Charging Station 중심부 → **Inlier**
- 외곽부 → **Outlier**

### Inlier 검출 (Station 중심부)
![RANSAC Inlier](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EB%9E%9C%EC%82%AD1.png?raw=true)

### Outlier 검출
![RANSAC Outlier](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EB%9E%9C%EC%82%AD2.png?raw=true)

---

# ⚙️ 현재 진행 상황

## RANSAC Line Segmentation
- Point Cloud에서 **선형 모델(Line Model)** 추출
- **RANSAC Line Segmentation**
- **Top Point / Bottom Point 계산**

---

# 📅 진행 기록

## 6/11
- Bottom Point, Top Point를 **map frame 좌표로 변환 (map.py)**
- 해당 좌표로 이동하는 **Python 이동 알고리즘 구현 진행**

---

## 6/12
- `movepoint2.py`
- 로봇이 **Bottom Point로 이동 → 3초 후 Top Point 방향으로 회전 후 이동**하는 시나리오 구현

---

## 6/13
### Top Point 수정

기존 방식
- Docking Station의 **가장 높은 y좌표를 Top Point로 설정**

문제
- Top Point가 **실시간으로 불안정하게 변동**

![Top Point Issue](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/bd801a10-9738-4c9e-ada4-b73d4c2621bc)

### 새로운 방식

Top Point → **Arrive Point**

- Docking Station **가로 방향 포인트들의 평균값(x 평균)** 사용
- 기존 방식보다 **안정적인 위치 추정**

![Arrive Point](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/e2d01c52-16d3-4c75-8ce1-d45fc458d957)

---

# 📅 7/10 진행 상황

- 실제 **Charge Station 형태와 동일한 모델 추가**
- 모델에 맞춰 **알고리즘 수정**

수정 내용
- Station **중간점 Docking**
- **Top Point Tracking**

---

# 🎥 Simulation Results

## Simulation Version 1
- `move_point2`
- Top Point 수정 및 파라미터 조정으로 **로봇 주행 정확도 향상**

![Docking Simulation](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/531e82e0-83a1-46d2-95cf-fd2216e09242)

---

## Simulation Version 2

실제 Charge Station 모델을 제작하여 **현장 적용 전 테스트**

### 모델
![Docking Model](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/model.png?raw=true)

### 알고리즘
- **Top Point + Bottom Point 기반 Docking**

![Docking Simulation 2](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/a26ae611-959a-44e9-8c0f-2afb32e62cf7)

---

# 📂 실행 방법 (Run)

### 실행 순서

1. `docking.launch` 실행
2. 다음 토픽 실행
   - line
   - bottom_point
   - top_point

⚠️ 로봇이 **충전 스테이션 근처에 위치해야 알고리즘이 정상 동작**

---

# 🧠 주요 알고리즘

## line_sgh.py

클러스터링 결과를 활용하여 충전 스테이션의 **윤곽선(outline)**을 계산

### 처리 과정

1️⃣ **가로 선분 생성** (Station Outline)

2️⃣ **세로 선분 생성**
- 중간 좌표 계산
- 가로선분을 수직으로 통과하는 세로선 생성

![Line Detection](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC1.png?raw=true)
![Line Detection](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC2.png?raw=true)
![Line Detection](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC3.png?raw=true)

---

## Top Point

- Inline 영역의 **가장 높은 좌표 추출**
- RViz Marker로 표시

---

## Bottom Point

- `line_sgh.py`에서 생성된 **세로선의 가장 아래 좌표**
- RViz Marker로 표시

![Bottom Point](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC4.png?raw=true)

---

# 📍 좌표 시스템

- `bottom_marker_sgh.py`
  - **Map Frame 기준 좌표**

- `bottom_marker copy.py`
  - **Robot Frame 기준 좌표**

- `clustering_ransac_ros_sgh`
  - 동일한 좌표 체계 사용

---

# 🤖 로봇 이동 코드

추출한 좌표를 이용해 로봇을 이동

현재 테스트 코드
