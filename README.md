# Airport_docking_Project
# 이전 진행상황

### Simulation
- ROS 기반 모바일 로봇 turtlebot3를 오픈 소스 3D 로봇 공학 시뮬레이터인 Gazebo 환경에서 시뮬레이션
Gazebo 환경에 임시로 Charging Station을 설치하여 시뮬레이션 진행
- LiDAR에서 측정하는 data와 Docking 알고리즘의 연산 결과를 rviz에서 실시간으로 시각화 하며 테스트
![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B0%80%EC%A0%9C%EB%B3%B4.png?raw=true)
### Roi Filtering
- 센서에서 측정되는 Point Cloud를 줄여 불필요한 연산 부하를 줄이기 위해 사용
<ROI Filtering 전>

![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/ROI%201.png?raw=true)
<ROI Filtering 후>

![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/ROI2.png?raw=true)

### Euclidean Clustering
- 가장 간단한 방법의 군집화 방법으로 두 점 사이의 유클리드 거리를 계산하여, 특정 거리 이하일 경우 동일한 군집으로 간주
<Euclidean Clustering>

![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EC%97%90%EB%93%80%EC%8B%9C%EC%95%88.png?raw=true)


### RANSAC
- RANSAC은 inlier와 outlier를 구분하는 간편하면서도 효과적인 알고리즘
- 포인트 클라우드가 형성되는 패턴에 따라 charging station의 중심부와 외곽부를 각각 inlier, outlier로 설정하여  두 클라스를 간단하면서 효율적으로 분리 가능

<inlier로 검출된 station 중심부>

![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EB%9E%9C%EC%82%AD1.png?raw=true)

<outlier로 검출된 station 중심부>

![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EB%9E%9C%EC%82%AD2.png?raw=true)
# 현재 진행상황
### RANSAC Line Segmentaion 
- 포인트 클라우스에서 선형 모델을 찾아내는 알고리즘 수행
- RANSAC Line Segmentaion (+ top point, bottom point) 


# 6/11 진행상황
- Bottom point, Top point를 map frame 좌표로 변환 (map.py)
- 주어진 좌표로 이동하는 파이썬 알고리즘 구현 중.

# 6/12 진행상황
- movepoint2.py : 로봇이 Bottom_point로 이동후 3초뒤에 Top_point쪽으로 회전 후 이동시키는 시나리오 구현.
# 6/13 진행상황
- Top point 수정 : 기존의 방법이었던 docking station의 가장 높은 y축을 Top_point로 지정하였던 방법을 대체하였음.
이 방법은 Top_point가 실시간으로 불안정한 모습을 보임. 
![Toppoint](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/bd801a10-9738-4c9e-ada4-b73d4c2621bc)
- Top_point > arrive_point로 대체 : 기존의 방법을 대체하여 docking station의 가로측의 중간값(station부분의 포인트들의 x값 평균값)으로 설정하여 구현. 기존의 point보다 안정적인 모습을 보임.    
![arrive_point](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/e2d01c52-16d3-4c75-8ce1-d45fc458d957)

- move_point2 : Top_point 수정과 파라미터 수정으로 인해 로봇 주행이 이전보다 훨씬 정확해짐.
![docking](https://github.com/gihoonbackend/Airport_docking_Project/assets/126891083/531e82e0-83a1-46d2-95cf-fd2216e09242)




### rull 
- laser2pc - launch - docking.launch 파일에 필요한 파이썬 코드들 통합해놓은 상태.
- 런치 실행 후, 발행되어진 토픽(line, bottom point, top point) 실행 
- 로봇은 충전 스테이션에 가깝게 위치해놓아야 코드가 제대로 실행되어짐. 
- line_sgh.py : 클러스터링 되어진 부분을 이용하여 충전 스테이션의 outline으로는 가로 선분을 생성시키고, inline부분에서는 중간 좌표를 계산하여 가로선분을 수직으로 지나치는 세로 선분을 생성함.
![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC1.png?raw=true)
![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC2.png?raw=true)
![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC3.png?raw=true)
- top_point : inline 부분의 가장 높은 좌표를 추출하고 마커로 표시.

- bottom_point : line_sgh.py에서 생성시킨 세로선분의 가장 아래 포인트의 좌표를 추출하고 마커로 표시.
![RANSAC Line](https://github.com/gihoonbackend/Airport_docking_Project/blob/main/image/%EA%B7%B8%EB%A6%BC4.png?raw=true)
- bottom_marker_sgh.py 는 map frame 기준 좌표가 추출되고, bottom_marker copy.py는 로봇 좌표계 기준 좌표가 발행됨( clustering_ransac_ros_sgh도 마찬가지임.)
- 추출한 좌표를 이용해 로봇을 이동 시키는 코드 (아직 구현중임) 테스트 중인 코드 이름: follow_line_node.py, movepoint.py, movepoint2.py > movepoint2.py 구현 완료( 세부 조정 필요 )
