# Programmers Autonomous-Driving Dev course. Mission Pass Competition

## Video
---


## Goal
---
주어진 미션을 수행하며 map 주행  
<img src="./image/map.png"/>

## Environment
---
- Ubuntu 18.04  
- ROS Melodic  
- Xycar model D  
- Nvidia TX 2  

## Structure
---
~~~
Project2
  └─ src
  │    └─ main_final.py                 # xycar control main code
  │    └─ drive_module.py               # xycar steering, speed control module
  │    └─ image_processing_module.py    # image processing Module
  │    └─ ar_module.py                  # AR tag control module
  │    └─ lidar_module.py               # lidar sensor control module
  │    └─ ultra_module.py               # ultrasonic sensor control module
  │    └─ yolo_module.py                # object detection control module
  └─ launch
        └─ final_project.launch          # xycar control main launch
~~~

## Usage
---
~~~bash
$ roslaunch final_project final_project.launch
~~~

## Procedure
---
### 1. Mode  
<img src="./image/mode.png"/>  

- mode 1: 차선 주행, 끼어들기 주행  
- mode 2: 정지선 인식, 신호등 인식, T 주차, YOLO Mission  
- mode 3: S자 곡선 주행, 경사로 주행  
- mode 4: 정지선 인식, 로터리 진입 판단  
- mode 5: 로터리 주행  
- mode 6: 장애물 회피 주행  
- mode 7: 정지선 인식, 신호등 인식, 평행 주차  

### 2. Change mode  
- mode 1: default  
- mode 2: 끼어들기 후, 한쪽 차선을 잃어버려서 회전 구간으로 판단하면 모드 2로 변경  
- mode 3: YOLO Mission을 수행한 후, 한쪽 차선을 잃어버려서 회전 구간으로 판단할 때 모드 3으로 변경  
- mode 4: 경사로를 검출하면 모드 4로 변경  
- mode 5: 정지선에서 멈춘 후, 왼쪽에 차량이 없다고 판단되면 모드 5로 변경  
- mode 6: 로터리 회전 중 기둥이 인식되지 않으면 모드 6으로 변경  
- mode 7: 평행 주차를 위한 AR tag의 거리 조건으로 모드 7 변경  

### 3. Lane Detection
① Calibration  
② ROI 설정  
③ Grayscale  
④ Gaussian Blur  
⑤ Canny Edge Detecion  
⑥ Hough Transform으로 후보 Line 선정  
⑦ 양쪽 차선 모두 검출되었으면 오른쪽 차선 기준으로 주행  
⑧ 왼쪽 차선만 검출되었다면 왼쪽 차선 기준으로 주행  
⑨ 오른쪽 차선만 검출되었다면 오른쪽 차선 기준으로 주행  
⑩ 양쪽 차선이 검출되지 않았다면 우회전  
⑪ PID Control을 통한 최종 Steering 제어  

### 4. Recognition traffic light
① Calibration  
② ROI 설정  
③ Gaussian Blur  
④ HSV 채널로 Split  
⑤ 첫번째 신호등과 두번째 신호등에 맞게 cv2.HoughCircles 함수로 원 검출  
⑥ 검출된 원의 개수가 3개 이상 검출되었을 때의 조건 설정  
⑦ 검출된 원들이 모여있는지 조건 설정  
⑧ 검출된 원들 중 가장 오른쪽 원을 초록불로 지정  
⑨ 초록불의 중심 V 값을 기준으로 ON / OFF 판단  

### 5. Lane Barge
① 차선 폭이 좁아지는 지점을 끼어들기 구간으로 판단  
② 끼어들기 구간에 도착하면 좌측 전면 라이다와 좌측 후면 초음파로 주변 차량 판단 후 진입  

### 6. Recognition stop line
① 차선을 기준으로 ROI 설정  
② 이진화(밝은 부분을 0)  
③ cv2.countNonZero 함수를 사용해서 0 개수 파악 후, 정지선 인식  

### 7. T parking
① AR tag와의 distance를 기준으로 시작 위치 파악  
② 시작 위치에서 AR tag와의 arctan를 기준으로 후진 주차

### 8. YOLO Mission
① Tiny YOLO v2 voc 모델을 사용해서 'person' class 검출  
② 'person' class의 크기를 판단해서 정지 후 검출할 class를 'cat'으로 변경  
③ 'cat' class의 크기를 판단해서 정지  

### 9. Driving on a ramp
① 특정 범위를 ROI로 선정  
② cv2.countNonZero 함수를 이용해서 경사로 인식  
③ 경사로일때 속도를 높여서 오르막길 주행  
④ 내리막길일때 정지해서 정지선을 넘어가지 않도록 주행  

### 10. Driving rotary
① 좌측 전면 라이다 센서로 주행 중인 차량 판단 후 주행  

### 11. Obstacle avoidance driving
① 기본적 차량 주행  
② YOLO로 갓길 주차되어 있는 차량 검출  
③ 차량이 검출된다면 차선 위치 조정해서 주행  

### 12. Parallel parking
① Bird's eyes view 형태로 원근 변환  
② Grayscale  
③ 이진화  
④ ROI 설정(화면의 오른쪽 부분)  
⑤ cv2.countNonZero 함수로 주차 판단 구역까지 이동  
⑥ 주차 판단 구역에서 우측 초음파 센서를 이용해서 주차 판단  
⑦ 평행 주차 실행  

## Problems & Resolution
---
