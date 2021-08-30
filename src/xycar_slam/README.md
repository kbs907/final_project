<설정 파일> 은 config폴더에서
mapping, localization 파일 사용하면 됨.
default.lua는 기본 설정파일이고 xycar_mapping.lua는 허석님 파일인데 참고하려고 가져옴


<런치 파일>
마찬가지로 localization.launch와 mapping.launch를 사용하면 됨

<맵>
gangnam_map은 교차로쪽 맵이고 final_track은 전체 맵인데 불안정함

맵 저장 방법은
1. mapping.launch를 실행하여 한바퀴 주행
2. mapping 터미널 종료하지 말고 새 터미널 열기
3. 새 터미널에서 다음 커맨드 라인 두가지를 순차적으로 입력하는데, 두번째 명령어에서 your_map 말고 ***원하는 이름으로 맵 데이터 저장하기***
$ rosservice call /finish_trajectory 0
$ rosservice call /write_state "{filename: "$(rospack find xycar_slam)/maps/your_map.pbstream", include_unfinished_submaps: "true"}"




**해야할 일**
xycar_slam 하위에 맵 전체 주행하면서 lidar 토픽을 저장한 lidar.bag 파일이 있음
mapping.lua파일의 파라미터를 변경해보며
offline_mapping.launch 파일을 실행하여 매핑이 잘되는 최적의 값 찾기.
파라미터 튜닝 공식 가이드는 https://dreamofadd.tistory.com/144 에 링크가 있고람
제가 튜닝하면서 느낀 점 적은 글은 https://dreamofadd.tistory.com/146 이므로 참고 바
(만약 mapping.lua파일 사본을 만들어 다른 이름을 사용하거나 다른 bag파일 사용할시 launch파일에서 사용하는 파일 이름 수정!)
