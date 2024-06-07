## 실행 순서
Terminal 1

'''
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
'''

Terminal 2
'''
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
Terminal 3
'''  
roslaunch waypoint save_point.launch
'''

## 사용법
본 패키지는 waypoint 패키지만 있으면 작동함 (+ caselab_rviz_plugin)

### 1. move_path 모드
로봇의 waypoint를 찍어 node를 yaml file로 저장함.

1. Rviz의 왼쪽 하단의 Make SIP_Node에서 --SELECT--를 클릭 후 move_path 선택
2. Send! 버튼 누르기 (-> move_path 모드로 바뀜)
3. Rviz 상단의 첫번째 Publish Point을 통해 이동할 지점들 클릭
4. Rviz 상에서 node 및 edge가 나타남.
-> 이후 아래 명령어로 waypoint navigation 실행 가능
'''
rosrun waypoint waypoint_navigation
'''
5. waypoint/config/move_paths.yaml 파일에서 해당 노드들 확인 가능

### 2. object_local 모드
object를 표시하기 위해 각 대각선의 좌표를 찍어 object들을 yaml file로 저장함.

1. Rviz의 왼쪽 하단의 Make SIP_Node에서 --SELECT--를 클릭 후 object_local 선택
2. Object란에 해당 object의 이름 입력
3. Send! 버튼 누르기 (-> object_local 모드로 바뀜)
4. Rviz 상단의 두번째 Publish Point을 통해 해당 오브젝트를 설정할 사각형의 대각선 좌표 해당하는 점 2개 클릭
5. waypoint/config/save_objects.yaml 파일에서 해당 object들 확인 가능

### 3. polygon_local 모드
polygon를 표시하기 위해 각 대각선의 좌표를 찍어 polygon들을 yaml file로 저장함.
(추후 사각형이 아닌 다각형 형태로 업데이트할 예정)

1. Rviz의 왼쪽 하단의 Make SIP_Node에서 --SELECT--를 클릭 후 polygon_local 선택
2. Polygon란에 해당 polygon의 이름 입력
3. Send! 버튼 누르기 (-> polygon_local 모드로 바뀜)
4. Rviz 상단의 세번째 Publish Point을 통해 해당 polygon를 설정할 사각형의 대각선 좌표 해당하는 점 2개 클릭 (추후 사각형이 아닌 다각형 형태로 업데이트할 예정)
5. waypoint/config/save_polygon.yaml 파일에서 해당 polygon들 확인 가능


    
