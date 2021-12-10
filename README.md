# erion_pi
자율주행 전동카트 
1) 개발환경
  ROS : Linux Ubuntu 18.04 – Ros melodic
  Application : React Native ( expo Cli )
  Server & DB : Fire Base (google)
  Language : python3 ( ROS / HW  Server )   , React & JavaScript ( Font-end)
  
2) 개발목표
  수동조작 기능 : 키보드 또는 Application으로 실시간 동작이 가능해야 한다.
  자율주행 기능 : 라이다를 이용하여 장애물을 회피하면서 사람을 탐지하고 쫓아간다.
  차량탑승 기능 : 2개의 BLDC 모터 및 후륜의 인휠모터를 활용하여 차량에 탑승해야 한다.
  통신 : 모드를 실시간으로 변경할 수 있어야 하며, 원격 통신이 가능해야한다.
  
3) 기능상세
- Jetson nano
    기능 : 
    1) Logitech camera를 통해 비디오를 받아옴 
         ( node : camera , publisher : 카메라 이미지 (cv2bridge) )
    2)  MobilNet (google) 를 통해 카메라에서 받아온 이미지에서 추적할      대상을 찾음 ( node : mobilnet , subscriber : 카메라 이미지) 
       - Custom 학습된 모델에서 추적할 대상을 찾음 ( 사람 / 자동차 번호판)
    3) 추적할 대상의 이미지의 위치정보를 보냄 
          (node : mobilNet , publisher : 이미지 위치정보 (ID / Width /         
          Height / center_x / center_y))
    4) Lidar 의 입력을 받아서 거리정보를 보냄
        (node : sonar , publisher : Noise filtered -40도/ 0도/ 40도)
        
- Pi
    기능 : 
    1) 이미지 위치정보를 받음  자율주행 판단 연산
         ( node : auto_drive, subscriber : 
          이미지 위치정보 (ID / Width / Height / center_x / center_y))
    2)  Lidar 정보를 받음  자율주행 판단연산
          ( node : auto_drive, subscriber : Noise filtered -40도/ 0도/ 40도)
    3) 키보드의 입력을 받음  ex) 키보드 조정 운전 or 차량에 탑승 모드 설정,해제 
          ( node : teleop_keyboard , subscriber : 차량의 [accel, steer, 작동모드] 배열 )
    4) 이미지 위치정보 , 초음파센서 정보를 합하여 선속도 / 각속도 벡터형태로 보낸다.
           ( node : auto_drive, pusblisher : 차량의 [accel, steer, 작동모드] 배열 )
    5) 구동모터의 pwm 지령 및 encoder를 통한 피드백 제어 
           (node : md_motor_command , subscriber : 차량의 [accel, steer, 작동모드]에 
          자율주행 알고리즘 적용 Publisher : GPIO를 입력 / 출력을 통한 실제 모터 제어  )
    6) 탑승 모터 pwm 제어 
            (node : motor_lift_command , subscriber : 차량의 [직선, 선회,작동모드] 배열  
            , publisher : GPIO 출력을 통한 실제모터제어) 




