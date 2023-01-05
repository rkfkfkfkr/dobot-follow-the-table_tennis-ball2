import cv2
import numpy as np
import imutils
import matplotlib.pyplot as plt
import threading
import DobotDllType as dType
import math
import time
import timeit


cap = cv2.VideoCapture(0)

w = round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')

out = cv2.VideoWriter('following_ball_220404.mp4', fourcc, fps, (w, h))

objectPoints = np.array([[0,0,0],
                        [0,-0.325,0],
                        [0,0.325,0],
                        [0.449,0.325,0],
                        [0.449,-0.325,0],
                        [0.449,0,0]],dtype = 'float32')
imagePoints = np.array([[320,243],
                       [139,243],
                       [478,243],
                       [602,478],
                       [36,478],
                       [320,478]],dtype = 'float32')

fx = float(470.5961)
fy = float(418.18176)
cx = float(275.7626)
cy = float(240.41246)
k1 = float(0.06950)
k2 = float(-0.07445)
p1 = float(-0.01089)
p2 = float(-0.01516)

cameraMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype = 'float64')
distCoeffs = np.array([k1,k2,p1,p2],dtype = 'float64')
_,rvec,t = cv2.solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs)
R,_ = cv2.Rodrigues(rvec)

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound", 
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#dll을 메모리에 읽어들여 CDLL 인스턴스 가져오기
#Load Dll and get the CDLL object
api = dType.load()

#dobot과의 연결을 설정
#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

def get_distance(R, t, fx, fy, cx, cy, x, y):
    
    u = (x - cx) / fx
    v = (y - cy) / fy
    Qc = np.array([[u],[v],[1]])
    Cc = np.zeros((3,1))
    Rt = np.transpose(R)
    Qw = Rt.dot((Qc-t))
    Cw = Rt.dot((Cc-t))
    V = Qw - Cw
    k = -Cw[-1,0]/V[-1,0]
    Pw = Cw + k*V
    
    return Pw

def cpmove(of_x,of_y):
    
    l_index = dType.SetCPCmd(api,dType.ContinuousPathMode.CPAbsoluteMode, of_x, of_y, 10, 30, isQueued = 1)

    dType.dSleep(100)

    return l_index

class Points:
    x = []
    y = []
    value = []


Ball_position = Points()
Velocity = Points()
acceleration = Points()

Velocity.x.append(0)
Velocity.y.append(0)
Velocity.value.append(0)

p_change = Points() # 변위
p_change.x.append(0)
p_change.y.append(0)
p_change.value.append(0)

p_x = []
p_y = []

p_x.append(0)
p_y.append(0)

V_x = []
V_y = []

V_x.append(0)
V_y.append(0)

a_x = []
a_y = []

time_list = []

def caculate_V(start_x,start_y,end_x,end_y,time_step): # V_time(시변)

    print("start(%f,%f) end(%f,%f)" %(start_x,start_y,end_x,end_y))

    position_change = math.sqrt(math.pow(end_x - start_x,2) + math.pow(end_y - start_y,2)) # 변위

    px = end_x - start_x
    py = end_y - start_y

    p_x.append(px)
    p_y.append(py)
    p_change.value.append(position_change)

    print("변위: %f, px: %f, py: %f" %(position_change,px,py))
    print("시간: %f" %time_step)

    V = position_change / time_step
    Vx = (end_x - start_x)/time_step
    Vy = (end_y - start_y)/time_step

    V_x.append(Vx)
    V_y.append(Vy)

    Velocity.value.append(V)

    print("V: %f cm/s, V_x: %f, V_y: %f" %(V,Vx,Vy))
    

def caculate_a(time_step):

    v1 = Velocity.value[-1] #최종속력
    v0 = Velocity.value[-2] #초기속력

    #acceleration

    a = (v1 - v0)/time_step # 가속도 m/s^2

    ax = (V_x[-1] - V_x[-2])/time_step
    ay = (V_y[-1] - V_y[-2])/time_step

    a_x.append(ax)
    a_y.append(ay)
    
    acceleration.value.append(a)

    #print('a: %f cm/s^2, ax: %f, ay: %f\n' %(a,ax,ay))

def create_path(start_x,start_y,end_x,end_y):

    Navigation_direction = 0

    D = p_change.value[-1]

    y = p_y[-1]

    theta = 0
    
    if D != 0:
        theta = math.asin(abs(y)/D) * 180/math.pi
        
    if (start_x - end_x > 0) and (start_y - end_y < 0):
        theta = 180 - theta
    elif (start_x - end_x >= 0) and (start_y - end_y >= 0):
        theta = 180 + theta
    elif (start_x - end_x < 0) and (start_y - end_y > 0):
        theta = 360 - theta

    time_av = np.mean(time_list)

    V_av = sum(Velocity.value)/(len(Velocity.value) - 1)

    predict_D = time_av * V_av
    predict_Dx = predict_D * math.sin(theta * math.pi/180) + end_x
    predict_Dy = predict_D * math.sin(theta * math.pi/180) + end_y

    print("theta: %f, PD: %f, PDx: %f, PDy: %f \n" %(theta, predict_D,predict_Dx,predict_Dy))

    px = predict_Dx * 100
    py = predict_Dy * 100

    offset_x = (px+4) *10 + 28
    offset_y = py *10 + 37

    return offset_x,offset_y

if (state == dType.DobotConnect.DobotConnect_NoError):
    
    #Clean Command Queued
    dType.SetQueuedCmdClear(api)
    
    #Async Motion Params Setting
    dType.SetHOMEParams(api, 200, 0, 10, 100, isQueued = 1) # x, y, z, r 
    dType.SetCPParams(api, 50, 200, 200, True, isQueued = 1 ) # SetCPParams(api, planAcc, juncitionVel, acc, realTimeTrack = 0,  isQueued=0)
    dType.SetCPCommonParams(api, 50,50, isQueued = 1)
    
    #Async Home
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)

    count = 0

    dType.SetQueuedCmdStartExec(api)

    while(1):
        ret,frame = cap.read()

        start_t = timeit.default_timer()

        img_ycrcb = cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
        y,cr,cb = cv2.split(img_ycrcb)

        _, cb_th = cv2.threshold(cb, 100, 255, cv2.THRESH_BINARY_INV)
        cb_th = cv2.erode(cb_th, None, iterations=2)
        cb_th = cv2.dilate(cb_th, None, iterations=2)

        cnts = cv2.findContours(cb_th, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 5:
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                Pw = get_distance(R,t,fx,fy,cx,cy, center[0], center[1])
                px = 100 * Pw[0]
                py = 100 * Pw[1]
                text = " %f , %f" %(px,py)
                cv2.putText(frame,text,center,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                
                offset_x = px
                offset_y = py

                Ball_position.x.append(float(offset_x/100))
                Ball_position.y.append(float(offset_y/100))

                terminate_t = timeit.default_timer()

                FPS = int(1./(terminate_t - start_t ))

                if len(Ball_position.x) > 2:

                    time_step = terminate_t - start_t

                    time_list.append(time_step)

                    start_x = round(Ball_position.x[-2],4)
                    start_y = round(Ball_position.y[-2],4)

                    end_x = round(Ball_position.x[-1],4)
                    end_y = round(Ball_position.y[-1],4)

                    caculate_V(start_x,start_y,end_x,end_y,time_step)
                    caculate_a(time_step)
                    offset_x,offset_y = create_path(start_x,start_y,end_x,end_y)

                    length = math.sqrt(math.pow(offset_x,2) + math.pow(offset_y,2))
                    #print("offset_x: %f, offset_y: %f, length: %f \n" %(offset_x,offset_y,length))

                    if length < 260:
                        print("offset_x: %f, offset_y: %f, length: %f \n" %(offset_x,offset_y,length))
                
                        if count == 0:
                            current_ofx = px
                            current_ofy = py

                            last_index = cpmove(offset_x,offset_y)

                        else:
                            x_move = abs(current_ofx - px)
                            y_move = abs(current_ofy - py)

                            if last_index == dType.GetQueuedCmdCurrentIndex(api)[0]:
                                    dType.SetQueuedCmdStartExec(api)
                                    pirnt("a")

                            if 2 < x_move or 2 < y_move:

                                    last_index = cpmove(offset_x,offset_y)
                                    
                        count += 1
    
        cv2.imshow('cam',frame)
        out.write(frame)

        if cv2.waitKey(1) == 27:
            break

    dType.SetQueuedCmdStopExec(api)
    

dType.DisconnectDobot(api)
cap.release()
out.release()
cv2.destroyAllWindows()
