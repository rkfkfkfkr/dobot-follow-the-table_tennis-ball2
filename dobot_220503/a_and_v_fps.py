import cv2
import numpy as np
import imutils
import matplotlib.pyplot as plt
import threading
import math
import time
import timeit


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

    of_x = (of_x+4) *10 + 28
    of_y = of_y *10 + 37
    
    l_index = dType.SetCPCmd(api,dType.ContinuousPathMode.CPAbsoluteMode, of_x, of_y, 50, 30, isQueued = 1)

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
    
cap = cv2.VideoCapture(0)

w = round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')

#out_frame = cv2.VideoWriter('moving_ball_calibration.mp4', fourcc, 20, (w, h))
#out_th = cv2.VideoWriter('moving_th.mp4', fourcc, 20, (w, h))

num = 0
count = 0
while(1):
    
    ret,frame = cap.read()

    #frame = np.zeros((480,640,3),np.uint8)

    #cv2.circle(frame, (num*3, num * 3), 10, (0,127,255),-1)

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
            px = 100 * Pw[0] #cm기준
            py = 100 * Pw[1]
            
            text = " %f , %f" %(px,py)
            cv2.putText(frame,text,center,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)

            #offset_x = (px+4) *10 + 28 #dobot 좌표계에 맞춰지는 좌표
            #offset_y = py *10 + 37

            offset_x = px / 100 # m기준
            offset_y = py / 100

            Ball_position.x.append(float(offset_x)) # m기준
            Ball_position.y.append(float(offset_y))

            #print(Ball_position.x[-1])
            #print(Ball_position.y[-1])

            terminate_t = timeit.default_timer()

            FPS = int(1./(terminate_t - start_t ))
            #print(FPS)
            #print("time: %f\n" %(terminate_t - start_t ))

            if len(Ball_position.x) > 2:

                time_step = terminate_t - start_t

                time_list.append(time_step)

                #print(len(Ball_position.x))
                #print(Ball_position.x[-2])
                #print(Ball_position.x[-1])

                start_x = round(Ball_position.x[-2],4)
                start_y = round(Ball_position.y[-2],4)

                end_x = round(Ball_position.x[-1],4)
                end_y = round(Ball_position.y[-1],4)

                #print("start(%f,%f) end(%f,%f)" %(start_x,start_y,end_x,end_y))

                caculate_V(start_x,start_y,end_x,end_y,time_step)
                #caculate_a(time_step)
                offset_x,offset_y = create_path(start_x,start_y,end_x,end_y)
                length = math.sqrt(math.pow(offset_x,2) + math.pow(offset_y,2))
                print("offset_x: %f, offset_y: %f, length: %f \n" %(offset_x,offset_y,length))

                
                


    cv2.imshow('cam',frame)
    num += 1
    if cv2.waitKey(1) == 27:
        break

cap.release()
#out_frame.release()
#out_th.release()
cv2.destroyAllWindows()
