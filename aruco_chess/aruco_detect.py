#检测AruCO,画框
import cv2
import numpy as np

W,H=1280,720
dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)#与原码保持一致
params=cv2.aruco.DetectorParameters()#默认参数表

#aruco亚像素修复(4几何角点)
#CORNER_REFINE_NONE:不精修，只用整数像素
#CORNER_REFINE_SUBPIX:梯度法亚像素精化（cornerSubPix同类算法）
#CORNER_REFINE_CONTOUR:沿轮廓拟合直线求交点
#CORNER_REFINE_APRILTAG:AprilTag的三线交点法
params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX

#组装可复用检测对象,第3参数:refineParams,不传等价于RefineParams()(mark黑白盒的bit格)
detector=cv2.aruco.ArucoDetector(dict,params)

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)#设置分辨率宽
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)#设置分辨率高

while True:
    ok,frame=cap.read()
    if not ok:
        continue
    #Aruco,棋盘格本质是黑白图对比
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #高斯滤波降噪，sigmX=0:拒绝模糊,防止信息丢失，越大越模糊，越容易丢失信息
    gray=cv2.GaussianBlur(gray,(3,3),0)

    #核心检测调用:喂灰度图得到
    #corners:marker(Aruco码图)4个角点,维度（1,4,2)
    #ids:每个marker的编号
    corners,ids,_=detector.detectMarkers(gray)

    if ids is not None:#判断有无码
        #循环：存在多个Aruco码，enumerate:自动计数器
        #i为第几组corners(corners[i]),mid为序号
        for i,mid in enumerate(ids.ravel()):
            pts=corners[i].reshape(4,2)
            #画绿色框
            cv2.polylines(frame,[pts.astype(np.int32)],True,(0,255,0),2)
            #在方框左上角上方，写一行绿色字 "id=0"
            cv2.putText(frame, f"id={mid}", (int(pts[0][0]), int(pts[0][1]) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            print("id", mid, "corners", pts.astype(int).tolist())
    cv2.imshow("frame",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
