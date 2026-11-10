#计算位姿
import cv2
import numpy as np

W=1280
H=720
MARKER_SIZE=0.06#用尺子量出aruo码黑框边缘的边长
MARKER_ID=0
data = np.load(r"输入绝对路径")
#numpy引入内存数据
K,dist=data['K'],data['dist']#内参和畸变
half=MARKER_SIZE/2.0
#强制顺序：左上，右上，右下，左下,half为坐标值建坐标
OBJ=np.array([[-half,half,0.0],
             [half,half,0.0],
             [half,-half,0.0],
             [-half,-half,0.0]],np.float32)

#初始化检测器
dict=(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
params=cv2.aruco.DetectorParameters()
params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
detector=cv2.aruco.ArucoDetector(dict,params)

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)#设置分辨率
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
if not cap.isOpened():
    print("摄像头打不开")
    exit()
while True:
    ok,frame=cap.read()
    if not ok:
        continue
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #高斯滤波降噪
    gray=cv2.GaussianBlur(gray,(3,3),0)
    #corners:4个角点，ids:每个aruco码的框的编号
    corners,ids,_=detector.detectMarkers(gray)
    if ids is not None:
        #sel:id=0 的码在列表里的位置
        sel=np.where(ids.ravel()==MARKER_ID)[0]
        if sel.size:
            #去除[1,4,2]的1维标签维，得到4角点坐标列表
            img_pts=corners[sel[0]].reshape(4,2).astype(np.float32)

            #正方形位姿有两组解,（右移左移拍出照片几乎一致）
            #Generic:返回全部解
            """
            solvePnPGeneric()核心2d转3d
            输入：现实坐标(OBJ),图片坐标(img_pts),相机标定参数(k,dist)
            输出：bool是否成功(ok2),
                 旋转向量列表(rvecs),
                 平移向量列表(tvecs),
                 每个解的误差(errs)
            """
            ok2,rvecs,tvecs,errs=cv2.solvePnPGeneric(
                OBJ,img_pts,K,dist
            )

            #输出最可信解
            if ok2:
                #从含有两误差的列表，选取最小的值的组，作为解
                i=int(np.argmin(np.asarray(errs).ravel()))#ravel多维压为一维
                #保证取出的解是（3，1）维度，f64提高精度
                rvec=np.asarray(rvecs[i],np.float64).reshape(3,1)
                tvec=np.asarray(tvecs[i],np.float64).reshape(3,1)

                #画绿框，和3d坐标维度图，和距离显示
                cv2.polylines(frame, [img_pts.astype(np.int32)], True, (0, 255, 0), 2)#画任意多边形
                cv2.drawFrameAxes(frame, K, dist, rvec, tvec, MARKER_SIZE * 0.5, 2)
                #z值转厘米，保留一位小数;x=20，距顶20；y=40，距左40；什么字体；字大小；颜色；笔粗细
                cv2.putText(frame, f"Z={tvec[2][0] * 100:.1f}cm",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imshow('step4',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()