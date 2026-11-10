#相机标定
import glob
import os
import sys
import numpy as np
import cv2
PATTERN=(9,6)
SQUARE_M=0.018  #实际测量黑格子边长(要改)
W,H=1280,720
IMGDIR="calib_img"
#读图
def capture():
    os.makedirs(IMGDIR, exist_ok=True)#创建保存文件夹
    cap=cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,W)#分辨率宽w
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)#分辨率高h
    if not cap.isOpened():
        print("摄像头打开失败")
        return

    #glob.glob("calib_imgs/*.jpg")
    #返回一个列表：
    #['calib_imgs/cb_001.jpg', 'calib_imgs/cb_000.jpg', 'calib_imgs/cb_002.jpg']
    n=len(glob.glob(f"{IMGDIR}/*.jpg"))#多少张jpg照片
    print("空格=存图 q=退出")
    while True:
        ok,f=cap.read()
        if not ok:
            continue
        cv2.imshow("capture",f)
        k=cv2.waitKey(1) & 0xFF
        if k==32:#空格的asllc码为32
            cv2.imwrite(f"{IMGDIR}/cb_{n:03d}.jpg",f)
            n+=1
        elif k==ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

#计算机内参
def calibrate():
    path=sorted(glob.glob(f"{IMGDIR}/*.jpg"))

    #建立数组(9*6,3),每行(x,y,z)三维坐标->棋盘格角点在现实的位置
    objp=np.zeros((PATTERN[0]*PATTERN[1],3),np.float32)

    #把2×9×6的三维结构,变成我们需要的54×2的二维表（每行一个点）
    #生成2个9*6网络，x,y，坐标的复制；转置拉平为54*2，每行（x,y）；索引成真实边长，变为真实物理坐标
    objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_M

    #op:每张图的3D世界坐标;ip:每张图的2D图像坐标
    op,ip,shape=[],[],None
    for p in path:
        g=cv2.cvtColor(cv2.imread(p),cv2.COLOR_BGR2GRAY)#转灰度图

        #找内角点，NORMALIZE_IMAGE：把亮度拉到统一范围，ADAPTIVE_THRESH：局部自适应二值化(局部阈值，抗光照不均)
        ok, c = cv2.findChessboardCorners(
            g, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print('跳过',p)
            continue

        #亚像素修复,使用11*11窗口，迭代30次至亚像素要0.001
        c=cv2.cornerSubPix(g,c,(11,11),(-1,-1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        #成对存3d,2d坐标
        op.append(objp)
        ip.append(c)
        shape=g.shape[::-1]#opencv和numpy的宽高是相反的
    print("有效图数量：",len(op),"/",len(path))
    if len(op) < 10:
        print("有效图不足，重采：别糊、别反光、让棋盘占满画面")
        return
    #rms:误差，K:内参，dist:畸变，revecs:旋转，tevecs:平移
    rms,K,dist,_,_=cv2.calibrateCamera(op,ip,shape,None,None)
    print("RMS =", round(rms, 4), " (必须 < 0.5)")
    print("K =\n", K)
    print("dist =", dist.ravel())
    np.savez("camera.npz", K=K, dist=dist)
    print("已保存 camera.npz")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        mode = sys.argv[1]                      # 命令行指定了就用指定的
    else:
        exist = len(glob.glob(f"{IMGDIR}/*.jpg"))
        mode = "cap" if exist < 10 else "calibrate"   # 照片不够→拍照, 够了→标定
        print(f"未指定模式, 自动选择: {mode} (当前已有 {exist} 张)")

    if mode == "cap":
        capture()
    elif mode == "calibrate":
        calibrate()
    else:
        print(f"未知参数 {mode}, 用法: cap | calibrate")