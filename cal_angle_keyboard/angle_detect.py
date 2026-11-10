import cv2
import numpy as np
import time


W,H=1280,720
MARKER_SIZE=0.06#需要实际测量aruco黑框边缘
MARKER_ID=0
DEAD=4#转角死区，<不动
SAT=90#饱和度，最大转角
DEAD_EXIT= 6.0 # 死区滞回：进 4°，出 6°
SIGN=1.0#反向为-1.0
LOST_HOLD=0.20#丢码后保持输出时间
ERR_MAX=8#重投影误差上限，超过视为无效帧

OE_MINCUT, OE_BETA, OE_DCUT = 2.5, 0.08, 1.0   # One Euro 参数
CALIB_N, CALIB_SPREAD = 5, 2.0                 # 校准帧数 / 允许抖动（度）

"""
刚体运动学的标准结论:
    两个旋转矩阵 Ra 和 Rb，它们之间的转角 θ 满足：
    cos θ = (trace(Raᵀ · Rb) - 1) / 2
    trace = 矩阵的迹 = 对角线元素之和
    clip(c,-1,1)夹紧到 [-1, 1]
    arccos()反余弦 → 弧度,arccos(1) = 0，arccos(0) = 1.5708(π/2)，arccos(-1) = 3.1416(π)
    degrees()弧度 → 度
"""
#两旋转矩阵的转角
def rot_dist(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))

#scd合并压缩图片
def rot_mean(Rs):
    #对于和矩阵，相当于只通过U,Vt关注旋转角度，不关注伸缩大小
    #u,vt要保证3向量互相垂直，每列长度为1(3元素平方和)，所以正交
    U, _, Vt = np.linalg.svd(sum(Rs))
    R = U @ Vt
    #det:矩阵把空间缩放的倍数(3维求主俯听面积),大小，倍数；符号，方向
    #排除镜像
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


#角度提取q
def angle_deg(R_curr,R_base):
    u=R_base[:,0]
    v=R_base[:,1]
    u1=R_curr[:,0]
    v1=R_curr[:,1]
    #u1=cos·u+sin·v;v1=-sin·u+cos·v
    #su=从u轴计算出的sin,sv=从v轴计算出的sin,cu,cv同理cos
    su,cu=u1@v,u1@u
    sv,cv=-(v1@u),v1@v
    #PnP给的R有噪声时θ₁=θ+ε₁,θ₂=θ+ε₂,合成结果≈θ+(ε₁+ε₂)/2
    return float(np.degrees(np.arctan2(su+sv,cu+cv)))

#one euro滤波,角度变化越快，当前角度占比越大,使抖动更平滑
#x:原始角度，st:夸针记忆，dt:一帧的时间，
def one_euro(x, st, dt, mincut=OE_MINCUT, beta=OE_BETA, dcut=OE_DCUT):
    def alpha(cut):
        T=dt
        t=1.0/(2*np.pi*cut)
        a=1.0/(1.0+t/T)
        return a
    #角速度
    if st['xp'] is not None:
        dx=(x-st['xp'])/dt
    else:
        dx=0.0
    st['dx']=alpha(dcut)*dx+(1-alpha(dcut))*st['dx']
    cut=mincut+beta*abs(st['dx'])
    st['x']=alpha(cut)*x+(1-alpha(cut))*st['x']
    st['xp']=x
    return st['x']



#输出摇杆值
#死区滞回＋饱和，<4开死区，>6关死区，若果>6后在6上下抖动也是关死区
def to_stick(a, st):
    aa = abs(a)#角度
    if st['dead']:
        if aa > DEAD_EXIT:
            st['dead'] = False
    else:
        if aa < DEAD:
            st['dead'] = True
    if st['dead']:
        return 0.0
    s = 1.0 if a > 0 else -1.0
    #线性映射，占空比 ：一个周期（比如 100ms）里，按键时间占比 = 摇杆值
    return s * min(1.0, (aa - DEAD) / (SAT - DEAD))


data = np.load(r"输入绝对路径")
K=data['K'].astype(np.float64)
dist=data['dist'].astype(np.float64)
print("已加载标定")
half=MARKER_SIZE/2.0
#强制顺序：左上，右上，右下，左下,half为坐标值建现实坐标
OBJ=np.array([[-half,half,0.0],
             [half,half,0.0],
             [half,-half,0.0],
             [-half,-half,0.0]],np.float32)

#初始化检测器
dict=(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
params=cv2.aruco.DetectorParameters()
params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
detector=cv2.aruco.ArucoDetector(dict,params)


def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)  # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        print("摄像头打不开")
        exit()
    aw=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"分辨率{aw}✖{ah}")
    if aw != W or ah != H:
        print('与标定分辨率不一致')
    R_base=None#回正基准矩阵
    last_R=None#上一帧的有效姿态
    #x:滤波角度，xp:上帧角度，dx:角度变化速度，dead:死区开关
    st = {'x': 0.0, 'xp': None, 'dx': 0.0, 'dead': True}
    angle_s = 0.0#平滑后角度
    stick = 0.0#最终摇杆值
    lost_t = 0.0#目标丢失时长
    calib_req, buf = False, []#标定请求标志，标定缓冲列表
    err_now = 0.0#当前重投影误差
    z_cm = 0.0#相机到方向盘的距离
    fps = 0.0#实测帧率
    key = 255#本帧按下的键
    t_prev = time.perf_counter()#上一帧的时刻
    print('按 C 校准回正基准，R 清除基准，Q/ESC 退出')
    while True:
        ok,frame=cap.read()
        if not ok:
            continue

        #帧率时间计算
        now =time.perf_counter()#读时间
        #dt:每帧多少秒，帧率界限120~10，计算2帧之间花费的时间
        dt=float(np.clip(now - t_prev, 1.0/120,0.1))
        t_prev = now
        #实时帧率=90%过去+10%现在
        fps=0.9*fps+0.1*(1.0/dt)

        #进行检测
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (3, 3), 0)
        corners, ids, _ = detector.detectMarkers(gray)

        #这帧的R_curr是否可信的
        valid=False
        R_curr=None

        """PnP + 双解 + 误差门槛"""
        if ids is not None:
            # sel:id=0 的码在列表里的位置
            sel = np.where(ids.ravel() == MARKER_ID)[0]
            if sel.size:
                # 去除[1,4,2]的1维标签维，得到4角点坐标列表
                img_pts = corners[sel[0]].reshape(4, 2).astype(np.float32)
                """
                bool是否成功(ok2),
                旋转向量列表(rvecs),
                平移向量列表(tvecs),
                每个解的误差(errs)
                """
                ok2, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                    OBJ, img_pts, K, dist,flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if ok2 and len(rvecs)>0:
                    #多维压一维：[[1],[2]]->[1,2]
                    errs=np.asarray(errs).ravel()
                    #最小误差的标签
                    i=int(np.argmin(errs))
                    if errs[i] <=ERR_MAX:
                        #存在多个解，且有上一帧姿态
                        if len(rvecs)>1 and last_R is not None:

                            #通过罗德里格斯求出旋转向量的旋转矩阵，再求两旋转矩阵的旋转角
                            #通过旋转矩阵的前后角度变化体现连贯性误差
                            ds=[]
                            for r in rvecs:
                                r=np.asarray(r,np.float64)
                                ds.append(rot_dist(cv2.Rodrigues(r)[0],last_R))
                            j=int(np.argmin(ds))

                            #?
                            #当误差能说明问题时信误差，当误差说明不了问题时信连贯性。
                            if errs[j]<=errs[i]*1.5+5.0:#可改进点：误差和连贯性的选择关系
                                i=j
                        #Rodrigues结果 = ( R矩阵 , 雅可比矩阵 )
                        R_curr=cv2.Rodrigues(np.asarray(rvecs[i],np.float64))[0]
                        #得到最优平移向量
                        tv=np.asarray(tvecs[i]).ravel()
                        last_R=R_curr
                        err_now=float(errs[i])
                        z_cm=float(tv[2]*100)
                        valid=True

                        # 保证取出的解是（3，1）维度，f64提高精度
                        rvec = np.asarray(rvecs[i], np.float64).reshape(3, 1)
                        tvec = np.asarray(tvecs[i], np.float64).reshape(3, 1)

                        # 画绿框，和3d坐标维度图，和距离显示
                        cv2.polylines(frame, [img_pts.astype(np.int32)], True, (0, 255, 0), 2)  # 画任意多边形
                        cv2.drawFrameAxes(frame, K, dist, rvec, tvec, MARKER_SIZE * 0.5, 2)

        """可靠校准，校准反向盘的最初姿态"""
        #按c回正基准，这一帧可信
        if calib_req and valid:
            buf.append(R_curr)
            #多帧图片整合,降低抖动影响
            if len(buf)>=CALIB_N:
                spread = max(rot_dist(buf[a], buf[b])
                             for a in range(len(buf))
                             for b in range(a + 1, len(buf)))
                if spread<CALIB_SPREAD:
                    R_base=rot_mean(buf)
                    st['x'], st['xp'], st['dx'] = 0.0, None, 0.0  # 重置滤波
                    st['dead'] = True  # 重置死区
                    angle_s, stick = 0.0, 0.0
                    print("校准完成，基准已设 0°")
                else:
                    print(f"姿态不稳(抖动{spread:.1f}°)，校准作废，请保持不动再按 C")
                calib_req = False

        """丢码计时"""
        if not valid:
            #累积丢帧时间
            lost_t+=dt#dt=每帧的时间
            if lost_t>LOST_HOLD:#超过保留时间就回正
                st['x'], st['xp'], st['dx'] = 0.0, None, 0.0
                angle_s, stick = 0.0, 0.0
        else:
            lost_t=0.0

        """角度->滤波->摇杆"""
        if valid and R_base is not None:
            #raw:角度+方向
            raw=SIGN*angle_deg(R_curr,R_base)
            #滤波后平滑角度
            angle_s=one_euro(raw,st,dt)
            stick=to_stick(angle_s, st)

        """显示画面叠加"""
        #当前角度
        cv2.putText(frame, f'Angel {angle_s:+6.1f} deg', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        #摇杆输出
        cv2.putText(frame, f'Stick {stick:+.2f}', (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
        #z轴距离，损失值，实时帧率
        cv2.putText(frame, f'Z {z_cm:.1f}cm  Err {err_now:.2f}px  {fps:.0f}fps', (20, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        #如果未做初始标定，红字提醒按c
        if R_base is None:
            cv2.putText(frame, 'Pess c to Calibrate ', (20, 145),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        #帧画面丢失
        if (not valid) and lost_t > LOST_HOLD:
            cv2.putText(frame, 'picture loss', (20, 175),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)


        cv2.imshow("Vision Wheel", frame)
        key=cv2.waitKey(1) & 0xFF
        if key==ord('c'):
            if valid:
                calib_req=True
                buf=[]
                print("开始校准")
            else:
                print("本帧未检测到有效帧图片，校准失败")
        elif key==ord('r'):
            R_base=None
            st['x'], st['xp'], st['dx'] = 0.0, None, 0.0
            st['dead'] = True
            angle_s, stick = 0.0, 0.0
            print('基准已清除')
        elif key==ord('q')or key==27:
                break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
