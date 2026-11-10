#生成ArUco和棋盘格
import cv2
import numpy as np
#ArUco
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)#预定义字典中选一套ArUco码——4×4的内部黑方块+50个不同编号
marker=cv2.aruco.generateImageMarker(d,0,600,1)
marker=cv2.copyMakeBorder(marker,75,75,75,75,cv2.BORDER_CONSTANT,value=255)
cv2.imwrite("aruco_id0..png",marker)
print("aruco_id0.png",marker.shape)

#棋盘格9*6内角点
#cols:棋盘格方格列数,rows:棋盘格方格行数,cell:每个方格200×200像素,pad:棋盘格四周留120像素白边
cols,rows,cell,pad=10,7,200,120
w,h=cols*cell+2*pad,rows*cell+2*pad#计算整张图的宽高
board=np.full((h,w),255,dtype=np.uint8)#创建一张全白的h×w单通道图
for r in range(rows):
    for c in range(cols):
        if(r+c)%2==0:
            board[pad+r*cell:pad+(r+1)*cell,pad+c*cell:pad+(c+1)*cell]=0#行列切片
cv2.imwrite("chessboard.png", board)
print("chessboard.png", board.shape)