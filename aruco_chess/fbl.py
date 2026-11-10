#查分辨率
import cv2
cap = cv2.VideoCapture(0)
# 打印默认/当前分辨率
print("当前宽:", int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
print("当前高:", int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
# 依次尝试常见分辨率，看摄像头实际能开到多大
resolutions = [(640, 480), (1280, 720), (1920, 1080), (2560, 1440)]
print("\n尝试各分辨率（若不支持会被强制降级）：")
for w, h in resolutions:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  请求 {w}×{h}  →  实际 {real_w}×{real_h}")

cap.release()
cv2.destroyAllWindows()