import cv2
import numpy as np

def detect_red_balls(img_path):
    # 读取图像
    img = cv2.imread(img_path)
    print("[📊] 图像已加载，尺寸：", img.shape)  # 新增：输出原始图像尺寸
    
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imwrite('1_hsv_image.jpg', hsv)  # 新增：保存HSV图像
    print("[🛠️ ] HSV图像已保存为1_hsv_image.jpg")

    # 定义红色范围（HSV空间）
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # 创建掩膜
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    cv2.imwrite('2_combined_mask.jpg', mask)  # 新增：保存合并后的掩膜
    print(f"[🛠️ ] 红色掩膜已生成，有效像素数：{cv2.countNonZero(mask)}")

    # 形态学操作
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)
    cv2.imwrite('3_morph_cleaned.jpg', cleaned)  # 新增：保存形态学处理结果
    print(f"[🛠️ ] 形态学处理完成，剩余有效像素：{cv2.countNonZero(cleaned)}")

    # 查找轮廓
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"[📊] 初步检测到候选轮廓数量：{len(contours)}")  # 新增：候选轮廓统计
    
    # 存储结果坐标
    coordinates = []
    valid_contours = 0  # 新增：有效轮廓计数器
    
    for i, cnt in enumerate(contours, 1):
        # 面积筛选
        area = cv2.contourArea(cnt)
        print(f"\n[🔲 {i}] 原始面积：{area:.1f}")  # 新增：轮廓信息
        
        if area < 100:
            print(f"[过滤] 面积不足，已跳过（{area:.1f} < 100）")
            continue
            
        # 圆形度检测
        (x,y), radius = cv2.minEnclosingCircle(cnt)
        circle_area = np.pi * (radius**2)
        circularity = area / circle_area
        print(f"[🔲 {i}] 圆形度：{circularity:.2f}，半径：{radius:.1f}px")
        
        if circularity < 0.4:
            print(f"[过滤] 圆形度不足，已跳过（{circularity:.2f} < 0.4）")
            continue
            
        # 获取边界框
        x,y,w,h = cv2.boundingRect(cnt)
        
        # 存储坐标
        center = (x + w//2, y + h//2)
        coordinates.append(center)
        valid_contours += 1
        print(f"[ 🎯 ] 有效目标 {valid_contours}，中心坐标：{center}")

        # 绘制结果
        cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
        cv2.putText(img, f"({center[0]}, {center[1]})", 
                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
    
    # 显示结果
    cv2.imwrite('4_final_result.jpg', img)  # 新增：明确标注结果文件名
    print(f"\n[📊] 处理完成，共检测到 {valid_contours} 个红色小球")
    print("[ 🛠️ ] 结果图像已保存为4_final_result.jpg")
    
    return coordinates

# 使用示例
if __name__ == "__main__":
    coords = detect_red_balls("red_balls.png")
    print("\nDetected Coordinates:")
    for i, (x,y) in enumerate(coords, 1):
        print(f"Ball {i}: X={x}, Y={y}")