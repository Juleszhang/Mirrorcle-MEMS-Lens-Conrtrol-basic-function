import os
import time
import threading

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk
from functools import partial
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import MEMS_Control_Function
import dv_processing as dv
from collections import deque
from sklearn.mixture import GaussianMixture
from scipy.signal import find_peaks, savgol_filter
from filterpy.kalman import KalmanFilter
import matplotlib
matplotlib.use('Agg')

class Tracker:
    def __init__(self, x_bias, y_bias, x_diff, y_diff, parent=None):
        self.x_bias = x_bias
        self.y_bias = y_bias
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.y_value = 0
        self.running = False
        self.thread_scan = None
        self.thread_receive = None
        self.client = None
        self.plot_data = []
        self.delay_compensation = 0
        self.event_counts = []
        self.corrected_y_values = []
        self.figure, self.ax = plt.subplots()
        self.colors = ['b', 'g', 'r', 'c', 'm']

        if parent:
            self.canvas = FigureCanvasTkAgg(self.figure, master=parent)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)


    def connect_to_server(self):
        if self.client is None:
            print("Connecting to server at 192.168.100.1:12345...")
            self.client = dv.io.NetworkReader("192.168.100.10", 12345)

            if not self.client.isEventStreamAvailable():
                raise RuntimeError("Server does not provide event data!")
            else:
                print("Connected to server and event stream is available.")
        else:
            print("Already connected to the server.")


    def tracking(self):
        self.connect_to_server()
        # 初始化卡尔曼滤波器
        kf = KalmanFilter(dim_x=4, dim_z=2)
        dt = 1.0  # 预测间隔时间
        kf.F = np.array([[1, 0, dt, 0],
                        [0, 1, 0, dt],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        kf.H = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0]])
        kf.P *= 1000
        kf.R = np.array([[0.01, 0],
                        [0, 0.01]])
        kf.Q = np.array([[0.1, 0, 0, 0],
                        [0, 0.1, 0, 0],
                        [0, 0, 0.1, 0],
                        [0, 0, 0, 0.1]])

        #初始化变量
        #previous_centroid = None
        monitoring_printed = False
        tracking_initialized = False
        track_window = None
        event_hist = None

        #相机分辨率
        frame_width, frame_height = 346.0, 240.0
        center_x, center_y = frame_width / 2, frame_height / 2 #相机中心点

         # Step 2:初始化Camshift算法
        def initialize_camshift_from_events(events, centroid):
            x, y= centroid
            w, h = 50, 50 #初始化搜索窗口大小
            track_window = (int(x-w // 2), int(y-h // 2), w, h)


            # 将事件像素绘制到事件图像中
            event_image = np.zeros((frame_height, frame_width), dtype=np.uint8)
            for coord in events:
                ex, ey = int(coord[0], int(coord[1]))
                if 0 <= ex < frame_width and 0 <= ey < frame_height:
                    event_image[ey, ex] = 255 #设置事件点位白色

            # 使用事件图像构建概率密度图
            event_hist = cv2.calcBackProject([event_image], [0], None, [0, 256], scale=1) #反向显示事件分布
            cv2.normalize(event_hist, event_hist, 0, 255, cv2.NORM_MINMAX)
            return track_window, event_hist

        print("Tracking started.")
        event_hist = np.ones((256,), dtype=np.float32)

        while True:
            events = self.client.getNextEventBatch()
            if events is not None:
                current_event_count = events.size()
                print(f"Received {current_event_count} events.")
                if current_event_count > 20000:
                    print(f"High event count detected: {current_event_count}, please reduce light energy.")
                    #停止跟踪
                    break

                # 获取像素变化坐标
                coordinates = events.coordinates()

                #初始化跟踪，寻找光斑位置
                if not tracking_initialized:
                    track_window = initialize_camshift_from_events(coordinates)
                    if track_window is not None:
                        print(f"Tracking initialized with window: {track_window}")
                        tracking_initialized = True
                        continue
                        """
                        centroid = centroid_detection_from_events(cooridinates)
                        print(f"Initial centroid detected at: {centroid}")
                        # 调用MEMS_Control_Function将光斑移动到相机中心点
                        offset_x, offset_y = centroid[0] - center_x, centroid[1] - center_y
                        while abs(offset_x) > 5 or abs(offset_y) > 5:  # 偏移在5像素以内时停止调整
                            direction_x = "left" if offset_x > 0 else "right"
                            direction_y = "up" if offset_y > 0 else "down"

                            # 根据X轴和Y轴的偏移分别调整
                            X_diff = self.x_diff * (-offset_x / frame_width)  # 根据X轴方向的偏移调整
                            Y_diff = self.y_diff * (-offset_y / frame_height)  # 根据Y轴方向的偏移调整

                            # 调用MEMS控制函数进行两轴调整
                            MEMS_Control_Function.Rotation_Control_X(self.x_bias, X_diff, 55)  # 控制X轴调整
                            MEMS_Control_Function.Rotation_Control_Y(self.y_bias, Y_diff, 55)  # 控制Y轴调整
                        

                        print(f"Adjusting light spot: {direction_x}, {direction_y} | Offset: ({offset_x}, {offset_y})")
                        """
                #相机识别阶段
                if tracking_initialized:
                    #创建事件图像进行反向投影：
                    event_image = np.zeros((frame_width, frame_height), dtype=np.uint8)
                    for coord in coordinates:
                        ex, ey = int(coord[0]), int(coord[1])
                        if 0 <= ex < frame_width and 0 <= ey < frame_height:
                            event_image[ey, ex] = 255

                    cv2.imshow("Event Image", event_image)

                    dst = cv2.calcBackProject([event_image], [0], event_hist, [0, 256], 1)
                    ret, track_window = cv2.CamShift(dst, track_window,
                                                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))
                    x, y, w, h = track_window
                    measured_centroid = np.array([x + w // 2, y + h // 2])

                    #卡尔曼滤波器更新
                    kf.perdict()
                    kf.update(measured_centroid)
                    estimated_position = kf.x[:2]

                    #动态显示跟踪框和质心
                    display_image = event_image.copy()
                    cv2.rectangle(display_image, (x,y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.circle(display_image, (int(measured_centroid[0]), int(measured_centroid[1])),
                               5, (255, 255, 255), -1)
                    cv2.putText(display_image, f"Centroid: {measured_centroid}", (10, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.imshow("Tracking", display_image)

                    #动态跟踪
                    if previous_centroid is not None:
                        movement_vector = estimated_position - previous_centroid
                        max_movement = np.max(np.abs(movement_vector))
                        movement_vector_normalized = movement_vector / max_movement if max_movement != 0 else movement_vector
                        delta_x,delta_y = movement_vector_normalized
                        delta_x = -delta_x
                        delta_y = -delta_y
                        delta_x = np.clip(delta_x, -1, 1)
                        delta_y = np.clip(delta_y, -1, 1)
                        direction_x = "right" if delta_x > 0 else "left"
                        direction_y = "up" if delta_y > 0 else "down"

                        #X_diff = self.y_diff * delta_y
                        #Y_diff = self.y_diff * delta_y
                        #MEMS_Control_Function.Rotation_Control_Y(self.y_bias, Y_diff, 55)
                        #MEMS_Control_Function.Rotation_Control_Y(self.y_bias, Y_diff, 55)
                        print(f"Moving with direction_X: {direction_x}, direction_Y: {direction_y}")

                    previous_centroid = estimated_position

                else:
                    if not monitoring_printed:
                        print("Monitoring... No significant moving detected.")
                        monitoring_printed = True

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()

def main():
    Vbias = 90
    Vdiff = 25

    app = tk.Tk()
    app.title("Tracking Control Interface")

    main_frame = ttk.Frame(app)
    main_frame.pack(fill=tk.BOTH, expand=True)

    plot_frame = ttk.Frame(main_frame)
    plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    control_frame = ttk.Frame(main_frame)
    control_frame.pack(side=tk.BOTTOM, fill=tk.X)

    tracker = Tracker(Vbias, Vbias, Vdiff, Vdiff, plot_frame)

    clock_generator = MEMS_Control_Function.ClockSignalGenerator(pin1=22, pin2=27, frequency=30000)

    # MEMS START GUI
    ttk.Button(control_frame, text="Enable Driver", command=MEMS_Control_Function.Enable_Driver).pack(side=tk.LEFT, padx=10, pady=5)
    ttk.Button(control_frame, text="Disable Driver", command=MEMS_Control_Function.Disable_Driver).pack(side=tk.LEFT, padx=10, pady=5)
    ttk.Button(control_frame, text="Enable FCLK", command=clock_generator.start).pack(side=tk.LEFT, padx=10, pady=5)
    ttk.Button(control_frame, text="Disable FCLK", command=clock_generator.stop).pack(side=tk.LEFT, padx=10, pady=5)

    # Initialize MEMS GUI
    ttk.Button(control_frame, text="Initialize DAC", command=MEMS_Control_Function.initialize_dac).pack(side=tk.LEFT, padx=10, pady=5)
    ttk.Button(control_frame, text="Reset All Bias", command=partial(MEMS_Control_Function.Reset_AllBias, Vbias)).pack(side=tk.LEFT, padx=10, pady=5)

    ttk.Button(control_frame, text="Connect to server", command=tracker.connect_to_server).pack(side=tk.LEFT, padx=10, pady=20)
    ttk.Button(control_frame, text="Start tracking", command=tracker.tracking).pack(side=tk.LEFT, padx=10, pady=20)

    app.mainloop()


if __name__ == "__main__":
    main()
