#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rclpy
import time
import cv2
import numpy as np
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from queue import Queue
from rknnlite.api import RKNNLite
from geometry_msgs.msg import Point
from concurrent.futures import ThreadPoolExecutor
from ball_interfaces.msg import BallInfo  # 根据实际接口修改

class RKNNProcessor:
    """RKNN模型处理器，管理多线程推理"""
    
    def __init__(self, rknnModel, TPEs, func):
        """
        初始化RKNN处理器
        :param rknnModel: RKNN模型路径
        :param TPEs: 线程池大小
        :param func: 推理函数
        """
        self.TPEs = TPEs
        self.queue = Queue()
        self.rknnPool = self.initRKNNs(rknnModel, TPEs)
        self.pool = ThreadPoolExecutor(max_workers=TPEs)
        self.func = func
        self.num = 0

    def initRKNN(self, rknnModel, id=None):
        """初始化单个RKNN实例"""
        print(f"DEBUG: 正在尝试加载模型，文件路径为: {rknnModel}")
        rknn_lite = RKNNLite()
        ret = rknn_lite.load_rknn(rknnModel)
        if ret != 0:
            print("Load RKNN rknnModel failed")
            exit(ret)
        if id == 0:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        elif id == 1:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_1)
        elif id == 2:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_2)
        elif id == None:
            ret = rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        else:
            ret = rknn_lite.init_runtime()
        if ret != 0:
            print("Init runtime environment failed")
            exit(ret)
        print(rknnModel, "\t\tdone")
        return rknn_lite

    def initRKNNs(self, rknnModel, TPEs=1):
        """初始化RKNN实例池"""
        rknn_list = []
        for i in range(TPEs):
            rknn_list.append(self.initRKNN(rknnModel, i % 3))
        return rknn_list


    def put(self, frame):
        """添加处理任务到队列"""
        self.queue.put(self.pool.submit(
            self.func, self.rknnPool[self.num % self.TPEs], frame))
        self.num += 1

    def get(self):
        """获取处理结果"""
        if self.queue.empty():
            return None, False
        fut = self.queue.get()
        return fut.result(), True

    def release(self):
        """释放资源"""
        self.pool.shutdown()
        for rknn_lite in self.rknnPool:
            rknn_lite.release()

class YoloV8Detector(Node):
    """YOLOv8目标检测ROS2节点"""
    
    def __init__(self):
        super().__init__('detect_node')
        
        # 初始化参数
        self._declare_parameters()
        self._load_parameters()
        
        # 初始化工具
        self.bridge = CvBridge()
        self.classes = ("blue", "black", "red", "yellow", "redsafe", "bluesafe")

        self.declare_parameter('team_color', 'red')  # 从启动参数获取队伍颜色
        self.team_color = self.get_parameter('team_color').value
        #根据队伍颜色确定安全区颜色
        if self.team_color =='red':
            self.safe_color = 'redsafe'
        else:
            self.safe_color = 'bluesafe'
        self.get_logger().info(f"队伍颜色: {self.team_color}")
        self.get_logger().info(f"安全区颜色: {self.safe_color}")
        
        # 初始化摄像头或订阅
        self._initialize_camera()

        self.TPEs = 3
        # 初始化RKNN处理器
        self.rknn_processor = RKNNProcessor(
            rknnModel=self.model_path,
            TPEs=self.TPEs,
            func=self._process_frame
        )
        # 初始化异步所需要的帧
        if (self.cap.isOpened()):
            for i in range(self.TPEs + 1):
                ret, frame = self.cap.read()
                if not ret:
                    self.cap.release()
                    del self.rknn_processor
                    exit(-1)
                self.rknn_processor.put(frame)
        
        # 初始化发布器
        self.visualization_pub = self.create_publisher(
            Image, 
            '/detection_visualization', 
            10
        )
        self.info_pub = self.create_publisher(BallInfo, 'ball_info', 10)
        #self.start_info_pub = self.create_publisher(BallInfo, 'start_info', 10)

        self.centers = []
        self.class_names = []

        #添加启动区
        #self.start_centers = []
        #self.start_class_names = []

        self.safe_areas = []
        self.frames, self.loopTime, self.initTime = 0, time.time(), time.time()
        self.create_timer(0.033, self._timer_callback)
        
        self.get_logger().info("YOLOv8检测节点初始化完成")

    def _declare_parameters(self):
        """声明所有参数"""
        try:
            default_model_path = os.path.join(
                get_package_share_directory('ball_detection'),
                '88.rknn',
            )
        except PackageNotFoundError:
            default_model_path = os.path.join(
                os.path.dirname(__file__),
                '88.rknn',
            )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', default_model_path),
                ('camera_topic', '/raw_image'),
                ('obj_thresh', 0.30),  # 目标置信度阈值
                ('nms_thresh', 0.45),  # NMS阈值
                ('img_size', 640),      # 输入图像尺寸
                ('use_camera', True),  # 是否使用本地摄像头
                ('camera_index', 1),   # 摄像头索引
                ('camera_width', 640),
                ('camera_height', 480),
                ('camera_fps', 30.0),
                ('camera_auto_exposure', True),
                ('camera_exposure', 40.0),
                ('camera_brightness', -1.0),
                ('camera_contrast', -1.0),
                ('camera_gain', -1.0),
                ('camera_saturation', -1.0),
            ]
        )

    def _load_parameters(self):
        """加载参数值"""
        self.model_path = self.get_parameter('model_path').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.obj_thresh = self.get_parameter('obj_thresh').value
        self.nms_thresh = self.get_parameter('nms_thresh').value
        self.img_size = self.get_parameter('img_size').value
        self.use_camera = self.get_parameter('use_camera').value
        self.camera_index = self.get_parameter('camera_index').value
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fps = float(self.get_parameter('camera_fps').value)
        self.camera_auto_exposure = bool(self.get_parameter('camera_auto_exposure').value)
        self.camera_exposure = float(self.get_parameter('camera_exposure').value)
        self.camera_brightness = float(self.get_parameter('camera_brightness').value)
        self.camera_contrast = float(self.get_parameter('camera_contrast').value)
        self.camera_gain = float(self.get_parameter('camera_gain').value)
        self.camera_saturation = float(self.get_parameter('camera_saturation').value)

    def _initialize_camera(self):
        """初始化摄像头或订阅"""
        if self.use_camera:
            self.cap = cv2.VideoCapture(self.camera_index)
            fourcc = cv2.VideoWriter_fourcc(*'MJPG') 
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.camera_width))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.camera_height))
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            self._configure_camera_controls()
            #self.timer = self.create_timer(0.01, self._camera_callback)  # ~30fps
        else:
            self.subscription = self.create_subscription(
                Image,
                self.camera_topic,
                self._image_callback,
                10
            )

    def _configure_camera_controls(self):
        """根据参数尝试设置摄像头曝光、增益和亮度。"""
        if not self.cap or not self.cap.isOpened():
            return

        self._set_auto_exposure(self.camera_auto_exposure)

        if not self.camera_auto_exposure:
            self._try_set_camera_property('exposure', cv2.CAP_PROP_EXPOSURE, self.camera_exposure)

        self._try_set_optional_camera_property(
            'brightness', cv2.CAP_PROP_BRIGHTNESS, self.camera_brightness
        )
        self._try_set_optional_camera_property(
            'contrast', cv2.CAP_PROP_CONTRAST, self.camera_contrast
        )
        self._try_set_optional_camera_property(
            'gain', cv2.CAP_PROP_GAIN, self.camera_gain
        )
        self._try_set_optional_camera_property(
            'saturation', cv2.CAP_PROP_SATURATION, self.camera_saturation
        )

        self.get_logger().info(
            'Camera settings requested: '
            f'index={self.camera_index}, size={self.camera_width}x{self.camera_height}, '
            f'fps={self.camera_fps:.1f}, auto_exposure={self.camera_auto_exposure}, '
            f'exposure={self.camera_exposure}, brightness={self.camera_brightness}, '
            f'contrast={self.camera_contrast}, gain={self.camera_gain}, '
            f'saturation={self.camera_saturation}'
        )

    def _set_auto_exposure(self, enabled: bool):
        """兼容不同 OpenCV/V4L2 驱动对 auto exposure 的取值习惯。"""
        candidates = (0.75, 3.0) if enabled else (0.25, 1.0)
        success = False
        for candidate in candidates:
            success = self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, candidate) or success
        if not success:
            self.get_logger().warn('Camera auto exposure property may not be supported by this device')

    def _try_set_optional_camera_property(self, name, prop_id, value):
        if value < 0.0:
            return
        self._try_set_camera_property(name, prop_id, value)

    def _try_set_camera_property(self, name, prop_id, value):
        success = self.cap.set(prop_id, value)
        actual = self.cap.get(prop_id)
        if success:
            self.get_logger().info(f'Camera {name} set request={value}, actual={actual}')
        else:
            self.get_logger().warn(f'Camera {name} set failed, request={value}, actual={actual}')

    def _timer_callback(self):
        if self.cap.isOpened():
            self.frames += 1
            ret, frame = self.cap.read()
            if not ret:
                return
            #print(frame.shape)
            self.rknn_processor.put(frame)
            frame, flag = self.rknn_processor.get()
            if flag == False:
                return
            #print(frame.shape)
            self._publish_results(frame)
            if self.frames % 30 == 0:
                #self.get_logger().info(f"30帧平均帧率: {30 / (time.time() - self.loopTime):.2f} 帧")
                self.loopTime = time.time()

    '''def _camera_callback(self):
        """定时器回调函数 - 从摄像头捕获帧"""
        ret, frame = self.cap.read()
        if ret:
            self.rknn_processor.add_task(frame, self._process_frame)
            self._publish_results()'''

    def _image_callback(self, msg):
        """图像订阅回调函数"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.rknn_processor.add_task(frame, self._process_frame)
            self._publish_results(frame)
        except Exception as e:
            self.get_logger().error(f"图像转换错误: {str(e)}")

    def _publish_results(self,frame):
        """发布处理结果"""
        # 发布 BallInfo 消息
        if self.centers and self.class_names:
            ball_info_msg = BallInfo()
            # 填充位置信息
            for center in self.centers:
                point = Point()
                point.x = float(center[0])
                point.y = float(center[1])
                point.z = 0.0  # 2D检测，z设为0
                ball_info_msg.positions.append(point)
            
            ball_info_msg.classes = self.class_names
            # 发布消息
            self.info_pub.publish(ball_info_msg)
        '''# 发布 启动区 消息
        if self.start_centers and self.start_class_names:
            start_info_msg = BallInfo()
            # 填充位置信息
            for center in self.start_centers:
                point = Point()
                point.x = float(center[0])
                point.y = float(center[1])
                point.z = 0.0  # 2D检测，z设为0
                start_info_msg.positions.append(point)
            
            start_info_msg.classes = self.start_class_names
            # 发布消息
            self.start_info_pub.publish(start_info_msg)'''
        if frame is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.visualization_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"发布图像错误: {str(e)}")

    def _process_frame(self, rknn, frame):
        """处理单帧图像"""
        # 预处理
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized, ratio, padding = self._letterbox(img_rgb)

        img_resized = np.expand_dims(img_resized, axis=0)
        
        # 推理
        outputs = rknn.inference(inputs=[img_resized], data_format=['nhwc'])
        
        # 后处理
        boxes, classes, scores = self._post_process(outputs)
        # 绘制结果
        if boxes is not None:
            self._draw_detections(frame, boxes, scores, classes, ratio, padding)
        else:
            self.centers = []
            self.class_names = []
            #self.start_centers = []
            #self.start_class_names = []

        return frame

    def _letterbox(self, im, new_shape=(640, 640), color=(0, 0, 0)):
        """等比例缩放图像并添加边框"""
        shape = im.shape[:2]  # 原始形状 [高, 宽]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        
        # 计算缩放比例
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        
        # 计算填充
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]

        dw /= 2
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=color)  # add border
        
        return im, (r, r), (left, top)

    def _post_process(self, outputs):
        """后处理YOLOv8输出 (适配最新的 1x10x8400 单分支结构)"""
        # 取出唯一的输出张量
        out = outputs[0]
        
        # 压缩多余维度，变成 (10, 8400) 或 (8400, 10)
        out = np.squeeze(out)
        
        # 统一转置为 (8400, 10) 方便处理
        if out.shape[0] == 10:
            out = out.transpose(1, 0)
            
        # 提取边界框 (cx, cy, w, h) 和 类别概率
        boxes_cxcywh = out[:, :4]
        class_probs = out[:, 4:]
        
        # 找出每个框概率最大的类别和对应的分数
        class_ids = np.argmax(class_probs, axis=1)
        scores = np.max(class_probs, axis=1)
        
        # 1. 置信度过滤 (只保留大于 obj_thresh 的目标)
        valid_indices = np.where(scores >= self.obj_thresh)[0]
        if len(valid_indices) == 0:
            return None, None, None
            
        valid_boxes_cxcywh = boxes_cxcywh[valid_indices]
        valid_scores = scores[valid_indices]
        valid_class_ids = class_ids[valid_indices]
        
        # 2. 将中心点格式 (cx, cy, w, h) 转换为角点格式 (x1, y1, x2, y2)
        valid_boxes = np.zeros_like(valid_boxes_cxcywh)
        valid_boxes[:, 0] = valid_boxes_cxcywh[:, 0] - valid_boxes_cxcywh[:, 2] / 2
        valid_boxes[:, 1] = valid_boxes_cxcywh[:, 1] - valid_boxes_cxcywh[:, 3] / 2
        valid_boxes[:, 2] = valid_boxes_cxcywh[:, 0] + valid_boxes_cxcywh[:, 2] / 2
        valid_boxes[:, 3] = valid_boxes_cxcywh[:, 1] + valid_boxes_cxcywh[:, 3] / 2
        
        # 3. NMS (非极大值抑制)
        final_boxes, final_classes, final_scores = [], [], []
        for class_id in set(valid_class_ids):
            indices = np.where(valid_class_ids == class_id)[0]
            cls_boxes = valid_boxes[indices]
            cls_scores = valid_scores[indices]
            
            keep_indices = self._nms_boxes(cls_boxes, cls_scores)
            if len(keep_indices) > 0:
                final_boxes.append(cls_boxes[keep_indices])
                final_classes.append(np.full(len(keep_indices), class_id))
                final_scores.append(cls_scores[keep_indices])
                
        if not final_boxes:
            return None, None, None
            
        return np.concatenate(final_boxes), np.concatenate(final_classes), np.concatenate(final_scores)

    def _box_process(self, position):
        """处理边界框输出
        Args:
            position: 模型输出的边界框位置信息
            
        Returns:
            ndarray: 处理后的边界框坐标(xyxy格式)
        """
        shape = position.shape
        if len(shape) == 4:
            grid_h, grid_w = shape[2:4]
        elif len(shape) == 3:
            # 如果是 3 维 [1, C, N]，假设它是正方形网格 N = H * W
            grid_h = grid_w = int(np.sqrt(shape[2]))
            position = position.reshape(shape[0], shape[1], grid_h, grid_w)
        else:
            self.get_logger().error(f"意外的模型输出形状: {shape}")
            return None
       
        
        # 创建网格坐标
        col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
        col = col.reshape(1, 1, grid_h, grid_w)
        row = row.reshape(1, 1, grid_h, grid_w)
        grid = np.concatenate((col, row), axis=1)  # 1x2xHxW
        
        # 计算步长
        stride = np.array([self.img_size // grid_h, 
                        self.img_size // grid_w]).reshape(1, 2, 1, 1)
        
        # 分布焦点损失(DFL)处理
        position = self._dfl(position)
        
        # 计算边界框坐标
        box_xy = grid + 0.5 - position[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
        xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
        
        return xyxy

    def _dfl(self, position):
        """分布焦点损失(DFL)处理
        Args:
            position: 模型输出的位置信息
            
        Returns:
            ndarray: 处理后的位置信息
        """
        n, c, h, w = position.shape
        p_num = 4  # 每个边界框的参数数量(x,y,w,h)
        mc = c // p_num  # 每个参数的通道数
        
        # 重塑并计算softmax
        y = position.reshape(n, p_num, mc, h, w)
        y = np.exp(y - np.max(y, axis=2, keepdims=True))  # 数值稳定性
        y = y / np.sum(y, axis=2, keepdims=True)
        
        # 计算加权和
        acc_matrix = np.arange(mc).reshape(1, 1, mc, 1, 1)
        y = (y * acc_matrix).sum(axis=2)
        
        return y

    '''def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """根据置信度阈值过滤边界框
        Args:
            boxes: 边界框坐标
            box_confidences: 边界框置信度
            box_class_probs: 类别概率
            
        Returns:
            tuple: 过滤后的(boxes, classes, scores)
        """
        box_confidences = box_confidences.reshape(-1)
        num_candidates = box_class_probs.shape[0]
        
        # 计算每个框的最大类别分数
        class_max_scores = np.max(box_class_probs, axis=1)
        class_ids = np.argmax(box_class_probs, axis=1)
        
        # 应用置信度阈值
        valid_indices = np.where(class_max_scores * box_confidences >= self.obj_thresh)[0]
        
        if len(valid_indices) == 0:
            return None, None, None
        
        # 返回有效检测
        return boxes[valid_indices], class_ids[valid_indices], \
            (class_max_scores * box_confidences)[valid_indices]'''

    def _filter_boxes(self, boxes, box_confidences, box_class_probs):
        """Filter boxes with object threshold.
        """
        box_confidences = box_confidences.reshape(-1)
        candidate, class_num = box_class_probs.shape

        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)

        _class_pos = np.where(class_max_score* box_confidences >= self.obj_thresh)
        scores = (class_max_score* box_confidences)[_class_pos]

        boxes = boxes[_class_pos]
        classes = classes[_class_pos]

        return boxes, classes, scores

    '''def _nms_boxes(self, boxes, scores):
        """非极大值抑制(NMS)
        Args:
            boxes: 边界框坐标
            scores: 对应的分数
            
        Returns:
            ndarray: 保留的边界框索引
        """
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]  # 按分数降序排序
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            # 计算交并比(IoU)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            intersection = w * h
            
            iou = intersection / (areas[i] + areas[order[1:]] - intersection)
            
            # 保留IoU低于阈值的框
            keep_indices = np.where(iou <= self.nms_thresh)[0]
            order = order[keep_indices + 1]
        
        return np.array(keep)'''
    def _nms_boxes(self,boxes, scores):
        """Suppress non-maximal boxes.
        # Returns
            keep: ndarray, index of effective boxes.
        """
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        areas = w * h
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.nms_thresh)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep


    '''def _draw_detections(self, image, boxes, scores, classes, ratio, padding):
        """在图像上绘制检测结果"""
        centers = []
        class_names = []

        if len(boxes) == 0:
            # 无检测时清除状态
            self.centers = []
            self.class_names = []
            self.get_logger().debug("No detections found")
            return image, centers, class_names  # 返回空结果
        
        for box, score, cl in zip(boxes, scores, classes):
            # 坐标转换到原始图像
            top = int((box[0] - padding[0]) / ratio[0])
            left = int((box[1] - padding[1]) / ratio[1])
            right = int((box[2] - padding[0]) / ratio[0])
            bottom = int((box[3] - padding[1]) / ratio[1])
            current_box = (top, left, right, bottom)

            # 计算中点
            center_x = (top + right) // 2
            center_y = (bottom + left) // 2
        
            if center_y > 6:
                centers.append((center_x, center_y))
                class_names.append(self.classes[cl])
            else:
                continue

            # 绘制边界框
            cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
            
            # 绘制中点和标签
            cv2.circle(image, (center_x, center_y), 2, (0, 255, 0), -1)
            cv2.putText(image, f'{self.classes[cl]} {score:.2f}',
                       (top, left - 6), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (0, 0, 255), 2)
            
            
            self.centers = centers
            self.class_names = class_names'''
    def _draw_detections(self, image, boxes, scores, classes, ratio, padding):
        # 处理检测结果的代码
        centers = []
        class_names = []
        #start_centers = []  # 起始点坐标
        #start_class_names = []  # 起始点类别
        temp_safe_areas = []  # 临时存储本轮检测到的安全区域
        height, width = image.shape[:2]
        debug_grab_y = 290
        debug_safe_y = 230

        cv2.line(image, (0, debug_grab_y), (width - 1, debug_grab_y), (0, 165, 255), 2)
        cv2.putText(
            image,
            f'grab_y={debug_grab_y}',
            (10, max(25, debug_grab_y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 165, 255),
            2,
        )
        cv2.line(image, (0, debug_safe_y), (width - 1, debug_safe_y), (255, 255, 0), 2)
        cv2.putText(
            image,
            f'safe_y={debug_safe_y}',
            (10, max(50, debug_safe_y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

        if len(boxes) == 0:
            # 无检测时清除状态
            self.centers = []
            self.class_names = []
            #self.start_centers = []
            #self.start_class_names = []
            self.get_logger().debug("No detections found")
            return image, centers, class_names  # 返回空结果

        # 第一遍：先收集所有安全区域
        for box, score, cl in zip(boxes, scores, classes):
            class_name = self.classes[cl]
            if class_name not in ['bluesafe', 'redsafe']:
                continue

            if class_name in ['bluesafe', 'redsafe']:
                # 坐标转换
                top = int((box[0] - padding[0]) / ratio[0])
                left = int((box[1] - padding[1]) / ratio[1])
                right = int((box[2] - padding[0]) / ratio[0])
                bottom = int((box[3] - padding[1]) / ratio[1])

                # 检查ROI是否有效
                if top >= right or left >= bottom:
                    continue

                # 计算中心点
                center_x, center_y = (top+right)//2, (bottom+left)//2
                class_name = self.classes[cl]

                centers.append((center_x, center_y))
                class_names.append(class_name)

                # 绘制检测结果
                cv2.rectangle(image, (top,left), (right,bottom), (255,0,0), 2)
                cv2.circle(image, (center_x,center_y), 2, (0,255,0), -1)
                cv2.putText(image, f'{class_name} {score:.2f}',
                        (top, left-6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0,0,255), 2)
                
                try:
                    roi = image[left:bottom, top:right]
                    if roi.size == 0:  # 检查ROI是否为空
                        continue
                    if class_name == 'bluesafe':
                        lower, upper = np.array([93,50,50]), np.array([123,255,255])  # 蓝色
                    else:
                        lower, upper = np.array([139,50,50]), np.array([180,255,255])  # 红色
                        
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange(hsv, lower, upper)

                    # 查找最大轮廓
                    '''contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        cnt = max(contours, key=cv2.contourArea)
                        x,y,w,h = cv2.boundingRect(cnt)
                        # 转换回原图坐标并保存
                        safe_rect = (top+x, left+y, top+x+w, left+y+h)
                        temp_safe_areas.append(safe_rect)
                        # 绘制安全区域
                        cv2.drawContours(image, [cnt + (top,left)], -1, (0,255,255), 2)
                        cv2.rectangle(image, (safe_rect[0], safe_rect[1]), 
                                    (safe_rect[2], safe_rect[3]), (255,0,255), 2)'''
                    # 查找最大轮廓
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        cnt = max(contours, key=cv2.contourArea)
                        x, y, w, h = cv2.boundingRect(cnt)

                        if class_name == self.safe_color:
                            # 计算扩大的宽度和高度
                            w_new = w
                            h_new = h
                            # 计算新的左上角坐标
                            x_new = x
                            y_new = y
                            # 转换回原图坐标并保存
                            safe_rect = (top + x_new, left + y_new, top + x_new + w_new, left + y_new + h_new)
                            #self.get_logger().info(f"当前为本队安全区域")
                        else:
                            safe_rect = (top, left, right, bottom)
                        
                        temp_safe_areas.append(safe_rect)
                        # 对轮廓的每个点进行坐标转换
                        cnt_translated = cnt + np.array([top, left]).reshape(-1, 1, 2)
                        # 绘制安全区域
                        cv2.drawContours(image, [cnt_translated.astype(int)], -1, (0, 255, 255), 2)
                        # 绘制矩形
                        cv2.rectangle(image, (int(safe_rect[0]), int(safe_rect[1])), 
                                    (int(safe_rect[2]), int(safe_rect[3])), (255, 0, 255), 2)

                except Exception as e:
                    self.get_logger().warn(f"处理安全区域时出错: {str(e)}")
                    continue
            
        # 更新安全区域
        self.safe_areas = temp_safe_areas



        # 第二遍：处理所有目标
        for box, score, cl in zip(boxes, scores, classes):
            # 坐标转换
            top = int((box[0] - padding[0]) / ratio[0])
            left = int((box[1] - padding[1]) / ratio[1])
            right = int((box[2] - padding[0]) / ratio[0])
            bottom = int((box[3] - padding[1]) / ratio[1])
            
            # 计算中心点
            center_x, center_y = (top+right)//2, bottom
            class_name = self.classes[cl]
            
            # 跳过安全区域自身
            if class_name in ['bluesafe', 'redsafe']:
                continue

            # 添加启动区
            '''if class_name in ['redstart', 'bluestart']:
                center_y = (bottom + left) // 2
                start_centers.append((center_x, center_y))
                start_class_names.append(class_name)
                
                # 绘制起始点
                cv2.circle(image, (center_x,center_y), 2, (0,255,0), -1)
                cv2.putText(image, f'{class_name} {score:.2f}',
                        (top, left-6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0,0,255), 2)
                self.start_centers = start_centers
                self.start_class_names = start_class_names
                continue'''
                
            # 检查是否在安全区域内
            in_safe_area = any(
                (x1 < center_x < x2 and y1 < center_y < y2)
                for x1,y1,x2,y2 in self.safe_areas
            )
            
            # 只处理不在安全区域且y坐标大于6的目标
            if not in_safe_area and center_y > 6:
                centers.append((center_x, center_y))
                class_names.append(class_name)
                
                # 绘制检测结果
                cv2.rectangle(image, (top,left), (right,bottom), (255,0,0), 2)
                cv2.circle(image, (center_x,center_y), 2, (0,255,0), -1)
                cv2.putText(image, f'{class_name} {score:.2f}',
                        (top, left-6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0,0,255), 2)

        # 保存结果
        for center_x, center_y in centers:
            cv2.putText(
                image,
                f'x={center_x} y={center_y}',
                (center_x + 8, min(height - 10, center_y + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

        self.centers = centers
        self.class_names = class_names
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'rknn_processor'):
            self.rknn_processor.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    detector = YoloV8Detector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info("节点关闭")
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
