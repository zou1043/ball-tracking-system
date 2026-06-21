#!/usr/bin/env python3
import rclpy
import serial
import struct
import time
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Int32, Bool
from serial.tools import list_ports
from geometry_msgs.msg import Twist
from ball_interfaces.msg import BallInfo
from ball_interfaces.srv import Check
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class STMNode(Node):
    def __init__(self):
        super().__init__('stm_node')

        self.group = MutuallyExclusiveCallbackGroup()
        
        # 参数配置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('timeout', 1.0),
                ('safe_threshold', 0.5)
            ]
        )
        self.declare_parameter('team_color', 'red')  # 从启动参数获取队伍颜色
        self.team_color = self.get_parameter('team_color').value
        self.get_logger().info(f"队伍颜色: {self.team_color}")

        #串口初始化
        try:
            port_param = self.get_parameter('port')
            selected_port = port_param.value
            # 如果参数未设置，自动检测可用端口
            if port_param == '/dev/ttyUSB0':
                ports = list_ports.comports()
                available_ports = [port.device for port in ports]
                if available_ports:
                    selected_port = available_ports[0]
                    self.set_parameters([rclpy.parameter.Parameter(
                        'port', 
                        rclpy.Parameter.Type.STRING, 
                        selected_port
                    )])
        except Exception as e:
            self.get_logger().error(f"参数初始化失败: {str(e)}")
            raise

        self.ser = None
        try:
            self.ser = serial.Serial(
                port=selected_port,
                baudrate=self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value
            )
            self.get_logger().info(f"成功连接到串口: {selected_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"串口连接失败: {str(e)}")
            # 可根据需要重试或终止
            raise

        # 4. 改进的析构方法
        self._shutdown = False
            
        # 状态变量
        self.arrive_status = 0
        self.last_clamp_cmd = 0
        
        # 速度相关变量
        self.last_vision_linear = 0.0
        self.last_vision_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.run_first = True

        #接收到消息
        #self.receive = True
        self.servoup = False
        self.servodown = False
        self.ball_info_future = None  # 用于同步等待

        #目标信息
        self.pos = []
        self.cls = []
        
        self.vision_twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel_vision',
            self.vision_twist_callback,
            10)
        
        self.getball_sub = self.create_subscription(
            Bool,
            '/get_ball',
            self.getball_callback,
            10)
        
        self.ball_info_sub = self.create_subscription(
            BallInfo,
            '/ball_info',
            self.ball_info_callback,
            10)
        
        self.closeservo_sub = self.create_subscription(
            Bool,
            '/close_servo',
            self.closeservo_callback,
            10)
        
        # 发布器
        self.arrive_pub = self.create_publisher(
            Bool,
            'safe_arrive',
            qos_profile=rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        
        '''self.go_start_pub = self.create_publisher(
            Bool,
            'go_start',
            10)'''
        
        self.srv = self.create_service(
            Check,                 # 服务类型
            'check_ball',             # 服务名称
            self.check_callback)          # 回调函数

            
        '''# 定时器（50ms周期）
        self.timer = self.create_timer(
            0.05,
            self.update_status)'''

    def vision_twist_callback(self, msg):
        """视觉速度回调"""
        self.last_vision_linear = msg.linear.x
        self.last_vision_angular = msg.angular.z
        #self.receive = True
        self.update_status()
    def closeservo_callback(self, msg):
        """关闭舵机回调"""
        self.closeservo = msg
        if self.closeservo:
            self.ser.write(bytes.fromhex('FF0301FE'))
            self.servodown = True
            self.get_logger().info("舵机关闭")


    def check_callback(self, request, response):
        """检查球回调"""
        if request.enable:

            self.get_logger().info(f"self.cls:{self.cls}")
            check_flag = self.judge_ball(self.pos,self.cls)
            if check_flag == 1:
                #成功,前往安全区
                response.success = True
                self.ser.write(bytes.fromhex('FF0104FE'))
                self.get_logger().info("成功夹球")

            elif check_flag == 0:
                #扔球
                response.success = False
                self.ser.write(bytes.fromhex('FF0101FE'))
                self.get_logger().info("发送扔球指令")
                time.sleep(1)
            elif check_flag == -1:
                #没夹到球
                response.success = False
                self.ser.write(bytes.fromhex('FF0102FE'))
                self.get_logger().info("没夹到球")
            else :
                #后退夹球，随后前往安全区
                response.success = False
                self.ser.write(bytes.fromhex('FF0101FE'))
                self.get_logger().info("后退夹球")
                time.sleep(1)
            #舵机复位
            self.ser.write(bytes.fromhex('FF0302FE'))
            time.sleep(1)
            self.get_logger().info("舵机复位")
            return response

    
    def getball_callback(self,msg):
        '''抓球请求回调'''
        self.getball = msg
        #到达位置请求抓球
        if self.getball:
            try:
                self.ser.write(bytes.fromhex('FF0201FE'))
                self.get_logger().info("发送抓球指令")
            except serial.SerialException as e:
                self.get_logger().error(f"串口写入失败: {str(e)}")
                self.reconnect_serial()
    
    def ball_info_callback(self, msg):
        """球信息回调"""
        # 先在局部变量处理
        new_pos = [p for p, c in zip(msg.positions, msg.classes) 
                   if 200 <= p.x <= 440 and p.y >= 320 and c not in ['bluesafe', 'redsafe']]
        new_cls = [c for p, c in zip(msg.positions, msg.classes) 
                   if 200 <= p.x <= 440 and p.y >= 320 and c not in ['bluesafe', 'redsafe']]

        # 一次性更新（原子操作）
        self.pos, self.cls = new_pos, new_cls

    def update_status(self):
        """定时状态更新"""
        '''if not self.receive:
            self.last_vision_linear = 0.0
            self.last_vision_angular = 0.0'''

        #当目标出现，先停下
        if self.last_vision_linear != 0.0 and self.last_vision_angular != 0.0 and self.current_angular == 3.0 and self.current_linear == 0.0:
            self.current_angular = 0.0
            self.current_linear = 0.0
            linear_bytes = struct.pack('<i', int(self.current_linear * 1000))
            angular_bytes = struct.pack('<i', int(self.current_angular * 1000))
            frame = b'\xFF\x01' + linear_bytes + angular_bytes + b'\xFE'
            self.ser.write(frame)
            self.get_logger().info("发送停止指令")
            time.sleep(0.08)
            return
        
        #未找到目标旋转
        if self.last_vision_linear == 0.0 and self.last_vision_angular == 0.0:
            self.current_linear = 0.0
            self.current_angular = 3.0
        else:
            self.current_linear = self.last_vision_linear
            self.current_angular = self.last_vision_angular
        
        #发送速度信息
        linear_bytes = struct.pack('<i', int(self.current_linear * 1000))
        angular_bytes = struct.pack('<i', int(self.current_angular * 1000))
        frame = b'\xFF\x01' + linear_bytes + angular_bytes + b'\xFE'
        #self.get_logger().info(f"发送速度指令: {self.current_linear}, {self.current_angular}")
        self.ser.write(frame)
            # 重置计数器
        self.zero_count = 0
        #self.receive = False

        '''# 准备数据（放大1000倍转为整数，并限制在int16范围）
        linear_scaled = int(round(self.current_linear * 1000))
        angular_scaled = int(round(self.current_angular * 1000))
        
        # 限制在int16范围内(-32768~32767)
        linear_scaled = max(min(linear_scaled, 32767), -32768)
        angular_scaled = max(min(angular_scaled, 32767), -32768)
        
        # 将速度值打包为2字节大端有符号整数
        linear_bytes = struct.pack('>h', linear_scaled)  # 'h'表示int16
        angular_bytes = struct.pack('>h', angular_scaled)
        
        # 构建帧（不含校验和）
        frame_code = 0x01  # 速度控制帧码
        frame_length = 2 + 1 + 1 + 2 + 2 + 1  # 头2 + 长度1 + 帧码1 + 数据4 + 校验1 = 11(0x0B)
        frame_without_checksum = (
            b'\xAA\x55' + 
            bytes([frame_length]) + 
            bytes([frame_code]) + 
            linear_bytes + 
            angular_bytes
        )
        
        # 计算校验和
        checksum = sum(frame_without_checksum) & 0xFF
        
        # 构建完整帧
        full_frame = frame_without_checksum + bytes([checksum])
        
        # 调试输出
        self.get_logger().info(
            f"发送速度指令: 线速度={self.current_linear:.3f} m/s, 角速度={self.current_angular:.3f} rad/s\n"
        )
        
        self.ser.write(full_frame)'''
        # 接收数据
        self.recieve_data()
        

    def recieve_data(self):
        """接收数据"""
        # 处理接收数据（原有逻辑）
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(size=1)
                self.parse_sensor_data(data)
        except serial.SerialException as e:
            self.get_logger().error(f"串口读取失败: {str(e)}")
            self.reconnect_serial()

    def parse_sensor_data(self, data):
        """传感器数据解析（示例）"""
        # 检查接收到的数据是否为 '01'
        if data == b'\x01':
            msg = Bool()
            msg.data = True
            self.arrive_pub.publish(msg)
            self.get_logger().info("接收到stm32信息，到达安全区")
        elif data == b'\x02':
            self.servoup = True
        elif data == b'\x03':
            self.servodown = True
        #elif data == b'\x04':
        #    self.goto_start = Bool()
        #    self.goto_start.data = True
        #    self.go_start_pub.publish(self.goto_start)
        else:
            pass

    def reconnect_serial(self):
        """串口重连"""
        try:
            self.ser.close()
            self.ser.open()
            self.get_logger().info("串口重连成功")
        except Exception as e:
            self.get_logger().error(f"串口重连失败: {str(e)}")

    def judge_ball(self, ball_pos, ball_cls):
        """判断是否抓到正确球
        Args:
            ball_pos: 球的位置列表，格式为 [pos1, pos2, ...]，每个pos有x和y属性
            ball_cls: 球的类别列表，与ball_pos一一对应
        Returns:
            -1: 未检测到球
            0: 不符合抓取条件
            1: 符合所有抓取规则
            2: 颜色符合但数量不对，且该颜色球y坐标最接近480
        """
        
        if len(ball_pos) == 0:
            return -1  # 无球
        
        if self.run_first:
            # 首次运行只能夹取不大于三个红球
            if len(ball_pos) <= 3 and all(cls == self.team_color for cls in ball_cls):
                self.run_first = False
                return 1
            else:
                # 检查是否有红球且y坐标最接近480
                team_ball_indices = [i for i, cls in enumerate(ball_cls) if cls == self.team_color]
                if team_ball_indices:
                    # 计算所有红球到y=480的距离
                    distances = [abs(480 - ball_pos[i].y) for i in team_ball_indices]
                    min_dist_idx = team_ball_indices[distances.index(min(distances))]
                    # 检查是否全局最近
                    all_distances = [abs(480 - pos.y) for pos in ball_pos]
                    if distances[distances.index(min(distances))] == min(all_distances) and min(all_distances)>=450:
                        self.run_first = False
                        return 2
                self.get_logger().info("首次运行失败")
                return 0
        else:
            # 非首次运行规则
            # 检查黄球是否单独转运
            if "yellow" in ball_cls:
                if len(ball_pos) != 1:
                    # 检查黄球是否y坐标最接近480
                    yellow_indices = [i for i, cls in enumerate(ball_cls) if cls == "yellow"]
                    distances = [abs(480 - ball_pos[i].y) for i in yellow_indices]
                    min_dist_idx = yellow_indices[distances.index(min(distances))]
                    all_distances = [abs(480 - pos.y) for pos in ball_pos]
                    if distances[distances.index(min(distances))] == min(all_distances):
                        return 2
                    self.get_logger().info("黄球数量有误，扔掉")
                    return 0
                else:
                    return 1
            
            # 检查球数是否超过3个
            if len(ball_pos) > 3:
                self.get_logger().info("球数超过3个，后退夹球")
                return 2
            
            # 所有条件都满足
            return 1

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = STMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()