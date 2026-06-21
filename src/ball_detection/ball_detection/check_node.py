#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.srv import Check
import time

class CheckBallTester(Node):
    def __init__(self):
        super().__init__('check_node')
        self.client = self.create_client(Check, '/check_ball')
        self.timer = self.create_timer(1.0, self.test_call)
        self.call_count = 0

    def test_call(self):
        self.call_count += 1
        self.get_logger().info(f"\n{'='*40}\n测试调用 #{self.call_count}")

        # 1. 检查服务可用性
        if not self.client.service_is_ready():
            self.get_logger().error("服务不可用，等待连接...")
            if not self.client.wait_for_service(timeout_sec=2.0):
                self.get_logger().fatal("服务连接超时！")
                return

        # 2. 创建请求
        req = Check.Request()
        req.enable = True

        # 3. 异步调用+超时处理
        future = self.client.call_async(req)
        start_time = time.time()
        timeout_sec = 3.0

        while rclpy.ok():
            # 非阻塞处理
            rclpy.spin_once(self, timeout_sec=0.1)

            if future.done():
                try:
                    response = future.result()
                    if response is None:
                        self.get_logger().error("收到空响应")
                    else:
                        self.get_logger().info(f"测试成功！响应: success={response.success}")
                except Exception as e:
                    self.get_logger().error(f"服务异常: {str(e)}")
                break

            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"调用超时（{timeout_sec}秒）")
                break

def main(args=None):
    rclpy.init(args=args)
    tester = CheckBallTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("测试终止")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()