#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ai_msgs.msg import PerceptionTargets
import subprocess  # 用于执行外部命令
import time        # 修改点 1: 导入 time 模块用于时间戳处理 (注意：您代码中的 Sleep 不是标准库)

# 音频文件的路径是固定的
AUDIO_FILE_PATH = "/home/sunrise/jianye_dev/help.mp3"


class FalldownDetector(Node):
    def __init__(self):
        super().__init__('falldown_detector')
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_falldown_detection',
            self.perception_callback,
            10)
        self.publisher = self.create_publisher(Int32, '/falldown_alert', 10)

        # 添加点 1: 初始化上次触发时间和冷却周期
        # 初始化为0，确保第一次收到消息时总能触发
        self.last_triggered_time = 0.0
        # 设置30秒的冷却时间
        self.cooldown_period = 10.0

        self.get_logger().info("Falldown detector node has been started.")
        self.get_logger().info(f"Alerts will be sent at most once every {self.cooldown_period} seconds.")

    def perception_callback(self, msg):
        # 遍历消息中的目标，寻找跌倒事件
        for target in msg.targets:
            for attr in target.attributes:
                # 跌倒事件的判断逻辑保持不变
                if attr.type == 'falldown' and attr.value == 1.0:

                    # 添加点 2: 在执行核心逻辑前，检查冷却时间
                    current_time = time.time() # 获取当前时间戳
                    if current_time - self.last_triggered_time < self.cooldown_period:
                        # 如果还在冷却期内，则记录一条信息并直接返回，不执行任何操作
                        self.get_logger().info("Cooldown active. Skipping fall detection alert.")
                        return

                    # 如果冷却期已过，立即更新触发时间，为下一次冷却做准备
                    self.last_triggered_time = current_time

                    # --- 以下是原来的核心逻辑 ---
                    
                    # 播放音频
                    try:
                        subprocess.Popen(['play', AUDIO_FILE_PATH],
                                         stdout=subprocess.DEVNULL, 
                                         stderr=subprocess.DEVNULL)
                    except FileNotFoundError:
                        self.get_logger().error("播放命令 'play' 未找到。请确保已安装 'sox' (sudo apt-get install sox)。")
                    except Exception as e:
                        self.get_logger().error(f"播放音频时出错: {e}")
                    
                    # 发布消息
                    alert_msg = Int32()
                    alert_msg.data = 1
                    self.publisher.publish(alert_msg)
                    
                    # 记录警告日志
                    self.get_logger().warn(f"检测到跌倒！已在 /falldown_alert 话题上发布数据: {alert_msg.data}")
                    
                    # 在处理完第一个跌倒事件后返回，避免在单次消息中重复触发
                    return

def main(args=None):
    rclpy.init(args=args)
    node = FalldownDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()