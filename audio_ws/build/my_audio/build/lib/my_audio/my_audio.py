# 导入所需的库
import rclpy
from rclpy.node import Node
# 修正: 同时导入 SmartAudioData 和 SmartAudioFrameType
from audio_msg.msg import SmartAudioData, SmartAudioFrameType
import subprocess  # 用于执行外部命令
import os          # 用于处理文件路径

AUDIO_FILE_PATH = "/home/sunrise/jianye_dev/hello.mp3"

class AudioSubscriber(Node):
    """
    一个订阅 /audio_smart 话题并监听特定指令词的节点。
    """
    def __init__(self):
        """
        初始化节点并创建订阅者。
        """
        super().__init__('audio_subscriber_node')
        
        self.subscription = self.create_subscription(
            SmartAudioData,
            '/audio_smart',
            self.listener_callback,
            10)
        self.subscription
        
        self.get_logger().info('节点已启动，正在监听 /audio_smart 话题...')

    def listener_callback(self, msg):
        """
        当接收到消息时，此回调函数会被调用。
        """
        # 修正: 常量 SMART_AUDIO_TYPE_CMD_WORD 属于 SmartAudioFrameType， 而不是 SmartAudioData
        # 所以我们应该使用 SmartAudioFrameType.SMART_AUDIO_TYPE_CMD_WORD 来进行比较。
        if msg.frame_type.value == SmartAudioFrameType.SMART_AUDIO_TYPE_CMD_WORD and msg.cmd_word == "小玉同学":
            # 如果是，则在终端输出提示信息
            self.get_logger().info('检测到指令: "地瓜地瓜"！')
            """
            当接收到消息时，此回调函数被调用，直接播放指定的音频文件。
            """
            try:
                # 直接调用 'play' 命令播放固定的音频文件
                # Popen 会在后台执行，不会阻塞回调函数
                subprocess.Popen(['play', AUDIO_FILE_PATH],
                                stdout=subprocess.DEVNULL, 
                                stderr=subprocess.DEVNULL)
            except FileNotFoundError:
                # 这个错误只会在 'play' 命令本身找不到时发生
                self.get_logger().error("播放命令 'play' 未找到。请确保已安装 'sox' (sudo apt-get install sox)。")
            except Exception as e:
                self.get_logger().error(f"播放音频时出错: {e}")

def main(args=None):
    """
    主函数，用于初始化并运行 ROS 2 节点。
    """
    rclpy.init(args=args)
    
    audio_subscriber = AudioSubscriber()
    
    rclpy.spin(audio_subscriber)
    
    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()