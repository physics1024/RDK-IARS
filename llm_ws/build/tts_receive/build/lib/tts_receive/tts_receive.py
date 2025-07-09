import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from edge_tts import Communicate
import asyncio
import datetime
import threading
import os


class TTSSubscriber(Node):
    def __init__(self):
        super().__init__('tts_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/tts_text',
            self.listener_callback,
            10)
        self.get_logger().info('Listening to /tts_text...')

        # 创建 asyncio 队列，用于缓存收到的消息
        self.queue = asyncio.Queue()

        # 启动后台线程运行 asyncio 事件循环
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_asyncio_loop, daemon=True).start()

        # 启动异步任务处理语音合成
        asyncio.run_coroutine_threadsafe(self.process_queue(), self.loop)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        asyncio.run_coroutine_threadsafe(self.queue.put(msg.data), self.loop)

    def run_asyncio_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def process_queue(self):
        AUDIO_DIR = "tts_audio"
        os.makedirs(AUDIO_DIR, exist_ok=True)

        while True:
            text = await self.queue.get()

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"tts_{timestamp}.mp3"
            filepath = os.path.join(AUDIO_DIR, filename)
            self.get_logger().info(f'Saving as {filepath}')

            try:
                communicate = Communicate(text, voice="zh-CN-XiaoxiaoNeural")
                await communicate.save(filepath)
                self.get_logger().info(f'Audio saved to {filepath}')

                # 异步播放
                proc = await asyncio.create_subprocess_exec(
                    "play", filepath,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL
                )
                await proc.wait()
                self.get_logger().info(f'Playback finished: {filepath}')

            except Exception as e:
                self.get_logger().error(f'Failed to process {filepath}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TTSSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
