# -- coding: utf-8 --
import os, time, base64, asyncio
from omni_realtime_client import OmniRealtimeClient, TurnDetectionMode
import pyaudio
import queue
import threading

# 创建一个全局音频队列和播放线程
audio_queue = queue.Queue()
audio_player = None
# 添加全局中断标志
interrupt_flag = threading.Event()

# 初始化PyAudio
p = pyaudio.PyAudio()
RATE = 24000  # 采样率 24kHz
CHUNK = 3200  # 每个音频块的大小
FORMAT = pyaudio.paInt16  # 16位PCM格式
CHANNELS = 1  # 单声道

def clear_audio_queue():
    """清空音频队列"""
    with audio_queue.mutex:
        audio_queue.queue.clear()
    print("音频队列已清空")

def handle_interrupt():
    """处理中断事件 - 立即停止音频播放"""
    print("检测到语音输入，停止音频播放")
    interrupt_flag.set()  # 设置中断标志
    clear_audio_queue()  # 清空队列

def audio_player_thread():
    """后台线程用于播放音频数据"""
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=24000,
        output=True,
        frames_per_buffer=CHUNK,
    )

    try:
        while True:
            try:
                # 检查中断标志
                if interrupt_flag.is_set():
                    print("音频播放被中断")
                    interrupt_flag.clear()  # 清除中断标志
                    continue
                
                # 从队列获取音频数据
                audio_data = audio_queue.get(block=True, timeout=0.5)
                if audio_data is None:  # 结束信号
                    break
                
                # 再次检查中断标志（在播放前）
                if interrupt_flag.is_set():
                    print("音频播放被中断")
                    interrupt_flag.clear()  # 清除中断标志
                    audio_queue.task_done()
                    continue
                
                # 播放音频数据
                stream.write(audio_data)
                audio_queue.task_done()
            except queue.Empty:
                # 如果队列为空，继续等待
                continue
    finally:
        # 清理
        stream.stop_stream()
        stream.close()


def start_audio_player():
    """启动音频播放线程"""
    global audio_player
    if audio_player is None or not audio_player.is_alive():
        audio_player = threading.Thread(target=audio_player_thread, daemon=True)
        audio_player.start()


def handle_audio_data(audio_data):
    """处理接收到的音频数据"""
    # 打印接收到的音频数据长度（调试用）
    print(f"\n接收到音频数据: {len(audio_data)} 字节")
    # 将音频数据放入队列
    audio_queue.put(audio_data)


async def start_microphone_streaming(client: OmniRealtimeClient):
    CHUNK = 3200
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 24000

    p = pyaudio.PyAudio()
    stream = p.open(
        format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK
    )

    try:
        print("开始录音，请讲话...")
        while True:
            audio_data = stream.read(CHUNK)
            encoded_data = base64.b64encode(audio_data).decode("utf-8")

            eventd = {
                "event_id": "event_" + str(int(time.time() * 1000)),
                "type": "input_audio_buffer.append",
                "audio": encoded_data,
            }
            await client.send_event(eventd)

            # 保持较短的等待时间以模拟实时交互
            await asyncio.sleep(0.05)
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()


async def main():
    # 启动音频播放线程
    start_audio_player()

    realtime_client = OmniRealtimeClient(
        base_url="wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        api_key=os.environ.get("DASHSCOPE_API_KEY"),
        model="qwen-omni-turbo-realtime",
        voice="Chelsie",
        on_text_delta=lambda text: print(f"\nAssistant: {text}", end="", flush=True),
        on_audio_delta=handle_audio_data,
        on_interrupt=handle_interrupt,  # 添加中断回调函数
        turn_detection_mode=TurnDetectionMode.SERVER_VAD,
    )

    try:
        await realtime_client.connect()
        # 启动消息处理和麦克风录音
        message_handler = asyncio.create_task(realtime_client.handle_messages())
        streaming_task = asyncio.create_task(
            start_microphone_streaming(realtime_client)
        )

        while True:
            await asyncio.Queue().get()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 结束音频播放线程
        audio_queue.put(None)
        if audio_player:
            audio_player.join(timeout=1)
        await realtime_client.close()
        p.terminate()

if __name__ == "__main__":
    asyncio.run(main())