# -- coding: utf-8 --
import os
import asyncio
import time
import threading
import queue
import pyaudio
from omni_realtime_client import OmniRealtimeClient, TurnDetectionMode

class AudioPlayer:
    """实时音频播放器类"""
    def __init__(self, sample_rate=24000, channels=1, sample_width=2):
        self.sample_rate = sample_rate
        self.channels = channels
        self.sample_width = sample_width  # 2 bytes for 16-bit
        self.audio_queue = queue.Queue()
        self.is_playing = False
        self.play_thread = None
        self.pyaudio_instance = None
        self.stream = None
        self._lock = threading.Lock()  # 添加锁来同步访问
        self._last_data_time = time.time()  # 记录最后接收数据的时间
        self._response_done = False  # 添加响应完成标志
        self._waiting_for_response = False # 标记是否正在等待服务器响应
        # 记录最后一次向音频流写入数据的时间及最近一次音频块的时长，用于更精确地判断播放结束
        self._last_play_time = time.time()
        self._last_chunk_duration = 0.0
        
    def start(self):
        """启动音频播放器"""
        with self._lock:
            if self.is_playing:
                return
                
            self.is_playing = True
            
            try:
                self.pyaudio_instance = pyaudio.PyAudio()
                
                # 创建音频输出流
                self.stream = self.pyaudio_instance.open(
                    format=pyaudio.paInt16,  # 16-bit
                    channels=self.channels,
                    rate=self.sample_rate,
                    output=True,
                    frames_per_buffer=1024
                )
                
                # 启动播放线程
                self.play_thread = threading.Thread(target=self._play_audio)
                self.play_thread.daemon = True
                self.play_thread.start()
                
                print("音频播放器已启动")
            except Exception as e:
                print(f"启动音频播放器失败: {e}")
                self._cleanup_resources()
                raise
    
    def stop(self):
        """停止音频播放器"""
        with self._lock:
            if not self.is_playing:
                return
                
            self.is_playing = False
        
        # 清空队列
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break
        
        # 等待播放线程结束（在锁外面等待，避免死锁）
        if self.play_thread and self.play_thread.is_alive():
            self.play_thread.join(timeout=2.0)
        
        # 再次获取锁来清理资源
        with self._lock:
            self._cleanup_resources()
        
        print("音频播放器已停止")
    
    def _cleanup_resources(self):
        """清理音频资源（必须在锁内调用）"""
        try:
            # 关闭音频流
            if self.stream:
                if not self.stream.is_stopped():
                    self.stream.stop_stream()
                self.stream.close()
                self.stream = None
        except Exception as e:
            print(f"关闭音频流时出错: {e}")
        
        try:
            if self.pyaudio_instance:
                self.pyaudio_instance.terminate()
                self.pyaudio_instance = None
        except Exception as e:
            print(f"终止PyAudio时出错: {e}")
    
    def add_audio_data(self, audio_data):
        """添加音频数据到播放队列"""
        if self.is_playing and audio_data:
            self.audio_queue.put(audio_data)
            with self._lock:
                self._last_data_time = time.time()  # 更新最后接收数据的时间
                self._waiting_for_response = False # 收到数据，不再等待
    
    def stop_receiving_data(self):
        """标记不再接收新的音频数据"""
        with self._lock:
            self._response_done = True
            self._waiting_for_response = False # 响应结束，不再等待
    
    def prepare_for_next_turn(self):
        """为下一轮对话重置播放器状态。"""
        with self._lock:
            self._response_done = False
            self._last_data_time = time.time()
            self._last_play_time = time.time()
            self._last_chunk_duration = 0.0
            self._waiting_for_response = True # 开始等待下一轮响应
        
        # 清空上一轮可能残留的音频数据
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break

    def is_finished_playing(self):
        """检查是否已经播放完所有音频数据"""
        with self._lock:
            queue_size = self.audio_queue.qsize()
            time_since_last_data = time.time() - self._last_data_time
            time_since_last_play = time.time() - self._last_play_time
            
            # ---------------------- 智能结束判定 ----------------------
            # 1. 首选：如果服务器已标记完成且播放队列为空
            #    进一步等待最近一块音频播放完毕（音频块时长 + 0.1s 容错）。
            if self._response_done and queue_size == 0:
                min_wait = max(self._last_chunk_duration + 0.1, 0.5)  # 至少等待 0.5s
                if time_since_last_play >= min_wait:
                    return True

            # 2. 备用：如果长时间没有新数据且播放队列为空
            #    当服务器没有明确发出 `response.done` 时，此逻辑作为保障
            if not self._waiting_for_response and queue_size == 0 and time_since_last_data > 1.0:
                print("\n(超时未收到新音频，判定播放结束)")
                return True
            
            return False
    
    def _play_audio(self):
        """播放音频数据的工作线程"""
        while True:
            # 检查是否应该停止
            with self._lock:
                if not self.is_playing:
                    break
                stream_ref = self.stream  # 获取流的引用
            
            try:
                # 从队列中获取音频数据，超时0.1秒
                audio_data = self.audio_queue.get(timeout=0.1)
                
                # 再次检查状态和流的有效性
                with self._lock:
                    if self.is_playing and stream_ref and not stream_ref.is_stopped():
                        try:
                            # 播放音频数据
                            stream_ref.write(audio_data)
                            # 更新最近播放信息
                            self._last_play_time = time.time()
                            self._last_chunk_duration = len(audio_data) / (self.channels * self.sample_width) / self.sample_rate
                        except Exception as e:
                            print(f"写入音频流时出错: {e}")
                            break
                    
                # 标记该数据块已处理完成
                self.audio_queue.task_done()

            except queue.Empty:
                # 队列为空时继续等待
                continue
            except Exception as e:
                print(f"播放音频时出错: {e}")
                break

class MicrophoneRecorder:
    """实时麦克风录音器"""
    def __init__(self, sample_rate=16000, channels=1, chunk_size=1024):
        self.sample_rate = sample_rate
        self.channels = channels
        self.chunk_size = chunk_size
        self.pyaudio_instance = None
        self.stream = None
        self.frames = []
        self._is_recording = False
        self._record_thread = None

    def _recording_thread(self):
        """录音工作线程"""
        # 在 _is_recording 为 True 期间，持续从音频流中读取数据
        while self._is_recording:
            try:
                # 使用 exception_on_overflow=False 避免因缓冲区溢出而崩溃
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                self.frames.append(data)
            except (IOError, OSError) as e:
                # 当流被关闭时，读取操作可能会引发错误
                print(f"录音流读取错误，可能已关闭: {e}")
                break

    def start(self):
        """开始录音"""
        if self._is_recording:
            print("录音已在进行中。")
            return
        
        self.frames = []
        self._is_recording = True
        
        try:
            self.pyaudio_instance = pyaudio.PyAudio()
            self.stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            
            self._record_thread = threading.Thread(target=self._recording_thread)
            self._record_thread.daemon = True
            self._record_thread.start()
            print("麦克风录音已开始...")
        except Exception as e:
            print(f"启动麦克风失败: {e}")
            self._is_recording = False
            self._cleanup()
            raise

    def stop(self):
        """停止录音并返回音频数据"""
        if not self._is_recording:
            return None
            
        self._is_recording = False
        
        # 等待录音线程安全退出
        if self._record_thread:
            self._record_thread.join(timeout=1.0)
        
        self._cleanup()
        
        print("麦克风录音已停止。")
        return b''.join(self.frames)

    def _cleanup(self):
        """安全地清理 PyAudio 资源"""
        if self.stream:
            try:
                if self.stream.is_active():
                    self.stream.stop_stream()
                self.stream.close()
            except Exception as e:
                print(f"关闭音频流时出错: {e}")
        
        if self.pyaudio_instance:
            try:
                self.pyaudio_instance.terminate()
            except Exception as e:
                print(f"终止 PyAudio 实例时出错: {e}")

        self.stream = None
        self.pyaudio_instance = None

async def interactive_test():
    """
    交互式测试脚本：允许多轮连续对话，每轮可以发送音频和图片。
    """
    # ------------------- 1. 初始化和连接 (一次性) -------------------
    api_key = os.environ.get("DASHSCOPE_API_KEY")
    if not api_key:
        print("请设置DASHSCOPE_API_KEY环境变量")
        return

    print("--- 实时多轮音视频对话客户端 ---")
    print("正在初始化音频播放器和客户端...")
    
    audio_player = AudioPlayer()
    audio_player.start()

    def on_audio_received(audio_data):
        audio_player.add_audio_data(audio_data)

    def on_response_done(event):
        print("\n(收到响应结束标记)")
        audio_player.stop_receiving_data()

    realtime_client = OmniRealtimeClient(
        base_url="wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        api_key=api_key,
        model="qwen-omni-turbo-realtime",
        voice="Serena",
        on_text_delta=lambda text: print(f"助手回复: {text}", end="", flush=True),
        on_audio_delta=on_audio_received,
        turn_detection_mode=TurnDetectionMode.MANUAL,
        extra_event_handlers={"response.done": on_response_done}
    )

    message_handler_task = None
    try:
        await realtime_client.connect()
        print("已连接到服务器。输入 'q' 或 'quit' 可随时退出程序。")
        message_handler_task = asyncio.create_task(realtime_client.handle_messages())
        await asyncio.sleep(0.5)

        turn_counter = 1
        # ------------------- 2. 多轮对话循环 -------------------
        while True:
            print(f"\n--- 第 {turn_counter} 轮对话 ---")
            audio_player.prepare_for_next_turn()

            recorded_audio = None
            image_paths = []
            
            # --- 获取用户输入：从麦克风录音 ---
            loop = asyncio.get_event_loop()
            recorder = MicrophoneRecorder(sample_rate=16000) # 推荐使用16k采样率进行语音识别

            print("准备录音。按 Enter 键开始录音 (或输入 'q' 退出)...")
            user_input = await loop.run_in_executor(None, input)
            if user_input.strip().lower() in ['q', 'quit']:
                print("用户请求退出...")
                return
            
            try:
                recorder.start()
            except Exception:
                print("无法启动录音，请检查您的麦克风权限和设备。跳过本轮。")
                continue
            
            print("录音中... 再次按 Enter 键停止录音。")
            await loop.run_in_executor(None, input)

            recorded_audio = recorder.stop()

            if not recorded_audio or len(recorded_audio) == 0:
                print("未录制到有效音频，请重新开始本轮对话。")
                continue

            # --- 获取图片输入 (可选) ---
            # 以下图片输入功能已被注释，暂时禁用。若需启用请取消下方代码注释。
            # print("\n请逐行输入【图片文件】的绝对路径 (可选)。完成后，输入 's' 或按 Enter 发送请求。")
            # while True:
            #     path = input("图片路径: ").strip()
            #     if path.lower() == 's' or path == '':
            #         break
            #     if path.lower() in ['q', 'quit']:
            #         print("用户请求退出...")
            #         return
            #     
            #     if not os.path.isabs(path):
            #         print("错误: 请输入绝对路径。")
            #         continue
            #     if not os.path.exists(path):
            #         print(f"错误: 文件不存在 -> {path}")
            #         continue
            #     image_paths.append(path)
            #     print(f"已添加图片: {os.path.basename(path)}")
            
            # --- 3. 发送数据并获取响应 ---
            print("\n--- 输入确认 ---")
            print(f"待处理音频: 1个 (来自麦克风), 图片: {len(image_paths)}个")
            print("------------------")

            # 3.1 发送录制的音频
            #try:
            #    print(f"发送麦克风录音 ({len(recorded_audio)}字节)")
            #    await realtime_client.stream_audio(recorded_audio)
            #    await asyncio.sleep(0.1)
            #except Exception as e:
            #    print(f"发送麦克风录音失败: {e}")
            #    continue
            try:
                chunk_size = 250 * 1024  # 250KB (小于256KB限制)
                total_audio = len(recorded_audio)
                print(f"发送麦克风录音 ({total_audio}字节)，分块发送...")
                
                for i in range(0, total_audio, chunk_size):
                    chunk = recorded_audio[i:i+chunk_size]
                    print(f"发送音频块 [{i}-{min(i+chunk_size, total_audio)}]")
                    await realtime_client.stream_audio(chunk)
                    await asyncio.sleep(0.05)  # 添加短暂延迟避免拥塞
                
                print("音频发送完成")
            except Exception as e:
                print(f"发送麦克风录音失败: {e}")
                continue

            # 3.2 发送所有图片文件
            # 以下图片发送代码已被注释，暂时禁用。
            # for i, path in enumerate(image_paths):
            #     try:
            #         with open(path, "rb") as f:
            #             data = f.read()
            #         print(f"发送图片 {i+1}: {os.path.basename(path)} ({len(data)}字节)")
            #         await realtime_client.append_image(data)
            #         await asyncio.sleep(0.1)
            #     except Exception as e:
            #         print(f"发送图片 {os.path.basename(path)} 失败: {e}")

            # 3.3 提交并等待响应
            print("提交所有输入，请求服务器响应...")
            await realtime_client.commit_audio_buffer()
            await realtime_client.create_response()

            print("等待并播放服务器响应音频...")
            start_time = time.time()
            max_wait_time = 60
            while not audio_player.is_finished_playing():
                if time.time() - start_time > max_wait_time:
                    print(f"\n等待超时 ({max_wait_time}秒), 进入下一轮。")
                    break
                await asyncio.sleep(0.2)
            
            print("\n本轮音频播放完成！")
            turn_counter += 1

    except (asyncio.CancelledError, KeyboardInterrupt):
        print("\n程序被中断。")
    except Exception as e:
        print(f"发生未处理的错误: {e}")
    finally:
        # ------------------- 4. 清理资源 -------------------
        print("\n正在关闭连接并清理资源...")
        if message_handler_task and not message_handler_task.done():
            message_handler_task.cancel()
        
        if 'realtime_client' in locals() and realtime_client.ws and not realtime_client.ws.close:
            await realtime_client.close()
            print("连接已关闭。")

        audio_player.stop()
        print("程序退出。")

if __name__ == "__main__":
    try:
        asyncio.run(interactive_test())
    except KeyboardInterrupt:
        print("\n程序被用户强制退出。") 