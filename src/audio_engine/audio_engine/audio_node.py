#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import AudioSpeech

import os
import json
import uuid
import asyncio
import websockets
import threading
import sounddevice as sd
import sys
import logging
import numpy as np
import gzip
import time
import struct
import copy

# === 🛠️ 导入官方协议库 ===
# 确保 protocols.py 在同级目录下
try:
    from audio_engine.protocols import (
        Message, MsgType, MsgTypeFlagBits, EventType, 
        SerializationBits, CompressionBits, VersionBits, HeaderSizeBits
    )
except ImportError:
    try:
        from protocols import (
            Message, MsgType, MsgTypeFlagBits, EventType, 
            SerializationBits, CompressionBits, VersionBits, HeaderSizeBits
        )
    except ImportError:
        print("❌ 严重错误: 找不到 protocols.py，请确保它与 audio_node.py 在同一目录")
        raise

# === 🛠️ 配置区域 ===
VOLC_APPID = os.getenv("VOLC_APPID")
VOLC_TOKEN = os.getenv("VOLC_TOKEN")

# [ASR 配置]
VOLC_ASR_URL = "wss://openspeech.bytedance.com/api/v3/sauc/bigmodel_async"
ASR_RESOURCE_ID = os.getenv("VOLC_ASR_RESOURCE_ID", "volc.bigasr.sauc.duration")

# [TTS 配置 - V3 双向流式]
VOLC_TTS_V3_URL = "wss://openspeech.bytedance.com/api/v3/tts/bidirection"

# 使用标准版 ID (10029) 确保兼容性
TTS_RESOURCE_ID = "volc.service_type.10029"
TTS_VOICE_TYPE = "BV001_streaming" # 通用女声
TTS_SPEED_RATIO = 1.0              

# 🎛️ 噪声门配置
NOISE_GATE_THRESHOLD = 1200    
SPEECH_HOLD_TIME = 1.0         
FINAL_TIMEOUT = 1.5 
SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = 'int16'
CHUNK_SIZE = 1024 

class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        self.get_logger().info(f"🎤 Audio Engine 18.0 (Official Protocols) Init...")

        if not VOLC_APPID or not VOLC_TOKEN:
            self.get_logger().error("❌ 未检测到 VOLC_APPID 或 VOLC_TOKEN 环境变量！")

        self.speech_pub = self.create_publisher(AudioSpeech, '/audio/speech', 10)
        self.tts_sub = self.create_subscription(String, '/audio/tts_play', self.handle_tts_command, 10)

        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_loop, daemon=True)
        self._thread.start()

        self.loop.call_soon_threadsafe(self.start_asr_task)
        self.get_logger().info(f"✅ Ready. TTS: {TTS_VOICE_TYPE} on {TTS_RESOURCE_ID}")

    def _start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def start_asr_task(self):
        self.loop.create_task(self.run_asr_pipeline())

    def handle_tts_command(self, msg):
        text = msg.data
        self.get_logger().info(f"👄 TTS Request: {text}")
        asyncio.run_coroutine_threadsafe(self.run_tts_pipeline_v3(text), self.loop)

    # ==========================================
    # 👄 TTS Pipeline (基于官方 bidirection.py 重构)
    # ==========================================
    async def run_tts_pipeline_v3(self, text):
        import subprocess
        # 清理环境变量
        if "VOLC_TTS_RESOURCE_ID" in os.environ: del os.environ["VOLC_TTS_RESOURCE_ID"]

        headers = {
            "X-Api-App-Key": VOLC_APPID,
            "X-Api-Access-Key": VOLC_TOKEN,
            "X-Api-Resource-Id": TTS_RESOURCE_ID,
            "X-Api-Connect-Id": str(uuid.uuid4())
        }

        try:
            self.get_logger().info(f"🔗 Connecting to TTS V3 ({TTS_RESOURCE_ID})...")
            async with websockets.connect(VOLC_TTS_V3_URL, extra_headers=headers, max_size=1000000000) as ws:
                
                # --- Step 1: Start Connection ---
                # 参考 protocols.py/start_connection
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.StartConnection
                msg.payload = b"{}"
                await ws.send(msg.marshal())
                
                # 等待 ConnectionStarted (50)
                await self.wait_for_event(ws, MsgType.FullServerResponse, EventType.ConnectionStarted)
                
                # --- Step 2: Start Session ---
                session_id = str(uuid.uuid4())
                
                # 构造请求参数 (参考 bidirection.py)
                req_json = {
                    "user": {"uid": "neuro_bot_tts"},
                    "namespace": "BidirectionalTTS",
                    "req_params": {
                        "speaker": TTS_VOICE_TYPE,
                        "audio_params": {
                            "format": "wav",
                            "sample_rate": 24000,
                            "speed_ratio": TTS_SPEED_RATIO
                        }
                    }
                }
                
                # 发送 StartSession (Event 100)
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.StartSession
                msg.session_id = session_id
                msg.payload = json.dumps(req_json).encode('utf-8')
                await ws.send(msg.marshal())
                
                # 等待 SessionStarted (150)
                await self.wait_for_event(ws, MsgType.FullServerResponse, EventType.SessionStarted)

                # --- Step 3: Task Request (发送文本) ---
                # 官方例子是流式发字符，我们这里直接发整句 (TaskRequest 200)
                # 复用 req_json 结构，增加 text 字段
                task_req = copy.deepcopy(req_json)
                task_req["req_params"]["text"] = text
                
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.TaskRequest
                msg.session_id = session_id
                msg.payload = json.dumps(task_req).encode('utf-8')
                await ws.send(msg.marshal())

                # --- Step 4: Receive Audio Loop ---
                audio_buffer = bytearray()
                while True:
                    # 使用官方 Message 类解析
                    raw_data = await ws.recv()
                    msg = Message.from_bytes(raw_data)
                    
                    if msg.type == MsgType.FullServerResponse:
                        # 检查会话结束
                        if msg.event == EventType.SessionFinished:
                            self.get_logger().info("✅ TTS Session Finished.")
                            break
                        # 检查错误
                        if msg.event in [EventType.ConnectionFailed, EventType.SessionFailed]:
                             raise RuntimeError(f"Server Error Event {msg.event}: {msg.payload.decode('utf-8', 'ignore')}")

                    elif msg.type == MsgType.AudioOnlyServer:
                        # 接收音频
                        audio_buffer.extend(msg.payload)
                        
                    elif msg.type == MsgType.Error:
                        raise RuntimeError(f"Protocol Error {msg.error_code}: {msg.payload.decode('utf-8', 'ignore')}")

                # --- Step 5: Play Audio ---
                if audio_buffer:
                    filename = "/tmp/robot_tts_v3.wav"
                    with open(filename, "wb") as f: f.write(audio_buffer)
                    subprocess.run(["aplay", "-q", filename])
                    
        except Exception as e:
            self.get_logger().error(f"TTS Failed: {e}")

    # 辅助函数：等待特定事件
    async def wait_for_event(self, ws, msg_type, event_type):
        while True:
            raw_data = await ws.recv()
            msg = Message.from_bytes(raw_data)
            
            if msg.type == MsgType.Error:
                 raise RuntimeError(f"Wait Event Error: {msg.payload.decode('utf-8', 'ignore')}")
            
            if msg.type == msg_type and msg.event == event_type:
                return msg
            # 忽略其他不相关的包（如 AudioMuted 等）

    # ... (ASR 代码保持 V3 不变，这里使用 Message 类进行微调以保持一致性) ...
    async def run_asr_pipeline(self):
        # 注意：这里我们尝试复用 Message 类，或者保持您原本能用的 ASR 逻辑
        # 为了稳妥，建议您保持 ASR 部分不动。但既然引入了 protocols.py，
        # 下面展示如何用 protocols.py 来发 ASR 包 (如果不放心可以替换回您之前的 ASR 代码)
        self.get_logger().info(f"👂 Connecting to ASR V3...")
        while rclpy.ok():
            try:
                header = { "X-Api-App-Key": VOLC_APPID, "X-Api-Access-Key": VOLC_TOKEN, "X-Api-Resource-Id": ASR_RESOURCE_ID, "X-Api-Connect-Id": str(uuid.uuid4()) }
                async with websockets.connect(VOLC_ASR_URL, extra_headers=header, max_size=1000000000) as ws:
                    self.get_logger().info("✅ ASR Connected. Speak now!")
                    reqid = str(uuid.uuid4())
                    
                    # 构造 ASR 请求 (FullClientRequest, NoSeq)
                    request_params = { 
                        "user": {"uid": "neuro_bot"}, 
                        "audio": {"format": "pcm", "rate": SAMPLE_RATE, "bits": 16, "channel": CHANNELS, "codec": "raw"}, 
                        "request": {"reqid": reqid, "model_name": "bigmodel", "enable_itn": True, "enable_punc": True, "result_type": "single", "sequence": 1} 
                    }
                    
                    msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.NoSeq)
                    msg.serialization = SerializationBits.JSON
                    msg.compression = CompressionBits.Gzip
                    msg.payload = gzip.compress(json.dumps(request_params).encode('utf-8'))
                    await ws.send(msg.marshal())
                    
                    audio_queue = asyncio.Queue()
                    def callback(indata, frames, time, status): self.loop.call_soon_threadsafe(audio_queue.put_nowait, bytes(indata))
                    stream = sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=CHUNK_SIZE, dtype=DTYPE, channels=CHANNELS, callback=callback)
                    with stream:
                        asyncio.create_task(self.send_audio_loop_smart(ws, audio_queue))
                        await self.receive_asr_loop(ws)
            except Exception as e: self.get_logger().error(f"ASR Error: {e}"); await asyncio.sleep(2)

    async def send_audio_loop_smart(self, ws, queue):
        silence = b'\x00' * (CHUNK_SIZE * 2); hold = int(16000/1024*SPEECH_HOLD_TIME); cur = 0
        while True:
            data = await queue.get()
            if np.sqrt(np.mean(np.frombuffer(data, dtype=np.int16).astype(np.float64)**2)) > NOISE_GATE_THRESHOLD: cur = hold
            elif cur > 0: cur -= 1
            
            # 使用官方 Message 类发送音频
            msg = Message(type=MsgType.AudioOnlyClient, flag=MsgTypeFlagBits.NoSeq)
            msg.compression = CompressionBits.Gzip
            msg.payload = gzip.compress(data if cur > 0 else silence)
            await ws.send(msg.marshal())

    async def receive_asr_loop(self, ws):
        last_txt, last_t = "", time.time()
        while True:
            try:
                # 接收并使用官方类解析
                raw_data = await asyncio.wait_for(ws.recv(), timeout=0.1)
                msg = Message.from_bytes(raw_data)
                
                if msg.type == MsgType.FullServerResponse:
                    payload = gzip.decompress(msg.payload) if msg.compression == CompressionBits.Gzip else msg.payload
                    resp = json.loads(payload)
                    if 'result' in resp:
                        data = resp['result'][0] if isinstance(resp['result'], list) else resp['result']
                        txt = data.get('text', ''); final = data.get('utterances', [{}])[0].get('definite', False) if 'utterances' in data else False
                        if txt: 
                            if txt != last_txt: last_txt, last_t = txt, time.time()
                            if final: self.publish_final(txt); last_txt = ""
                            else: print(f"\r👂 Hearing: {txt}...", end="", flush=True)
            except asyncio.TimeoutError:
                if last_txt and (time.time() - last_t > FINAL_TIMEOUT): self.publish_final(last_txt); last_txt = ""
            except Exception: pass

    def publish_final(self, text):
        self.get_logger().info(f"\n🗣️ Final: {text}")
        msg = AudioSpeech(); msg.text = text; msg.confidence = 0.99; msg.is_final = True
        self.speech_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()