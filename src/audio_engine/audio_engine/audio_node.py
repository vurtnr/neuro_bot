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
import subprocess

# === 🛠️ 导入官方协议库 ===
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
        print("❌ 严重错误: 找不到 protocols.py")
        raise

# === 🛠️ 配置区域 ===
VOLC_APPID = os.getenv("VOLC_APPID")
VOLC_TOKEN = os.getenv("VOLC_TOKEN")

VOLC_ASR_URL = "wss://openspeech.bytedance.com/api/v3/sauc/bigmodel_async"
ASR_RESOURCE_ID = os.getenv("VOLC_ASR_RESOURCE_ID", "volc.bigasr.sauc.duration")
VOLC_TTS_V3_URL = "wss://openspeech.bytedance.com/api/v3/tts/bidirection"

# 🟢 [TTS 配置]
TTS_RESOURCE_ID = "volc.service_type.10029" 
TTS_VOICE_TYPE = "zh_male_jingqiangkanye_moon_bigtts" 
TTS_SPEED_RATIO = 1.0              

# 🟢 [硬件配置] 播放设备 ID
PLAYBACK_DEVICE = "plughw:0,0"  

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
        self.get_logger().info(f"🎤 Audio Engine 24.0 (Buffer Flush Fix) Init...")

        if not VOLC_APPID or not VOLC_TOKEN:
            self.get_logger().error("❌ 未检测到 VOLC_APPID 或 VOLC_TOKEN 环境变量！")

        self.speech_pub = self.create_publisher(AudioSpeech, '/audio/speech', 10)
        self.tts_sub = self.create_subscription(String, '/audio/tts_play', self.handle_tts_command, 10)

        # 状态锁
        self.is_speaking = False
        self.audio_queue = asyncio.Queue()
        self.asr_needs_reset = asyncio.Event()

        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_loop, daemon=True)
        self._thread.start()

        self.loop.call_soon_threadsafe(self.start_asr_task)
        self.get_logger().info(f"✅ Ready. TTS: {TTS_VOICE_TYPE} -> Speaker: {PLAYBACK_DEVICE}")

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
    # 👄 TTS Pipeline
    # ==========================================
    async def run_tts_pipeline_v3(self, text):
        self.is_speaking = True
        self.asr_needs_reset.set() # 强制中断 ASR
        
        self.get_logger().info("🔒 Muting Mic for TTS...")

        if "VOLC_TTS_RESOURCE_ID" in os.environ: del os.environ["VOLC_TTS_RESOURCE_ID"]
        headers = { "X-Api-App-Key": VOLC_APPID, "X-Api-Access-Key": VOLC_TOKEN, "X-Api-Resource-Id": TTS_RESOURCE_ID, "X-Api-Connect-Id": str(uuid.uuid4()) }

        try:
            async with websockets.connect(VOLC_TTS_V3_URL, extra_headers=headers) as ws:
                # 1. Start Connection
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.StartConnection; msg.payload = b"{}"
                await ws.send(msg.marshal())
                await self.wait_for_event(ws, MsgType.FullServerResponse, EventType.ConnectionStarted)
                
                # 2. Start Session
                session_id = str(uuid.uuid4())
                req_json = { "user": {"uid": "neuro_bot_tts"}, "namespace": "BidirectionalTTS", "req_params": { "speaker": TTS_VOICE_TYPE, "audio_params": {"format": "wav", "sample_rate": 24000, "speed_ratio": TTS_SPEED_RATIO} } }
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.StartSession; msg.session_id = session_id; msg.payload = json.dumps(req_json).encode('utf-8')
                await ws.send(msg.marshal())
                await self.wait_for_event(ws, MsgType.FullServerResponse, EventType.SessionStarted)

                # 3. Task Request
                task_req = copy.deepcopy(req_json); task_req["req_params"]["text"] = text
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.TaskRequest; msg.session_id = session_id; msg.payload = json.dumps(task_req).encode('utf-8')
                await ws.send(msg.marshal())

                # 4. Finish Session
                msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.WithEvent)
                msg.event = EventType.FinishSession; msg.session_id = session_id; msg.payload = b"{}"
                await ws.send(msg.marshal())

                # 5. Receive & Play
                audio_buffer = bytearray()
                print("📥 Receiving Audio: ", end="", flush=True)
                while True:
                    raw_data = await asyncio.wait_for(ws.recv(), timeout=5.0)
                    msg = Message.from_bytes(raw_data)
                    if msg.type == MsgType.FullServerResponse and msg.event == EventType.SessionFinished:
                        print("\n✅ Session Finished."); break
                    elif msg.type == MsgType.AudioOnlyServer:
                        audio_buffer.extend(msg.payload); print(".", end="", flush=True)

                if audio_buffer:
                    filename = "/tmp/robot_tts_v3.wav"
                    with open(filename, "wb") as f: f.write(audio_buffer)
                    self.get_logger().info(f"▶️ Playing...")
                    subprocess.run(["aplay", "-D", PLAYBACK_DEVICE, "-q", filename])
                
        except Exception as e:
            self.get_logger().error(f"TTS Failed: {e}")
        finally:
            self.get_logger().info("🔓 Unmuting Mic...")
            self.is_speaking = False

    async def wait_for_event(self, ws, msg_type, event_type):
        while True:
            raw_data = await ws.recv()
            msg = Message.from_bytes(raw_data)
            if msg.type == msg_type and msg.event == event_type: return msg

    # ==========================================
    # 👂 ASR Pipeline (自动重连 + 缓冲区清洗)
    # ==========================================
    async def run_asr_pipeline(self):
        self.get_logger().info(f"👂 ASR Engine Started.")
        
        def callback(indata, frames, time, status): 
            self.loop.call_soon_threadsafe(self.audio_queue.put_nowait, bytes(indata))
        stream = sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=CHUNK_SIZE, dtype=DTYPE, channels=CHANNELS, callback=callback)
        stream.start()

        while rclpy.ok():
            # 1. 说话期间暂停
            if self.is_speaking:
                await asyncio.sleep(0.1)
                # 🟢 关键：说话期间产生的音频数据，直接丢弃，不要累积！
                while not self.audio_queue.empty():
                    try: self.audio_queue.get_nowait()
                    except: break
                continue

            # 2. 建立新连接前，彻底清空队列！
            # 这是解决“文字重复”的核心：倒掉上一句话的残渣
            dropped_packets = 0
            while not self.audio_queue.empty():
                try: 
                    self.audio_queue.get_nowait()
                    dropped_packets += 1
                except: break
            if dropped_packets > 0:
                self.get_logger().info(f"🧹 Flushed {dropped_packets} old audio packets before new session.")

            try:
                self.asr_needs_reset.clear()
                header = { "X-Api-App-Key": VOLC_APPID, "X-Api-Access-Key": VOLC_TOKEN, "X-Api-Resource-Id": ASR_RESOURCE_ID, "X-Api-Connect-Id": str(uuid.uuid4()) }
                
                async with websockets.connect(VOLC_ASR_URL, extra_headers=header, max_size=1000000000) as ws:
                    self.get_logger().info("✅ ASR Connected (New Session).")
                    
                    reqid = str(uuid.uuid4())
                    request_params = { "user": {"uid": "neuro_bot"}, "audio": {"format": "pcm", "rate": SAMPLE_RATE, "bits": 16, "channel": CHANNELS, "codec": "raw"}, "request": {"reqid": reqid, "model_name": "bigmodel", "enable_itn": True, "enable_punc": True, "result_type": "single", "sequence": 1} }
                    msg = Message(type=MsgType.FullClientRequest, flag=MsgTypeFlagBits.NoSeq)
                    msg.serialization = SerializationBits.JSON; msg.compression = CompressionBits.Gzip
                    msg.payload = gzip.compress(json.dumps(request_params).encode('utf-8'))
                    await ws.send(msg.marshal())
                    
                    send_task = asyncio.create_task(self.send_audio_loop(ws))
                    recv_task = asyncio.create_task(self.receive_asr_loop(ws))
                    
                    done, pending = await asyncio.wait([send_task, recv_task], return_when=asyncio.FIRST_COMPLETED)
                    for task in pending: task.cancel()
                    self.get_logger().info("🔄 ASR Session Ended.")

            except Exception as e:
                self.get_logger().error(f"ASR Error: {e}")
                await asyncio.sleep(2)

    async def send_audio_loop(self, ws):
        silence = b'\x00' * (CHUNK_SIZE * 2)
        hold_frames = int(SAMPLE_RATE / CHUNK_SIZE * SPEECH_HOLD_TIME)
        cur_hold = 0
        
        while True:
            if self.is_speaking or self.asr_needs_reset.is_set(): return

            data = await self.audio_queue.get()
            
            rms = np.sqrt(np.mean(np.frombuffer(data, dtype=np.int16).astype(np.float64)**2))
            is_loud = rms > NOISE_GATE_THRESHOLD
            
            if is_loud: cur_hold = hold_frames
            elif cur_hold > 0: cur_hold -= 1
            
            msg = Message(type=MsgType.AudioOnlyClient, flag=MsgTypeFlagBits.NoSeq)
            msg.compression = CompressionBits.Gzip
            msg.payload = gzip.compress(data if cur_hold > 0 else silence)
            await ws.send(msg.marshal())

    async def receive_asr_loop(self, ws):
        last_txt, last_t = "", time.time()
        while True:
            try:
                if self.is_speaking or self.asr_needs_reset.is_set(): return

                raw_data = await asyncio.wait_for(ws.recv(), timeout=0.1)
                msg = Message.from_bytes(raw_data)
                
                if msg.type == MsgType.FullServerResponse:
                    payload = gzip.decompress(msg.payload) if msg.compression == CompressionBits.Gzip else msg.payload
                    resp = json.loads(payload)
                    if 'result' in resp:
                        data = resp['result'][0] if isinstance(resp['result'], list) else resp['result']
                        txt = data.get('text', '')
                        final = data.get('utterances', [{}])[0].get('definite', False) if 'utterances' in data else False
                        
                        if txt: 
                            if txt != last_txt: last_txt, last_t = txt, time.time()
                            if final: 
                                self.publish_final(txt)
                                return # 收到 Final，退出重连
                            else: 
                                print(f"\r👂 Hearing: {txt}...", end="", flush=True)
            
            except asyncio.TimeoutError:
                if last_txt and (time.time() - last_t > FINAL_TIMEOUT): 
                    self.publish_final(last_txt)
                    return # 超时，退出重连
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