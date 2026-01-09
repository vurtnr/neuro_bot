#!/usr/bin/env python3
import sys
import os

venv_lib_path = "/home/ubuntu/neuro_bot_env/lib/python3.12/site-packages"
if os.path.exists(venv_lib_path) and venv_lib_path not in sys.path:
    sys.path.insert(0, venv_lib_path)
    print(f"✅ 已强制加载虚拟环境库: {venv_lib_path}")


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

# 引入我们刚才定义的接口
from robot_interfaces.msg import AudioSpeech, RobotState
from robot_interfaces.srv import AskLLM

# 引入之前的 AI 库
import sherpa_ncnn
import sherpa_onnx
import wave
import numpy as np
import soundfile as sf
import os
import subprocess
from openai import OpenAI
import threading
import time

# ================= 配置 (与之前保持一致) =================
API_KEY = "sk-6e4321b5364d4883aa8224d49ca58ae1" # <--- ⚠️ 记得填你的 Key
BASE_URL = "https://api.deepseek.com/v1"
LLM_MODEL = "deepseek-chat"
STT_MODEL_DIR = "/home/ubuntu/sherpa_model/sherpa-ncnn-streaming-zipformer-zh-14M-2023-02-23"
TTS_MODEL_DIR = "/home/ubuntu/sherpa_tts_model/vits-zh-aishell3"

class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        self.get_logger().info("🎤 Audio Engine 正在启动...")

        # 1. 初始化 AI 模型 (STT / TTS / LLM)
        self.init_models()

        # 2. 通信接口
        # [Pub] 告诉大脑我听到了什么
        self.speech_pub = self.create_publisher(AudioSpeech, '/speech/text', 10)
        
        # [Sub] 听大脑指挥：让我说话
        self.tts_sub = self.create_subscription(String, '/mouth/say', self.speak_callback, 10)
        
        # [Service] 借大脑算力：LLM 问答
        self.llm_service = self.create_service(AskLLM, '/brain/ask_llm', self.handle_ask_llm)

        # 3. 开启录音线程 (简化版：循环检测音频文件，未来可接麦克风流)
        # 这里为了演示，我们暂时不做无限循环录音，而是等待外部指令或手动触发
        # 实际部署时，这里会是一个 VAD 循环
        self.get_logger().info("✅ Audio Engine 就绪！等待 Rust 大脑指令...")

    def init_models(self):
        # --- STT ---
        try:
            self.recognizer = sherpa_ncnn.Recognizer(
                tokens=f"{STT_MODEL_DIR}/tokens.txt",
                encoder_param=f"{STT_MODEL_DIR}/encoder_jit_trace-pnnx.ncnn.param",
                encoder_bin=f"{STT_MODEL_DIR}/encoder_jit_trace-pnnx.ncnn.bin",
                decoder_param=f"{STT_MODEL_DIR}/decoder_jit_trace-pnnx.ncnn.param",
                decoder_bin=f"{STT_MODEL_DIR}/decoder_jit_trace-pnnx.ncnn.bin",
                joiner_param=f"{STT_MODEL_DIR}/joiner_jit_trace-pnnx.ncnn.param",
                joiner_bin=f"{STT_MODEL_DIR}/joiner_jit_trace-pnnx.ncnn.bin",
                num_threads=4,
            )
        except Exception as e:
            self.get_logger().error(f"STT 加载失败: {e}")

        # --- TTS ---
        try:
            self.tts_engine = sherpa_onnx.OfflineTts(
                sherpa_onnx.OfflineTtsConfig(
                    model=sherpa_onnx.OfflineTtsModelConfig(
                        vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                            model=f"{TTS_MODEL_DIR}/vits-aishell3.onnx",
                            lexicon=f"{TTS_MODEL_DIR}/lexicon.txt",
                            tokens=f"{TTS_MODEL_DIR}/tokens.txt",
                        ),
                        provider="cpu", num_threads=4, debug=False
                    )
                )
            )
        except Exception as e:
            self.get_logger().error(f"TTS 加载失败: {e}")

        # --- LLM ---
        self.client = OpenAI(api_key=API_KEY, base_url=BASE_URL)

    # === 回调函数 ===

    def speak_callback(self, msg):
        """收到 /mouth/say 消息时触发"""
        text = msg.data
        self.get_logger().info(f"👄 正在说话: {text}")
        
        # 简单的文本清洗
        safe_text = text.replace("NeuroBot", "纽罗波特").replace("AI", "人工智能")
        
        try:
            audio = self.tts_engine.generate(safe_text, sid=0, speed=1.1)
            sf.write("temp_speak.wav", audio.samples, audio.sample_rate)
            subprocess.run(["aplay", "-q", "temp_speak.wav"])
        except Exception as e:
            self.get_logger().error(f"TTS 生成失败: {e}")

    def handle_ask_llm(self, request, response):
        """收到 /brain/ask_llm 请求时触发"""
        question = request.question
        self.get_logger().info(f"🧠 大脑请求思考: {question}")
        
        try:
            res = self.client.chat.completions.create(
                model=LLM_MODEL,
                messages=[
                    {"role": "system", "content": "你是NeuroBot。请用简短中文回答。"},
                    {"role": "user", "content": question}
                ],
                stream=False
            )
            answer = res.choices[0].message.content
            response.answer = answer
            response.success = True
            self.get_logger().info(f"💡 思考完成: {answer}")
        except Exception as e:
            response.answer = f"思考出错: {str(e)}"
            response.success = False
            self.get_logger().error(f"LLM 调用失败: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()