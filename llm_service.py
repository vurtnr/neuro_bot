#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from openai import OpenAI

# 导入服务定义
from robot_interfaces.srv import AskLLM

# --- 配置区域 ---
# 可以在这里修改机器人的初始人设
SYSTEM_PROMPT = """
你是一个运行在嵌入式设备上的智能机器人助手。
请用简短、口语化、有些俏皮的风格回答用户的提问。
因为你是用语音播报的，回答不要太长（控制在50字以内），不要使用复杂的Markdown格式。
"""
# ----------------

class LLMEngine(Node):
    def __init__(self):
        super().__init__('llm_engine')
        
        # 1. 获取 API Key (从环境变量)
        self.api_key = os.getenv("DEEPSEEK_API_KEY")
        if not self.api_key:
            self.get_logger().error("❌ 未找到 DEEPSEEK_API_KEY 环境变量！请 export DEEPSEEK_API_KEY='sk-...'")
            # 这里不退出，只是报错，方便调试
        
        # 2. 初始化 OpenAI 客户端 (适配 DeepSeek)
        # DeepSeek 的 base_url 是 https://api.deepseek.com
        if self.api_key:
            self.client = OpenAI(api_key=self.api_key, base_url="https://api.deepseek.com/v1")
            self.get_logger().info("✅ 已连接 DeepSeek API")
        else:
            self.client = None

        # 3. 创建服务
        self.srv = self.create_service(AskLLM, '/brain/ask_llm', self.handle_llm_request)
        self.get_logger().info("🤖 LLM Engine READY! Service: /brain/ask_llm")

    def handle_llm_request(self, request, response):
        question = request.question
        self.get_logger().info(f"📥 收到问题: \"{question}\"")
        
        # 检查是否有 Key
        if not self.client:
            response.answer = "我的大脑还没有配置 API Key，请检查环境变量。"
            response.success = False
            return response

        try:
            self.get_logger().info("🤔 正在请求 DeepSeek...")
            
            # --- 真正的 API 调用 ---
            completion = self.client.chat.completions.create(
                model="deepseek-chat",  # 或者 "deepseek-coder"
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": question}
                ],
                stream=False,
                temperature=0.7, # 稍微有创造力一点
                max_tokens=100   # 限制长度，防止说太多话
            )
            
            # 提取回答
            real_answer = completion.choices[0].message.content.strip()
            
            self.get_logger().info(f"💡 DeepSeek 回复: \"{real_answer}\"")
            
            response.answer = real_answer
            response.success = True
            
        except Exception as e:
            error_msg = f"API 调用失败: {str(e)}"
            self.get_logger().error(error_msg)
            response.answer = "我好像断网了，无法连接到云端大脑。"
            response.success = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LLMEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()