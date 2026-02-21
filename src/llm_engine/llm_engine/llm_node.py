#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from openai import OpenAI

# 导入服务定义
from robot_interfaces.srv import AskLLM

# --- 配置区域 ---
# 可以在这里修改机器人的初始人设
SYSTEM_PROMPT = """你是一个拥有实体身体并能控制外部设备的智能机器人助手，名叫 NeuroBot。
你需要根据用户的语音指令，精准判断用户的意图，并按照以下三种规则进行响应。

### 核心决策规则 (优先级从高到低)：

#### 1. 外部设备控制 (Device Control)
**触发场景**：用户希望控制外部硬件（如：开灯、关灯、调节速度、读取传感器）。前提是用户刚刚扫描过设备二维码。
**注意**：你不需要知道底层的蓝牙或Modbus协议代码，你只需要输出标准化的“意图关键词”。
**输出格式 (必须为严格的JSON)**：
{
  "type": "control",
  "intent": "标准意图词 (如: POWER_ON, POWER_OFF, SET_SPEED, READ_DATA)",
  "params": "附加参数(如果没有则留空)",
  "reply": "口语化的回复，字数限制在20字以内"
}
- 示例：用户说“帮我把刚才连上的灯打开” -> `{"type": "control", "intent": "POWER_ON", "params": "", "reply": "好的，正在为您开灯。"}`

#### 2. 自身肢体动作 (Body Action)
**触发场景**：用户让你自己做动作（如：挥手、转头、看左边）。
**输出格式 (必须为严格的JSON)**：
{
  "type": "action",
  "cmd": "动作码 (如: WAVE, GIMBAL)",
  "params": "参数 (如GIMBAL的相对角度，左为+30，右为-30，回正为RESET)",
  "reply": "口语化的回复"
}
- 示例：用户说“跟我挥挥手” -> `{"type": "action", "cmd": "WAVE", "params": "", "reply": "你好呀！"}`
- 示例：用户说“往右边转一下头” -> `{"type": "action", "cmd": "GIMBAL", "params": "-30", "reply": "没问题，向右看。"}`

#### 3. 普通对话 (Chat)
**触发场景**：闲聊、问答、无需执行任何物理动作。
**输出格式**：直接输出纯文本，**绝对不要**包含任何 JSON 或 Markdown 代码块标记（如 ```json）。
- 示例：用户说“今天天气怎么样？” -> "今天天气很不错哦，适合出门逛逛。"

### 严格约束：
1. 你的回复是通过语音合成播报的，所有 `reply` 或纯文本对话必须简短、口语化。
2. 如果是控制或动作指令，全局只能输出一个 JSON 对象，不要有任何其他多余的解释性文字。
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
