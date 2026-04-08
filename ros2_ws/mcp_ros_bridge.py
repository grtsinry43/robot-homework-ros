import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mcp.server.fastmcp import FastMCP
import threading

# ==========================================
# 1. ROS 2 节点部分：负责和底层机器人/仿真器通信
# ==========================================
class RobotMcpBridge(Node):
    def __init__(self):
        super().__init__('mcp_bridge_node')
        # 创建一个发布者，往 /cmd_vel 话题发送 Twist (速度) 消息
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.get_logger().info("✅ ROS 2 节点已就绪，等待 LLM 召唤...")

    def set_velocity(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher_.publish(msg)
        return f"执行成功: 已下发线速度 {linear_x} m/s, 角速度 {angular_z} rad/s"

# 初始化 ROS 2 环境并实例化节点
rclpy.init()
ros_node = RobotMcpBridge()

# ==========================================
# 2. MCP 服务端部分：负责和大语言模型 (LLM) 通信
# ==========================================
# 创建一个 FastMCP 实例，这个名字会显示给 LLM 看
mcp = FastMCP("ROS2_Commander")

@mcp.tool()
def command_velocity(linear_x: float, angular_z: float) -> str:
    """
    控制机器人的移动速度。
    当用户要求机器人前进、后退、转弯或停止时，请调用此工具。
    参数：
    - linear_x: 前进的线速度 (米/秒)，正数为前进，负数为后退，0为停止。
    - angular_z: 旋转的角速度 (弧度/秒)，正数为原地左转，负数为原地右转，0为不转。
    """
    # 打印日志方便你在终端观察大模型的调用情况
    ros_node.get_logger().info(f"🤖 LLM 触发了工具 -> 线速度: {linear_x}, 角速度: {angular_z}")
    # 调用 ROS 节点的方法发布消息
    return ros_node.set_velocity(linear_x, angular_z)

# ==========================================
# 3. 线程隔离与启动
# ==========================================
# 把 ROS 2 的事件循环丢到后台守护线程，防止阻塞 MCP 的标准输入输出
def spin_ros():
    rclpy.spin(ros_node)

ros_thread = threading.Thread(target=spin_ros, daemon=True)
ros_thread.start()

if __name__ == "__main__":
    # 启动 MCP Server (默认通过 stdio 标准输入输出与大模型客户端通信)
    mcp.run()