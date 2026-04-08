import os
import sys

# 【致命修复 1】：强制 ROS 2 把所有底层 C++ 日志输出到 stderr (错误流)
# 绝对禁止它污染 MCP 正在使用的 stdout (标准输出 JSON 流)
os.environ['RCUTILS_LOGGING_USE_STDERR'] = '1'

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from mcp.server.fastmcp import FastMCP
import threading

class PandaFullControl(Node):
    def __init__(self):
        super().__init__('mcp_panda_control_node')
        
        # 【致命修复 2】：MoveIt2 的标准 Action 名字其实是 'move_action'
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 发布可视化坐标 (RViz 插眼)
        self.goal_pub = self.create_publisher(PoseStamped, '/llm_target_pose', 10)
        
        # 【致命修复 3】：绝对不能在 __init__ 里死循环 wait_for_server，会导致永远无法 spin！
        self.get_logger().info("🚀 ROS 2 控制节点已挂载后台，等待 MCP 指令接入...")

    def send_move_goal(self, x: float, y: float, z: float):
        # 每次执行前瞬间检查一次服务是否在线，不在线直接报错返回，绝不死锁
        if not self._action_client.server_is_ready():
            self.get_logger().error("找不到 MoveIt2 服务，请确认 demo.launch.py 是否正在运行！")
            return False

        # --- 发布可视化坐标 ---
        viz_msg = PoseStamped()
        viz_msg.header.frame_id = "panda_link0"
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.pose.position.x = float(x)
        viz_msg.pose.position.y = float(y)
        viz_msg.pose.position.z = float(z)
        viz_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(viz_msg)

        # --- 构建 Action Goal ---
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # --- 设置目标位置约束 ---
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "panda_link0"
        pos_constraint.link_name = "panda_link8" 
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] 
        
        pos_constraint.constraint_region.primitives = [s]
        pos_constraint.constraint_region.primitive_poses = [viz_msg.pose]
        pos_constraint.weight = 1.0

        goal_msg.request.goal_constraints = [Constraints(position_constraints=[pos_constraint])]

        # 发送异步任务
        self._action_client.send_goal_async(goal_msg)
        return True

# ================= MCP 服务端逻辑 =================
mcp = FastMCP("Panda_Final_Boss")
ros_node = None

@mcp.tool()
def execute_arm_move(x: float, y: float, z: float) -> str:
    """
    让机械臂真实移动到目标三维坐标 (X, Y, Z)。
    范围检查：X[0.2~0.6], Y[-0.4~0.4], Z[0.2~0.7]
    """
    global ros_node
    
    if ros_node is None:
        return "错误：系统故障，ROS 2 节点未初始化。"

    if not (0.2 <= x <= 0.6 and -0.4 <= y <= 0.4 and 0.2 <= z <= 0.7):
        return "错误：目标坐标超出物理安全范围！请重新计算。"

    success = ros_node.send_move_goal(x, y, z)
    if success:
        return f"[成功] 运动请求已下发至 MoveIt2！机械臂正在开往坐标: X:{x}, Y:{y}, Z:{z}"
    else:
        return "[失败] 无法连接到底层运动规划器，请确认 RViz 环境是否启动。"

# ================= 守护线程与启动 =================
def spin_ros():
    global ros_node
    rclpy.init()
    ros_node = PandaFullControl()
    # 让 ROS 2 节点在独立线程里永动，处理收发消息
    rclpy.spin(ros_node)

if __name__ == "__main__":
    t = threading.Thread(target=spin_ros, daemon=True)
    t.start()
    
    mcp.run()