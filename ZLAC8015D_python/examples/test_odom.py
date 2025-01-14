import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
import math

class OdometryNode(Node):

    

    def encoder_callback(self, msg):
        # 从编码器数据中提取信息
        delta_left = msg.left_encoder_delta
        delta_right = msg.right_encoder_delta
        
        # 假设轮子间距和轮子半径已知
        wheel_base = 0.5
        wheel_radius = 0.1
        
        # 计算里程计
        delta_s = wheel_radius * (delta_right + delta_left) / 2.0
        delta_theta = wheel_radius * (delta_right - delta_left) / wheel_base
        
        # 更新机器人位置和姿态
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # 发布里程计信息
        current_time = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        
        # 设置位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q
        
        # 设置速度（如果有必要，可以计算速度）
        odom_msg.child_frame_id = 'base_link'
        delta_time = (current_time - self.last_time).nanoseconds * 1e-9
        odom_msg.twist.twist.linear.x = delta_s / delta_time
        odom_msg.twist.twist.angular.z = delta_theta / delta_time
        
        # 发布里程计消息
        self.odom_publisher.publish(odom_msg)
        
        # 发布TF变换
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)
        
        # 更新最后更新时间
        self.last_time = current_time
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
