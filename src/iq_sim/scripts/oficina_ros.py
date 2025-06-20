#!/usr/bin/env python3
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class MAVROSDrone:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('mavros_local_simple', anonymous=True)

        # Publisher: LOCAL_NED setpoints (x, y, z in meters)
        self.local_pub = rospy.Publisher(
            '/mavros/setpoint_position/local',
            PoseStamped,
            queue_size=10
        )  # :contentReference[oaicite:0]{index=0}

        # Subscriber: current local pose for feedback
        self.current_pose = None
        rospy.Subscriber(
            '/mavros/local_position/odom',
            Odometry,
            self._odom_cb
        )  # :contentReference[oaicite:1]{index=1}

        # Subscriber: flight state (to check mode/armed)
        self.state = None
        rospy.Subscriber(
            '/mavros/state',
            State,
            self._state_cb
        )

        # Wait for ARM and MODE services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def _odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def _state_cb(self, msg: State):
        self.state = msg

    def arm(self, arm: bool = True) -> bool:
        """Arm or disarm the vehicle."""
        res = self.arm_srv(arm)
        rospy.loginfo(f"Armed={arm}: success={res.success}")
        return res.success

    def set_mode(self, mode: str) -> bool:
        """Set flight mode (e.g., OFFBOARD, AUTO.LAND)."""
        res = self.mode_srv(0, mode)
        rospy.loginfo(f"SetMode to {mode}: sent={res.mode_sent}")
        return res.mode_sent

    def send_local_target(self, x: float, y: float, z: float):
        """Publish a LOCAL_NED target setpoint."""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # No orientation change
        msg.pose.orientation.w = 1.0
        self.local_pub.publish(msg)

    def wait_position(self, x: float, y: float, z: float,
                      tol: float = 0.3, timeout: float = 30.0) -> bool:
        """Block until current_pose is within tol (meters) or timeout."""
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rate.sleep()
                continue
            dx = abs(self.current_pose.position.x - x)
            dy = abs(self.current_pose.position.y - y)
            dz = abs(self.current_pose.position.z - z)
            if dx < tol and dy < tol and dz < tol:
                rospy.loginfo("Position reached")
                return True
            if time.time() - start > timeout:
                rospy.logwarn("Position wait timeout")
                return False
            rate.sleep()

if __name__ == "__main__":
    drone = MAVROSDrone()
    rospy.sleep(1)

    # Pre-fill setpoints before switching to OFFBOARD
    rate = rospy.Rate(20)
    for _ in range(100):
        drone.send_local_target(0, 0, 5.0)
        rate.sleep()

    # Switch to OFFBOARD mode and arm
    drone.set_mode("OFFBOARD")  # :contentReference[oaicite:2]{index=2}
    drone.arm(True)

    # Takeoff: fly to (0, 0, 5)
    drone.send_local_target(0, 0, 5.0)
    drone.wait_position(0, 0, 5.0)

    # Fly to another local waypoint, e.g., (5, 5, 5)
    rospy.loginfo("Flying to waypoint (5, 5, 5)")
    drone.send_local_target(5.0, 5.0, 5.0)
    drone.wait_position(5.0, 5.0, 5.0)

    rospy.sleep(2)

    # Land in place
    rospy.loginfo("Landing...")
    drone.set_mode("AUTO.LAND")  # :contentReference[oaicite:3]{index=3}
    # Optionally, keep publishing current position as setpoint until disarm
    while not rospy.is_shutdown() and drone.state and drone.state.armed:
        x = drone.current_pose.position.x
        y = drone.current_pose.position.y
        z = drone.current_pose.position.z
        drone.send_local_target(x, y, z)
        rate.sleep()

    rospy.loginfo("Disarmed, mission complete")

