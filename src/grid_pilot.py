import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import math

class GridPilot(Node):
    def __init__(self):
        super().__init__('grid_pilot')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_ctrl_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)

        # 30x30 meter Lawnmower Grid
        grid_width = 30.0      
        grid_length = 30.0     
        row_spacing = 10.0     
        search_altitude = -5.0 

        self.waypoints = [(0.0, 0.0, search_altitude)] 
        
        current_y = 0.0
        moving_forward = True  

        while current_y <= grid_length:
            target_x = grid_width if moving_forward else 0.0
            self.waypoints.append((target_x, current_y, search_altitude))
            
            current_y += row_spacing
            if current_y > grid_length:
                break 
                
            self.waypoints.append((target_x, current_y, search_altitude))
            moving_forward = not moving_forward

        self.waypoints.append((0.0, 0.0, search_altitude))

        self.current_waypoint_idx = 0
        self.timer_count = 0

    def timer_callback(self):
        # 1. Offboard Heartbeat
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_mode_pub.publish(msg)

        # 2. Grab Target Coordinates
        target_x, target_y, target_z = self.waypoints[self.current_waypoint_idx]

        # 3. Trajectory Command (WITH NaN VELOCITY FIX)
        setpoint = TrajectorySetpoint()
        setpoint.position = [float(target_x), float(target_y), float(target_z)]
        setpoint.yaw = 0.0 
        
        # Explicitly tell PX4 to ignore velocity and acceleration constraints
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(setpoint)

        # 4. Delay Arming for 3 Seconds to build a steady data stream
        if self.timer_count == 30:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Motors Armed! Executing Search Pattern...")

        # 5. Move to the next waypoint every 10 seconds
        if self.timer_count > 30 and self.timer_count % 100 == 0:
            if self.current_waypoint_idx < len(self.waypoints) - 1:
                self.current_waypoint_idx += 1
                self.get_logger().info(f"Navigating to Waypoint {self.current_waypoint_idx}: {self.waypoints[self.current_waypoint_idx]}")
            else:
                self.get_logger().info("Search Pattern Complete. Holding position at Home.")

        self.timer_count += 1

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    grid_pilot = GridPilot()
    rclpy.spin(grid_pilot)
    grid_pilot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
