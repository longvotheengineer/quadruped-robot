import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, String, Float64, Float32
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Point

class MockNode(Node):
    def __init__(self, name):
        super().__init__(name)

def main(args=None):
    rclpy.init(args=args)

    # --- MOCK SERIAL DRIVER A ---
    node_driver_a = MockNode('serial_driver_a')
    node_driver_a.create_subscription(Float64MultiArray, '/servo_commands_a', lambda msg: None, 10)
    node_driver_a.create_subscription(Float64MultiArray, '/servo_single_command_a', lambda msg: None, 10)
    node_driver_a.create_subscription(Bool, '/feedback_enable_a', lambda msg: None, 10)
    node_driver_a.create_publisher(JointState, '/joint_states_real_a', 10)

    # --- MOCK SERIAL DRIVER B ---
    node_driver_b = MockNode('serial_driver_b')
    node_driver_b.create_subscription(Float64MultiArray, '/servo_commands_b', lambda msg: None, 10)
    node_driver_b.create_subscription(Float64MultiArray, '/servo_single_command_b', lambda msg: None, 10)
    node_driver_b.create_subscription(Bool, '/feedback_enable_b', lambda msg: None, 10)
    node_driver_b.create_publisher(JointState, '/joint_states_real_b', 10)

    # --- MOCK LEG CONTROLLER ---
    node_leg_controller = MockNode('node_leg_controller')
    node_leg_controller.create_subscription(String, '/gait_control', lambda msg: None, 10)
    node_leg_controller.create_subscription(JointState, '/joint_states_real_a', lambda msg: None, 10)
    node_leg_controller.create_subscription(JointState, '/joint_states_real_b', lambda msg: None, 10)
    node_leg_controller.create_subscription(Point, '/posture_correction', lambda msg: None, 10)
    
    node_leg_controller.create_publisher(Float64MultiArray, '/servo_commands_a', 10)
    node_leg_controller.create_publisher(Float64MultiArray, '/servo_commands_b', 10)
    node_leg_controller.create_publisher(Bool, '/feedback_enable_a', 10)
    node_leg_controller.create_publisher(Bool, '/feedback_enable_b', 10)

    # --- MOCK BALANCE CONTROLLER ---
    node_balance = MockNode('nodeBalanceController')
    node_balance.create_subscription(Imu, '/imu/data_raw', lambda msg: None, 10)
    node_balance.create_subscription(Bool, '/balance_enable', lambda msg: None, 10)
    node_balance.create_subscription(String, '/balance_mode', lambda msg: None, 10)
    
    node_balance.create_publisher(Point, '/posture_correction', 10)
    node_balance.create_publisher(Point, '/posture_measurement', 10)

    # --- MOCK IMU SENSOR NODE ---
    node_imu = MockNode('imu_sensor_node')
    node_imu.create_publisher(Imu, '/imu/data_raw', 10)

    # --- MOCK TELEOP / COMMAND NODE ---
    node_teleop = MockNode('teleop_command_node')
    node_teleop.create_publisher(String, '/gait_control', 10)
    node_teleop.create_publisher(Bool, '/balance_enable', 10)
    node_teleop.create_publisher(String, '/balance_mode', 10)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node_driver_a)
    executor.add_node(node_driver_b)
    executor.add_node(node_leg_controller)
    executor.add_node(node_balance)
    executor.add_node(node_imu)
    executor.add_node(node_teleop)

    print("Mock Hardware Nodes are running! Open another terminal and run:")
    print("ros2 run rqt_graph rqt_graph")
    print("Press Ctrl+C to stop.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
