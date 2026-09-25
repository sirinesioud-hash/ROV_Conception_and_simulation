import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from pynput import keyboard

class BlueROV2SimpleController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Define individual publishers for each bluerov2/thruster
        self.pub1 = self.create_publisher(Float64, 'bluerov2/thruster_front_right_cmd', 10)
        self.pub2 = self.create_publisher(Float64, 'bluerov2/thruster_front_left_cmd', 10)
        self.pub3 = self.create_publisher(Float64, 'bluerov2/thruster_back_right_cmd', 10)
        self.pub4 = self.create_publisher(Float64, 'bluerov2/thruster_back_left_cmd', 10)
        self.pub5 = self.create_publisher(Float64, 'bluerov2/thruster_vertical_right_cmd', 10)
        self.pub6 = self.create_publisher(Float64, 'bluerov2/thruster_vertical_left_cmd', 10)
        self.pub_start_snapshot = self.create_publisher(Empty, 'take_snapshot', 10)

        self.get_logger().info('ROV Controller Active. Use Arrows, W/S for Up/Down, and Space to Stop.')

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def stop_rov(self):
        msg = Float64()
        msg.data = 0.0
        self.pub1.publish(msg)
        self.pub2.publish(msg)
        self.pub3.publish(msg)
        self.pub4.publish(msg)
        self.pub5.publish(msg)
        self.pub6.publish(msg)
        print("Stopping ROV")

    def move_forward(self):
        m = Float64()
        m.data = -5.0; self.pub1.publish(m)
        m.data = -5.0; self.pub2.publish(m)
        m.data = 5.0;  self.pub3.publish(m)
        m.data = 5.0;  self.pub4.publish(m)
        print("Moving Forward")

    def move_backward(self):
        m = Float64()
        m.data = 5.0;  self.pub1.publish(m)
        m.data = 5.0;  self.pub2.publish(m)
        m.data = -5.0; self.pub3.publish(m)
        m.data = -5.0; self.pub4.publish(m)
        print("Moving Backward")

    def rotate_ccw(self):
        m = Float64()
        m.data = -5.0; self.pub1.publish(m)
        m.data = 5.0;  self.pub2.publish(m)
        m.data = 5.0;  self.pub3.publish(m)
        m.data = -5.0; self.pub4.publish(m)
        print("Rotating Counter-Clockwise")

    def rotate_cw(self):
        m = Float64()
        m.data = 5.0;  self.pub1.publish(m)
        m.data = -5.0; self.pub2.publish(m)
        m.data = -5.0; self.pub3.publish(m)
        m.data = 5.0;  self.pub4.publish(m)
        print("Rotating Clockwise")

    def move_down(self):
        m = Float64()
        m.data = 5.0; self.pub5.publish(m)
        m.data = 5.0; self.pub6.publish(m)
        print("Moving Down")

    def move_up(self):
        m = Float64()
        m.data = -5.0; self.pub5.publish(m)
        m.data = -5.0; self.pub6.publish(m)
        print("Moving Up")
    def take_snapshot(self):
        m = Empty()
        self.pub_start_snapshot.publish(m)
        print("taking snapshot")

    def on_press(self, key):
        # Arrow Keys
        if key == keyboard.Key.up:
            self.move_forward()
        elif key == keyboard.Key.down:
            self.move_backward()
        elif key == keyboard.Key.left:
            self.rotate_ccw()
        elif key == keyboard.Key.right:
            self.rotate_cw()
        # Space to stop
        elif key == keyboard.Key.space:
            self.stop_rov()
        # W and S for Vertical
        try:
            if key.char == 'z':
                self.move_up()
            elif key.char == 's':
                self.move_down()
            elif key.char == 'b':
                self.take_snapshot()

        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2SimpleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()