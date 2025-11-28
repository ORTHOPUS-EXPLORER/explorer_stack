import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.axes = [0.0, 0.0]
        self.buttons = [0]
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.timer = self.create_timer(0.05, self.publish_joy)

    def on_press(self, key):
        try:
            if key.char in ['a', 'A']:
                self.buttons[0] = 1
        except AttributeError:
            if key == keyboard.Key.left:
                self.axes[0] = -1.0
            elif key == keyboard.Key.right:
                self.axes[0] = 1.0
            elif key == keyboard.Key.up:
                self.axes[1] = 1.0
            elif key == keyboard.Key.down:
                self.axes[1] = -1.0

    def on_release(self, key):
        try:
            if key.char in ['a', 'A']:
                self.buttons[0] = 0
        except AttributeError:
            if key == keyboard.Key.left and self.axes[0] == -1.0:
                self.axes[0] = 0.0
            elif key == keyboard.Key.right and self.axes[0] == 1.0:
                self.axes[0] = 0.0
            elif key == keyboard.Key.up and self.axes[1] == 1.0:
                self.axes[1] = 0.0
            elif key == keyboard.Key.down and self.axes[1] == -1.0:
                self.axes[1] = 0.0

    def publish_joy(self):
        msg = Joy()
        msg.axes = self.axes.copy()
        msg.buttons = self.buttons.copy()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
