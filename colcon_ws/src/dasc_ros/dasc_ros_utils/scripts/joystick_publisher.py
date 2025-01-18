
import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from datetime import datetime
import time
import sys
print(pygame.__file__)

REFRESH_RATE = 100.0

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.


class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 30)

    def text_print(self, screen, textString):
        textBitmap = self.font.render(textString, True, WHITE)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 30

    def indent(self):
        self.x += 20

    def unindent(self):
        self.x -= 20


pygame.init()

# Set the width and height of the screen [width, height]
size = [500, 800]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Joystick Tester")


# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

print("="*40)
print('start joystick publisher in {} hz'.format(REFRESH_RATE))
print('Ctrl-C or Start button to Exit')
#from rclpy.qos import qos_profile_sensor_data


class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('Joystick')
        qos_profile = QoSProfile(depth=1)
        self.joystick_publisher = self.create_publisher(
            Joy, 'joystick', qos_profile)
        self.timer = self.create_timer(
            1/REFRESH_RATE, self.publish_joystick_msg)
        self.exist_joy = False
        self.joystick = 0
        self.joy_idx = 0

    def publish_joystick_msg(self):
        #t1  = time.time()
        try:
            # Event processing
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # Drawing code
            screen.fill(BLACK)
            textPrint.reset()

            # Get count of joysticks
            joystick_count = pygame.joystick.get_count()

            textPrint.text_print(
                screen, "Number of joysticks: {}".format(joystick_count))
            textPrint.indent()

            # For each joystick:
            if self.exist_joy == False and joystick_count > 0:
                self.exist_joy = True
                #t1  = time.time()
                self.joy_idx = 0
                self.joystick = pygame.joystick.Joystick(self.joy_idx)
                #print(time.time() - t1)

            if self.exist_joy:
                joy = Joy()
                t = datetime.now()
                joy.header.stamp = super().get_clock().now().to_msg()  # in python, sec -> secs

                self.joystick.init()

                textPrint.text_print(
                    screen, "Joystick {}".format(self.joy_idx))
                textPrint.indent()

                # Get the name from the OS for the controller/joystick
                name = self.joystick.get_name()
                textPrint.text_print(screen, "Joystick name: {}".format(name))

                # Usually axis run in pairs, up/down for one, and left/right for the other.
                axes = self.joystick.get_numaxes()
                textPrint.text_print(screen, "Number of axes: {}".format(axes))
                textPrint.indent()

                for i in range(axes):
                    axis = self.joystick.get_axis(i)
                    textPrint.text_print(
                        screen, "Axis {} value: {:>6.3f}".format(i, axis))
                    joy.axes.append(axis)
                textPrint.unindent()

                buttons = self.joystick.get_numbuttons()
                textPrint.text_print(
                    screen, "Number of buttons: {}".format(buttons))
                textPrint.indent()

                for i in range(buttons):
                    button = self.joystick.get_button(i)
                    textPrint.text_print(
                        screen, "Button {:>2} value: {}".format(i, button))
                    joy.buttons.append(button)
                textPrint.unindent()

                # Hat switch. All or nothing for direction, not like joysticks.
                # Value comes back in a tuple.
                hats = self.joystick.get_numhats()
                textPrint.text_print(screen, "Number of hats: {}".format(hats))
                textPrint.indent()

                for i in range(hats):
                    hat = self.joystick.get_hat(i)
                    textPrint.text_print(
                        screen, "Hat {} value: {}".format(i, str(hat)))
                textPrint.unindent()

                textPrint.unindent()

                self.joystick_publisher.publish(joy)

                if joy.buttons[12] == 1:  # start button of Xbox joycon pressed
                    pygame.quit()

            self.joystick_publisher.publish(joy)
            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()
            # clock.tick(REFRESH_RATE)

        except KeyboardInterrupt:
            pygame.quit()
            sys.exit()
        #msg.data = 'Hello World: {0}'.format(self.count)
        #self.get_logger().info('Published message: {0}'.format(msg.data))
        # print(time.time()-t1)
        # print()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()