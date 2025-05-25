#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
     ___     _____ ___  _   _    ___     __     
    / \ \   / /_ _/ _ \| \ | |  / \ \   / /     
   / _ \ \ / / | | | | |  \| | / _ \ \ / /      
  / ___ \ V /  | | |_| | |\  |/ ___ \ V /       
 /_/  _\_\_/  |___\___/|_| \_/_/_  \_\_/        
 | \ | |/ _ \|  \/  |  / \  |  _ \| ____|       
 |  \| | | | | |\/| | / _ \ | | | |  _|         
 | |\  | |_| | |  | |/ ___ \| |_| | |___        
 |_| \_|\___/|_|  |_/_/   \_\____/|_____|       
   ___ ___  _ __ | |_ _ __ ___ | | | ___ _ __   
  / __/ _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|  
 | (_| (_) | | | | |_| | | (_) | | |  __/ |     
  \___\___/|_| |_|\__|_|  \___/|_|_|\___|_|   

  
  
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

---------------------------
  _                                        
 | |__ _  _                                
 | '_ \ || |                               
 |_.__/\_, |                               
  __  _|__/               _           _    
 |  \/  |__ _ ______ _ _ | |_______ _(_)__ 
 | |\/| / _` |_ / -_) ' \| / / _ \ V / / _|
 |_|  |_\__,_/__\___|_||_|_\_\___/\_/|_\__|
                                           


q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            print(msg)
            while True:
                key = self.get_key()
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print(f"currently:\tspeed {self.speed}\tturn {self.turn}")
                elif key == ' ' or key == 'k':
                    self.x = 0
                    self.th = 0
                else:
                    if key == '\x03':
                        break

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.angular.z = self.th * self.turn
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.publisher_.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_control = TeleopControl()
    teleop_control.run()
    teleop_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()