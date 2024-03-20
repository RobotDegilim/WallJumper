import rclpy
from std_msgs.msg import Float64
from rclpy.node import Node
from .communicate import Serial_Talker 

class CommunicationWPico(Node):
    def __init__(self):
        super().__init__('communication_w_pico')
        self.serial_comm = Serial_Talker() 
        self.subscribe_player_commands = self.create_subscription(
            Float64, 'y_position', self.head_cb_, 10)
        self.get_logger().info('Subscribed to player_head_pos')
    
    def head_cb_(self, msg):
        '''
        Parses the message and serializes it to a string to send to the PICO.
        Serializes it into this form: y
        '''
        serialized_msg = f"{msg.data}"
        self.serial_comm.send(serialized_msg)

    def __del__(self):
        if hasattr(self, 'serial_comm') and self.serial_comm is not None:
            self.serial_comm.close()

def main(args=None):
    rclpy.init(args=args)
    communication_w_pico = CommunicationWPico()
    rclpy.spin(communication_w_pico)
    communication_w_pico.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

