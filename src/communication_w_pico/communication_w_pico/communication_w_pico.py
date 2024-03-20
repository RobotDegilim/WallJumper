import rclpy
from player_commands.msg import PlayerCommand
from rclpy.node import Node
from .communicate import Serial_Talker 

class CommunicationWPico(Node):
    def __init__(self):
        super().__init__('communication_w_pico')
        self.serial_comm = Serial_Talker() 
        self.subscribe_player_commands = self.create_subscription(
            PlayerCommand, 'player_head_pos', self.player_commands_callback, 10)
        self.get_logger().info('Subscribed to player_head_pos')
    
    def player_commands_callback(self, msg):
        '''
        Parses the message and serializes it to a string to send to the PICO.
        Serializes it into this form: x,y;x,y
        '''
        head_pos = msg.head_pos
        frame_pos = msg.frame_pos
        serialized_msg = f"{head_pos[1]}"
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

