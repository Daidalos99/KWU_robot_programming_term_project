from msg_srv_action_interface_example.srv import SendToGUI
import rclpy
from rclpy.node import Node

class TempGUI(NOoe):

    def __init__(self):
        super().__init__('temporary_gui')
        self.gui_server = self.create_service
