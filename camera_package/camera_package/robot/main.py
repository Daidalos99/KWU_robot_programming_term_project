import rclpy
from rclpy.executors import MultiThreadedExecutor

from camera_package.robot.robot_core import RobotCore
from camera_package.robot.database import Database

def main(args=None):
    rclpy.init(args=args)

    try:
        robot_core = RobotCore()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(robot_core)
        try:
            executor.spin()
        except KeyboardInterrupt:
            robot_core.get_logger.info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            robot_core.state_in_server.destroy()
            robot_core.state_out_server.destroy()
            robot_core.camera_server.destroy()
            robot_core.destroy_node()
    finally:
        rclpy.shutdown()

def main_database(args=None):
    rclpy.init(args=args)

    try:
        database = Database()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(database)
        try:
            executor.spin()
        except KeyboardInterrupt:
            database.get_logger.info('Keyboard Interrupt (SIGINT)')
        finally:
            database.entered_car_server.destroy()
            database.exited_car_server.destroy()
            database.camera_car_server.destroy()
            database.gui_server.destroy()
            database.destroy_node()
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main_database()


