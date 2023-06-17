import rclpy
from rclpy.executors import MultiThreadedExecutor

from camera_package.robot.database import Database

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


