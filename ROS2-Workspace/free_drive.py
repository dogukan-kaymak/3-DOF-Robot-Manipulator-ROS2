import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import threading
import sys

class FreeDriveNode(Node):
    def __init__(self):
        super().__init__('free_drive_node')
        self.pub1 = self.create_publisher(Float32, '/motor1/target', 10)
        self.pub2 = self.create_publisher(Float32, '/motor2/target', 10)
        self.pub3 = self.create_publisher(Float32, '/motor3/target', 10)
        
        self.set_home_flag = False
        
        # 1 Hz ile surekli gonderiyoruz
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('==================================================')
        self.get_logger().info('FREE DRIVE (SERBEST SURUS) MODU AKTIF!')
        self.get_logger().info('Motorlar gucsuz birakildi. Robotu elinizle hareket ettirebilirsiniz.')
        self.get_logger().info('RViz uzerinde gercek konumu eszamanli gorebilirsiniz.')
        self.get_logger().info('--------------------------------------------------')
        self.get_logger().info('YENI SIFIR NOKTASI BELIRLEMEK ICIN: "ENTER" tusuna basin!')
        self.get_logger().info('CIKMAK ICIN: CTRL+C yapin.')
        self.get_logger().info('==================================================')

        # Klavye girdisini dinlemek icin ayri bir thread
        self.input_thread = threading.Thread(target=self.wait_for_enter)
        self.input_thread.daemon = True
        self.input_thread.start()

    def wait_for_enter(self):
        while True:
            sys.stdin.readline() # Enter tusuna basilmasini bekler
            self.set_home_flag = True
            self.get_logger().info('*** KONUM SIFIRLANDI! YENI HOME NOKTASI KAYDEDILDI. ***')

    def timer_callback(self):
        msg = Float32()
        
        if self.set_home_flag:
            msg.data = -999.0 # Sihirli SIFIRLAMA komutumuz (Home yap)
            self.set_home_flag = False # Bayragi indir
        else:
            msg.data = -998.0 # Sadece Serbest Surus (Sifirlama YOK)

        self.pub1.publish(msg)
        self.pub2.publish(msg)
        self.pub3.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FreeDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Free Drive Modu Kapatildi! Kontrol MoveIt\'e devredildi.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
