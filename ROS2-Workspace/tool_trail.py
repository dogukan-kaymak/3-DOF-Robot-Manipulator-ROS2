import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
import math

class ToolTrailNode(Node):
    def __init__(self):
        super().__init__('tool_trail_node')
        
        # RViz'e cizgi cizdirmek icin Marker yayinlayicisi
        self.publisher_ = self.create_publisher(Marker, '/tool_trail', 10)
        
        # TF dinleyicisi (Robotun anlik pozisyonunu bulmak icin)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.ns = "tool_trail"
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        
        # Cizgi kalinligi (5 mm)
        self.marker.scale.x = 0.005
        
        # Cizgi rengi (Kirmizi)
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0 # Opaklik
        
        # 10 Hz ile robotun ucunun nerede olduguna bakacagiz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_point = None
        
        self.get_logger().info("=========================================")
        self.get_logger().info("Uç Nokta (Tool Tip) Cizim Modu Aktif!")
        self.get_logger().info("RViz'de 'Add' butonuna basip 'Marker' ekleyin.")
        self.get_logger().info("Marker topic'ini '/tool_trail' secin.")
        self.get_logger().info("Cizgileri silmek isterseniz bu terminalde CTRL+C yapip yeniden calistirin.")
        self.get_logger().info("=========================================")

    def timer_callback(self):
        try:
            # base_link (yer) ile tool_tip (mizrak ucu) arasindaki anlik konumu oku
            t = self.tf_buffer.lookup_transform('base_link', 'tool_tip', rclpy.time.Time())
            
            p = Point()
            p.x = t.transform.translation.x
            p.y = t.transform.translation.y
            p.z = t.transform.translation.z
            
            # Eger pozisyon hic degismediyse (robot duruyorsa) listeye ekleyip RAM'i doldurma
            if self.last_point:
                dist = math.sqrt((p.x - self.last_point.x)**2 + (p.y - self.last_point.y)**2 + (p.z - self.last_point.z)**2)
                if dist < 0.001: # 1 mm'den az hareket ettiyse yeni cizgi atma
                    return
            
            self.last_point = p
            self.marker.points.append(p)
            
            # Zaman damgasini guncelle ve RViz'e gonder
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.marker)
            
        except tf2_ros.TransformException as ex:
            # Robot henuz acilmadiysa veya tool_tip yoksa hata vermesin, beklesin
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ToolTrailNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
