#!/usr/bin/env python
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import logging


class ImuSubscriberForPyBullet(Node):
    def __init__(self):
        super().__init__('imu_sub_pybullet')
    
    
        self.IMU_FRAME = None
        self.IMU_FRAME = self.declare_parameter("~imu_frame", 'imu_link').value
        self.sub_topic = self.declare_parameter("~sub_topic", '/imu/data').value

        imu_qos = rclpy.qos.QoSPresetProfiles.get_from_short_key('sensor_data')
        #self.create_timer(1.0/self.rate, self.read_imu_and_publish_sensormsg)  
        self.create_subscription(Imu, self.sub_topic, self.update_imu_data_for_pybullet, imu_qos)
        self.get_logger().info(f"imu_sub_pybullet:: started ... sub_topic::{self.sub_topic} ")
        self.get_logger().info('imu_sub_pybullet:: started reading imu listener...')
        self.q = 0
        self.init_pybullet()
 
    def init_pybullet(self):

        try:
            cid = p.connect(p.SHARED_MEMORY)
            self.get_logger().info(f"imu_sub_pybullet:: cid={cid} ")
            if (cid < 0):
            #p.connect(p.GUI)
                p.connect(p.GUI, options="--opengl2")
                self.get_logger().info(f"connect complete")
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.get_logger().info(f"loading urdf")
            self.q = p.loadURDF("husky/husky.urdf", useFixedBase=True)
            p.setGravity(0, 0, -10)
            self.get_logger().info(f"setting initial co-ordinates")
            
        except:
            logging.exception("error occurred")
            pass
        
    def update_imu_data_for_pybullet(self, msg):
        try:
            self.get_logger().info('imu_sub_pybullet:: msg received...')
            ev = p.getEulerFromQuaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.get_logger().info(f'imu_sub_pybullet:: got Euler::{ev}')
            #orn = msg.orientation #p.getQuaternionFromEuler([roll, pitch, yaw])
            p.resetBasePositionAndOrientation(self.q, ev, [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.get_logger().info('imu_sub_pybullet:: updated orientation.')
        except:
            logging.exception("error occurred")
            pass
def main(args=None):
    

    rclpy.init(args=args)
    try:
        imu_sub_pybullet = ImuSubscriberForPyBullet()
        rclpy.spin(imu_sub_pybullet)
    except rclpy.exceptions.ROSInterruptException:
        pass

    imu_sub_pybullet.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()