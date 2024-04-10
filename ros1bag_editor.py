import os
from rosbag import Bag
import rospy
from tqdm import tqdm
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix, PointCloud2, Image
from std_msgs.msg import Header
import numpy as np
import sensor_msgs.point_cloud2 as pcl2

class Writebag():
    def __init__(self, src_path, bag_path):
        self.src_path = src_path
        if os.path.exists(bag_path):
            os.remove(bag_path)
        self.bag_path = bag_path
        self.bag = Bag(bag_path, 'w')
        self.write_image()
        self.write_imu()
        self.write_pc()

    
    def write_image(self):
        bridge = CvBridge()
        path = self.src_path + '/pic/'
        pic_files = os.listdir(path)
        pic_files.sort()
        for file in tqdm(pic_files, total=len(pic_files), desc='Writing image data', leave=False):
            img = cv2.imread(path + file)
            # Create an Image message
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            # Set the header
            sec, nanosec, _ = str(file).split('.')
            img_msg.header = Header(stamp=rospy.Time(secs=int(sec), nsecs=int(nanosec)))
            # Write the Image message to the bag
            self.bag.write('/camera/image', img_msg, t=img_msg.header.stamp)
            # with Bag(self.bag_path, 'a') as bag:
            #     bag.write('/camera/image', img_msg, t=img_msg.header.stamp)
    
    def write_imu(self):
        path = self.src_path + '/imu.txt'
        with open(path, 'r') as f:
            lines = f.readlines()
            for line in tqdm(lines, total=len(lines), desc='Writing imu data', leave=False):
                data = line.split()
                # Create an Imu message
                imu_msg = Imu()
                imu_msg.header = Header(stamp=rospy.Time(secs=int(data[0].split('.')[0]), nsecs=int(data[0].split('.')[1])))
                imu_msg.linear_acceleration.x = float(data[1])
                imu_msg.linear_acceleration.y = float(data[2])
                imu_msg.linear_acceleration.z = float(data[3])
                imu_msg.angular_velocity.x = float(data[4])
                imu_msg.angular_velocity.y = float(data[5])
                imu_msg.angular_velocity.z = float(data[6])
                imu_msg.orientation.x = float(data[7])
                imu_msg.orientation.y = float(data[8])
                imu_msg.orientation.z = float(data[9])
                imu_msg.orientation.w = float(data[10])
                # Write the Imu message to the bag
                self.bag.write('/imu/data', imu_msg, t=imu_msg.header.stamp)
                # with Bag(self.bag_path, 'a') as bag:
                #     bag.write('/imu/data', imu_msg, t=imu_msg.header.stamp)
    
    def write_pc(self):
        path = self.src_path + '/point_cloud/merge/'
        pc_files = os.listdir(path)
        pc_files.sort()
        for file in tqdm(pc_files, total=len(pc_files), desc='Writing point cloud data', leave=False):
            pc_path = path + file
            point_cloud = np.fromfile(pc_path, dtype=np.float64).reshape(-1, 7)
            x = point_cloud[:, 0].astype(np.float32)
            y = point_cloud[:, 1].astype(np.float32)
            z = point_cloud[:, 2].astype(np.float32)
            intensity = point_cloud[:, 3].astype(np.float32)
            tag = point_cloud[:, 4].astype(np.uint8)
            line = point_cloud[:, 5].astype(np.uint8)
            timestamp = point_cloud[:, 6].astype(np.float64)
            # Create dtypes for the point cloud
            dtypes = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32),
                  ('tag', np.uint8), ('line', np.uint8), ('timestamp', np.float64)]
            point_cloud = np.empty(x.shape[0], dtype=dtypes)
            point_cloud['x'] = x
            point_cloud['y'] = y
            point_cloud['z'] = z
            point_cloud['intensity'] = intensity
            point_cloud['tag'] = tag
            point_cloud['line'] = line
            point_cloud['timestamp'] = timestamp

            # Create a PointCloud2 message
            sec, nanosec, _ = str(file).split('.')
            header = Header()
            header.frame_id = 'livox_frame'
            header.stamp = rospy.Time(secs=int(sec), nsecs=int(nanosec))
            fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                  PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
                  PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
                  PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1)]
            pc2_msg = pcl2.create_cloud(header, fields, point_cloud)
            # Write the PointCloud2 message to the bag
            self.bag.write('/livox/merge', pc2_msg, t=pc2_msg.header.stamp)
            # with Bag(self.bag_path, 'a') as bag:
            #     bag.write('/livox/merge', pc2_msg, t=pc2_msg.header.stamp)

def main(do="write"):
    if do == "write":
        Writebag(src_path='/media/oliver/Elements SE/rosbag2_2024_04_03-16_32_39',
                  bag_path='/media/oliver/Elements SE/rosbag1_write.bag')

if __name__ == '__main__':
    main()
