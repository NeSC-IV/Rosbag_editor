import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from tqdm import tqdm
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import numpy as np
import sys

import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix, PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pcl2 # point_cloud2.create_cloud() 函数是sensor_msgs.msg.PointCloud2消息的一个帮助函数，它将一系列点的x、y、z坐标和其他属性打包到点云消息中。
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform, PoseStamped
from nav_msgs.msg import Odometry
import yaml
import os
import shutil


def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rclpy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

class Extract():

    def __init__(self, path):
        # super().__init__('image_creator')
        bridge = CvBridge()
        with open(path + '/metadata.yaml', 'r') as file:
            config = yaml.safe_load(file)
        storage_options = StorageOptions(uri=path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format= 'cdr', output_serialization_format ='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()
        # Create a map for quicker lookup
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        # Set filter for topic of string type
        enable_topics = ['/camera/image', '/livox/front', '/livox/back', '/livox/mid360', '/imu/data']
        storage_filter = StorageFilter(topics=enable_topics)
        reader.set_filter(storage_filter)
        # No filter
        # reader.reset_filter()

        # Get the total number of messages
        total_messages = config['rosbag2_bagfile_information']['topics_with_message_count']
        total_messages_count = 0
        for i in range(len(topic_types)):
            if topic_types[i].name in enable_topics:
                total_messages_count += total_messages[i]['message_count']
        # Initialize a progress bar
        self.progress_bar = tqdm(total=total_messages_count)
        
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            self.progress_bar.update(1)
            msg_type = get_message(type_map[topic])
            ros_message = deserialize_message(data, msg_type)
            
            sensor_name = topic.split('/')
            if topic == "/camera/image":
                try:
                    cv_image = bridge.imgmsg_to_cv2(ros_message)
                except CvBridgeError as e:
                    print (e)
                timestr = "%.9f" % (ros_message.header.stamp.sec + ros_message.header.stamp.nanosec / 1e9)
                image_name = timestr+ ".jpg"
                cv2.imwrite(path+"/pic/"+image_name, cv_image)
            
            elif sensor_name[1] == "livox":
                point_cloud = pcl2.read_points(ros_message)
                point_cloud_rec = point_cloud.view(np.recarray)
                x = point_cloud_rec.x.astype(np.float32)
                y = point_cloud_rec.y.astype(np.float32)
                z = point_cloud_rec.z.astype(np.float32)
                intensity = point_cloud_rec.intensity.astype(np.float32)
                tag = point_cloud_rec.tag.astype(np.uint8)
                line = point_cloud_rec.line.astype(np.uint8)
                timestamp = point_cloud_rec.timestamp.astype(np.float64)
                point_cloud_np = np.stack((x,y,z,intensity,tag,line,timestamp), axis=1)
                timestr = "%.9f" % (ros_message.header.stamp.sec + ros_message.header.stamp.nanosec / 1e9)
                front_name = timestr+ ".bin"
                point_cloud_np.tofile(path+"/point_cloud/" + sensor_name[2] + "/"+front_name)
                # 读取点云数据
                # point_cloud_np1 = np.fromfile(path+"/point_cloud/front/"+front_name, dtype=np.float64).reshape(-1, 7)

            elif topic == "/imu/data":
                imu_data = np.array([ros_message.linear_acceleration.x, ros_message.linear_acceleration.y, ros_message.linear_acceleration.z,
                                    ros_message.angular_velocity.x, ros_message.angular_velocity.y, ros_message.angular_velocity.z,
                                    ros_message.orientation.x, ros_message.orientation.y, ros_message.orientation.z, ros_message.orientation.w])
                timestr = "%.9f" % (ros_message.header.stamp.sec + ros_message.header.stamp.nanosec / 1e9)
                write_txt_path = path + "/imu.txt"
                with open(write_txt_path, 'a') as f:
                    f.write(timestr + ' ' + ' '.join(str(i) for i in imu_data) + '\n')

class Writebag():
    def __init__(self, src_path, bag_path):
        self.src_path = src_path
        if os.path.exists(bag_path):
            shutil.rmtree(bag_path)
        writer = rosbag2_py.SequentialWriter()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format= 'cdr', output_serialization_format ='cdr')
        writer.open(storage_options, converter_options)
        self.write_image(writer)
        self.write_pc(writer)

    def write_image(self, writer):
        topic_info = rosbag2_py.TopicMetadata(name='/camera/image', type='sensor_msgs/Image', serialization_format='cdr')
        writer.create_topic(topic_info)
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
            img_msg.header.stamp.sec = int(sec)
            img_msg.header.stamp.nanosec = int(nanosec)
            # Write the Image message to the bag
            writer.write('/camera/image', serialize_message(img_msg), int(img_msg.header.stamp.sec*1e9 + img_msg.header.stamp.nanosec))

    def write_pc(self, writer):
        topic_info = rosbag2_py.TopicMetadata(name='/livox/merge', type='sensor_msgs/PointCloud2', serialization_format='cdr')
        writer.create_topic(topic_info)

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
            header.stamp.sec = int(sec)
            header.stamp.nanosec = int(nanosec)
            fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                    PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
                    PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
                    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1)]
            pc2_msg = pcl2.create_cloud(header, fields, point_cloud)
            # Write the PointCloud2 message to the bag
            writer.write('/livox/merge', serialize_message(pc2_msg), int(pc2_msg.header.stamp.sec*1e9 + pc2_msg.header.stamp.nanosec))
        
class merge():
    def __init__(self, path):
        # path='/media/oliver/Elements SE/rosbag2_2024_04_03-16_32_39/point_cloud'
        # Get a list of all files in the directory
        front_files = os.listdir(path + '/front')
        front_files.sort()
        back_files = os.listdir(path + '/back')
        back_files.sort()
        mid360_files = os.listdir(path + '/mid360')
        mid360_files.sort()
        front_times = []
        back_times = []
        mid360_times = []
        # Print the list of files
        for file in front_files:
            sec, nsec, _ = file.split('.')
            time = float(sec) + float(nsec) / 1e9
            front_times.append(time)
        for file in back_files:
            sec, nsec, _ = file.split('.')
            time = float(sec) + float(nsec) / 1e9
            back_times.append(time)
        for file in mid360_files:
            sec, nsec, _ = file.split('.')
            time = float(sec) + float(nsec) / 1e9
            mid360_times.append(time)
        front_times = np.array(front_times)
        back_times = np.array(back_times)
        mid360_times = np.array(mid360_times)
        for f_id, f_time in tqdm(enumerate(front_times), total=front_times.shape[0], desc='Merging point cloud data', leave=False):
            back_time_idx = np.argmin(np.abs(back_times - f_time))
            back_time = np.abs(back_times[back_time_idx] - f_time)
            if back_time > 0.01:
                continue
            mid360_time_idx = np.argmin(np.abs(mid360_times - f_time))
            mid360_time = np.abs(mid360_times[mid360_time_idx] - f_time)
            if mid360_time > 0.01:
                continue
            # Read point cloud data
            front_data = np.fromfile(path + '/front/' + front_files[f_id], dtype=np.float64).reshape(-1, 7)
            back_data = np.fromfile(path + '/back/' + back_files[back_time_idx], dtype=np.float64).reshape(-1, 7)
            mid360_data = np.fromfile(path + '/mid360/' +  mid360_files[mid360_time_idx], dtype=np.float64).reshape(-1, 7)
            # Merge point cloud data
            merge_data = self.merge_pc(front_data, back_data, mid360_data)
            # Save the merged point cloud data
            merge_data.tofile(path + '/merge/' + front_files[f_id])
        
    def merge_pc(self, front, back, mid360):
        back[:, 5] += 6
        mid360[:, 5] += 12
        merge_data = np.concatenate((front, back, mid360), axis=0)
        return merge_data


def main(do="extract"):
    # rclpy.init(args=args)
    if do == "extract":
        Extract(path='/media/oliver/Elements SE/rosbag2_2024_04_03-16_32_39')
    elif do == "write":
        Writebag(src_path='/media/oliver/Elements SE/rosbag2_2024_04_03-16_32_39', bag_path='/media/oliver/Elements SE/rosbag2_write')
    elif do == "merge":
        merge(path='/media/oliver/Elements SE/rosbag2_2024_04_03-16_32_39/point_cloud')

if __name__ == '__main__':
    main()
