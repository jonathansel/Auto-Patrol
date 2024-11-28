import rospy
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def create_rgb_point_field():
    # Define the point field for RGB
    return PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)

def pack_rgb_into_point(r, g, b):
    # Pack RGB values into a uint32
    return struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]

def create_point_cloud2(points_with_colors):
    header = Header(frame_id="base_link")
    point_cloud = PointCloud2()
    
    point_cloud.header = header
    point_cloud.height = 1
    point_cloud.width = len(points_with_colors)
    point_cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        create_rgb_point_field()
    ]
    point_cloud.is_bigendian = False
    point_cloud.point_step = 16  # Float32 for XYZ + UInt32 for RGB
    point_cloud.row_step = point_cloud.point_step * point_cloud.width
    point_cloud.is_dense = False
    buffer = []

    for x, y, z, r, g, b in points_with_colors:
        buffer.append(struct.pack('fffI', x, y, z, pack_rgb_into_point(r, g, b)))
    
    point_cloud.data = b''.join(buffer)

    return point_cloud

# Example usage:
# points_with_colors = [(x, y, z, r, g, b), ...]
# point_cloud2_msg = create_point_cloud2(points_with_colors)
# point_cloud.publish(point_cloud2_msg)

def pcl_to_ros(pcl_cloud):
    """
    Convert a PCL PointCloud to a ROS PointCloud2 message.

    :param pcl_cloud: The PCL PointCloud.
    :return: The ROS PointCloud2 message.
    """
    ros_cloud = PointCloud2()
    ros_cloud.header.stamp = rospy.Time.now()
    ros_cloud.header.frame_id = "base_link"  # Change to your frame_id
    ros_cloud.height = 1  # Assuming an unorganized point cloud
    ros_cloud.width = pcl_cloud.size

    # Assuming pcl_cloud is of type pcl.PointCloud_PointXYZRGB
    # Adjust field names if your point cloud has a different type
    ros_cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
    ]

    ros_cloud.is_bigendian = False  # Depends on your system architecture
    ros_cloud.point_step = 16  # For FLOAT32 RGB, the size is 4 bytes * 4 fields (x, y, z, rgb)
    ros_cloud.row_step = ros_cloud.point_step * pcl_cloud.size
    ros_cloud.is_dense = False  # If there are no invalid points, set to True

    buffer = []

    # for point in pcl_cloud:
    for x, y, z, r, g, b in pcl_cloud:
        # Assuming point is a tuple (x, y, z, rgb)
        buffer.append(struct.pack('fffI', x, y, z, pack_rgb_into_point(r, g, b)))
        # buffer.append(struct.pack('fffI', point[0], point[1], point[2], point[3]))

    ros_cloud.data = b''.join(buffer)

    return ros_cloud

def pcl_to_ros_with_color(pcl_cloud, r, g, b):
    """
    Convert a PCL PointCloud without RGB to a ROS PointCloud2 message and add color.

    :param pcl_cloud: The PCL PointCloud.
    :param r: Red component to add (0-255).
    :param g: Green component to add (0-255).
    :param b: Blue component to add (0-255).
    :return: The ROS PointCloud2 message.
    """
    ros_cloud = PointCloud2()
    ros_cloud.header.stamp = rospy.Time.now()
    ros_cloud.header.frame_id = "base_link"  # Adjust to your frame ID
    ros_cloud.height = 1  # Unorganized point cloud
    ros_cloud.width = pcl_cloud.size

    # Define the fields for an XYZRGB point cloud
    ros_cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    ros_cloud.is_bigendian = False
    ros_cloud.point_step = 16  # 4 bytes each for x, y, z, and rgb
    ros_cloud.row_step = ros_cloud.point_step * ros_cloud.width
    ros_cloud.is_dense = False

    # Pack RGB into a single float (this is a common approach in PCL for storing RGB)
    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

    buffer = []
    for point in pcl_cloud:
        # Pack each point with the RGB value
        buffer.append(struct.pack('fffI', point[0], point[1], point[2], rgb))

    ros_cloud.data = b''.join(buffer)

    return ros_cloud