'''
This script takes an input HILTI-22 rosbag and adds CameraInfo and TF topics to it.

These can be extracted from https://storage.googleapis.com/hsc2022/calibration/2022322_calibration_files.zip
'''

from copy import deepcopy
from pprint import pprint

import numpy as np
import rosbag
import tf
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm

IN_BAG = "/mnt/ssd_2T/hilti-22/exp23_the_sheldonian_slam_part_0.bag"
OUT_BAG = "/mnt/ssd_2T/hilti-22/exp23_the_sheldonian_slam_part_0+.bag"

T_cam0_imu = np.array([[0.006708021451800439, 0.9999264200621176, -0.01010727015365992, -0.04586422589354697],
                       [0.002425643641803532, 0.010091197021688258, 0.9999461405473755, 0.012631813183337478],
                       [0.9999745590269407, -0.00673217679702115, -0.0023577731969991467, -0.05098782892861867],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam1_imu = np.array([[0.0016556126470597954, 0.9999840642813064, -0.005397233569448934, 0.06262779955244471],
                      [0.0009350089535379302, 0.005395690615039778, 0.9999850060281122, 0.01343471252962214],
                       [0.9999981923508761, -0.0016606342845620903, -0.0009260608798763448, -0.05050835847481363],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam2_imu = np.array([[0.9999897552434941, 0.0042340516721277015, -0.0016006918802386892, -0.0068618144566339415],
                       [0.004238470700886935, -0.9999871881804255, 0.0027674611333545654, -0.007931259020147333],
                       [-0.0015889537990238962, -0.0027742172670246527, -0.9999948894591316, -0.034124943940492],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam3_imu = np.array([[-0.9998916894135631, 0.007396041673023157, 0.01272430781071447, -0.0030245656200529533],
                       [0.01270714046408309, -0.002363530935315569, 0.9999164676625464, 0.011180668261220912],
                       [0.007425498159515405, 0.9999698556902047, 0.002269292399476633, -0.05718309342102409],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam4_imu = np.array([[0.9999880402484476, -0.000942427662931895, -0.004799082221662863, 0.006416247252956556],
                      [0.004797008865106434, -0.0021900897852221157, 0.9999860960096802, 0.01670064540948574],
                       [-0.0009529249803788974, -0.9999971576643774, -0.0021855427584387965, -0.07488037729385075],
                       [0.0, 0.0, 0.0, 1.0]])
T_cams_imu = [T_cam0_imu, T_cam1_imu, T_cam2_imu, T_cam3_imu, T_cam4_imu]
print("Camera calibration info:")
pprint(T_cams_imu)

Cam0_CamInfo = CameraInfo()
Cam0_CamInfo.height = 540
Cam0_CamInfo.width = 720
Cam0_CamInfo.distortion_model = "plumb_bob"
Cam0_CamInfo.D = [0.03696737352869157, -0.008917880497032812, 0.008912969593422046, -0.0037685977496087313, 0.0]
Cam0_CamInfo.K = [351.31400364193297, 0.0, 367.8522793375995,
                  0.0, 351.4911744656785, 253.8402144980996,
                  0.0, 0.0, 1.0]
Cam0_CamInfo.P = [351.31400364193297, 0.0, 367.8522793375995, 0.0,
                  0.0, 351.4911744656785, 253.8402144980996, 0.0,
                  0.0, 0.0, 1.0, 0.0]

Cam1_CamInfo = CameraInfo()
Cam1_CamInfo.height = 540
Cam1_CamInfo.width = 720
Cam1_CamInfo.distortion_model = "plumb_bob"
Cam1_CamInfo.D = [-0.039086652082708805, -0.005525347047415151, 0.004398151558986798, -0.0019701263170917808, 0.0]
Cam1_CamInfo.K = [352.6489794433894, 0.0, 347.8170010310082,
                  0.0, 352.8586498571586, 270.5806692485468,
                  0.0, 0.0, 1.0]
Cam1_CamInfo.P = [352.6489794433894, 0.0, 347.8170010310082, 0.0,
                  0.0, 352.8586498571586, 270.5806692485468, 0.0,
                  0.0, 0.0, 1.0, 0.0]

Cam2_CamInfo = CameraInfo()
Cam2_CamInfo.height = 540
Cam2_CamInfo.width = 720
Cam2_CamInfo.distortion_model = "plumb_bob"
Cam2_CamInfo.D = [-0.041202246303621064, -0.0012607385825244833, 0.0006712169937177444, -0.0006234254968089226, 0.0]
Cam2_CamInfo.K = [350.70040966794545, 0.0, 375.2977403521422,
                  0.0, 350.8792449525716, 268.5927747079796,
                  0.0, 0.0, 1.0]
Cam2_CamInfo.P = [350.70040966794545, 0.0, 375.2977403521422, 0.0,
                  0.0, 350.8792449525716, 268.5927747079796, 0.0,
                  0.0, 0.0, 1.0, 0.0]

Cam3_CamInfo = CameraInfo()
Cam3_CamInfo.height = 540
Cam3_CamInfo.width = 720
Cam3_CamInfo.distortion_model = "plumb_bob"
Cam3_CamInfo.D = [-0.03890973498616883, -0.002604676547864069, 0.0004634700730293949, -0.00036698216675371063, 0.0]
Cam3_CamInfo.K = [352.9514843860555, 0.0, 363.93345228274336,
                  0.0, 353.32837903547403, 266.14511705007413,
                  0.0, 0.0, 1.0]
Cam3_CamInfo.P = [352.9514843860555, 0.0, 363.93345228274336, 0.0,
                  0.0, 353.32837903547403, 266.14511705007413, 0.0,
                  0.0, 0.0, 1.0, 0.0]

Cam4_CamInfo = CameraInfo()
Cam4_CamInfo.height = 540
Cam4_CamInfo.width = 720
Cam4_CamInfo.distortion_model = "plumb_bob"
Cam4_CamInfo.D = [-0.03842764034005408, -0.005841411460411122, 0.003451041303088915, -0.0011463543672005018, 0.0]
Cam4_CamInfo.K = [351.5132148653381, 0.0, 342.8425988673232,
                  0.0, 351.7557554938886, 259.91793254535776,
                  0.0, 0.0, 1.0]
Cam4_CamInfo.P = [351.5132148653381, 0.0, 342.8425988673232, 0.0,
                  0.0, 351.7557554938886, 259.91793254535776, 0.0,
                  0.0, 0.0, 1.0, 0.0]

# Read all messages from the input bag
inbag = rosbag.Bag(IN_BAG)
outbag = rosbag.Bag(OUT_BAG, "w")

TOTAL_MESSAGE_COUNT = -1
msgs_written = 0
last_tf_written_timestamp = None

print("Adding CameraInfo and TF messages to the bag...")
with outbag as outbag:

    for topic, msg, t in tqdm(inbag.read_messages(), total=inbag.get_message_count()):

        # Write Tf_static message
        if last_tf_written_timestamp is None or (t - last_tf_written_timestamp).to_sec() > 1.0:
            last_tf_written_timestamp = t

            # Add TF messages
            for i, T_cam_imu in enumerate(T_cams_imu):
                T_imu_cam = np.linalg.inv(T_cam_imu)
                tf_msg = TransformStamped()
                tf_msg.header.stamp = last_tf_written_timestamp
                tf_msg.header.frame_id = "imu_sensor_frame"
                tf_msg.child_frame_id = f"cam{i}_sensor_frame"

                tf_msg.transform.translation.x = T_imu_cam[0, 3]
                tf_msg.transform.translation.y = T_imu_cam[1, 3]
                tf_msg.transform.translation.z = T_imu_cam[2, 3]

                q = tf.transformations.quaternion_from_matrix(T_imu_cam)
                tf_msg.transform.rotation.x = q[0]
                tf_msg.transform.rotation.y = q[1]
                tf_msg.transform.rotation.z = q[2]
                tf_msg.transform.rotation.w = q[3]

                outbag.write("/tf_static", tf_msg, last_tf_written_timestamp)

        # Write CameraInfo message
        if topic == "/alphasense/cam0/image_raw":
            Cam0_CamInfo.header = msg.header
            outbag.write("/alphasense/cam0/camera_info", Cam0_CamInfo, t)

            msgs_written += 1
            if TOTAL_MESSAGE_COUNT > 0 and msgs_written >= TOTAL_MESSAGE_COUNT:
                break

        elif topic == "/alphasense/cam1/image_raw":
            Cam1_CamInfo.header = msg.header
            outbag.write("/alphasense/cam1/camera_info", Cam1_CamInfo, t)

        elif topic == "/alphasense/cam2/image_raw":
            Cam2_CamInfo.header = msg.header
            outbag.write("/alphasense/cam2/camera_info", Cam2_CamInfo, t)

        elif topic == "/alphasense/cam3/image_raw":
            Cam3_CamInfo.header = msg.header
            outbag.write("/alphasense/cam3/camera_info", Cam3_CamInfo, t)

        elif topic == "/alphasense/cam4/image_raw":
            Cam4_CamInfo.header = msg.header
            outbag.write("/alphasense/cam4/camera_info", Cam4_CamInfo, t)

        # Write all other messages except PC2
        if topic == "/hesai/pandar":
            continue

        outbag.write(topic, msg, t)
