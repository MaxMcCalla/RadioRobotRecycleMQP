import torch
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
import numpy as np
#import matplotlib.pyplot as plt
import cv2
from sensor_msgs import msg as sensorMsg
from geometry_msgs import msg as geomMsg
from std_msgs import msg as stdMsg
import torch
from torchvision.models.detection import maskrcnn_resnet50_fpn_v2
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from distinctipy import distinctipy
import struct
from sensor_msgs_py import point_cloud2
import torchvision.transforms as transforms

transform = transforms.Compose([
    transforms.ToTensor()
])

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix


class CameraService(Node):

    def __init__(self):
        super().__init__('Camera')
        #Publishers
        self.color_frame_publisher = self.create_publisher(sensorMsg.Image, 'color_frame', 10)
        self.depth_frame_publisher = self.create_publisher(sensorMsg.Image, 'depth_frame', 10)
        self.point_cloud_publisher = self.create_publisher(sensorMsg.PointCloud2, 'cloud', 10)
        self.point_tcloud_publisher = self.create_publisher(sensorMsg.PointCloud2, 'transform_cloud', 10)

        #Subscribers
        self.listener = self.create_subscription(geomMsg.Transform, 'robot_pose', self.publish, 10)

        #Variables of camera. 
        self.ctx = rs.context()
        listOfDevices = self.ctx.query_devices()
        if(len(listOfDevices) == 0):
            raise RuntimeError("No device connected.")
        self.device = list[0]
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline = rs.pipeline()
        #self.bridge = CvBridge()
        self.Cto4 = np.asarray([[.8860,-.4153,.2060,.0630],[.3917, .9084, .1466, -.1164],[-.2480, -.0492, .9675, .0136], [0, 0, 0, 1]])
        self.model = self.get_model(7)
        self.colors = distinctipy.get_colors(7)
        self.class_names = ['BG', 'Glass', 'Metal', 'Other', 'Paper', 'Plastic', 'Trash']

    def get_model(self, num_classes):
        model = maskrcnn_resnet50_fpn_v2(weights = 'DEFAULT')

        in_features_box = model.roi_heads.box_predictor.cls_score.in_features
        in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels

        dim_reduced = model.roi_heads.mask_predictor.conv5_mask.out_channels

        model.roi_heads.box_predictor = FastRCNNPredictor(in_channels = in_features_box, num_classes = 7)
        model.roi_heads.mask_predictor = MaskRCNNPredictor(in_channels = in_features_mask, dim_reduced = dim_reduced, num_classes = 7)

        model.load_state_dict(torch.load('/home/alecr/ros2_ws/src/mqp/model_weights10_10.pth', weights_only=True, map_location=torch.device('cpu')))
        model.eval()

        return model

    def publish(self, msg):
        
        t = .00254*np.asarray([msg.translation.x, msg.translation.y, msg.translation.z])
        print("T, ", t)
        Q = [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        rot_matrix = np.asarray(quaternion_rotation_matrix(Q))
        print("Rotational Matrix: ", rot_matrix)
        transform_mat = np.column_stack((rot_matrix, t))
        transform_mat = np.matmul(self.Cto4, np.vstack((transform_mat, [0, 0, 0, 1])))
        print("Transform mat: ", transform_mat)
        depth = t[2]+20
        #print("Depth: ", depth)
        depth_frame, color_frame = self.color_post_depth()
        #Only publish an image if we can get it. 
        if(depth_frame != None):
            color = np.asanyarray(color_frame.get_data())
            depth_data = np.asanyarray(depth_frame.get_data())
            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            depth_mask = np.uint8(np.where(depth_data < depth, 1, 0))
            #cv2.namedWindow('color')
            #cv2.imshow('color', color)
            #cv2.namedWindow('depth')
            #cv2.imshow('depth', colorized_depth)
            #cv2.namedWindow('depth_mask')
            #cv2.imshow('depth_mask', depth_mask)
            #cv2.waitKey()
            #color = cv2.bitwise_and(color, color, mask = combinedMask)
            
            #colorized_depth = cv2.bitwise_and(colorized_depth, colorized_depth, mask = combinedMask)

            #Publish image data.
            #self.color_frame_publisher.publish(self.bridge.cv2_to_imgmsg(color))
            #self.depth_frame_publisher.publish(self.bridge.cv2_to_imgmsg(colorized_depth))
            depth_profile = depth_frame.get_profile()
            depth_intrin = depth_profile.as_video_stream_profile().get_intrinsics()
            indices = np.transpose(np.nonzero(depth_data))

            point_cloud = []
            point_cloud_transform = []
            for index in indices:
                depth_val = depth_data[index[0], index[1]]
                #print("Depth_val: ", depth_val)
                point = rs.rs2_deproject_pixel_to_point(depth_intrin, index, float(depth_val/1000))
                point1 = np.append(point, 1)
                transformedPoint = np.matmul(transform_mat, np.transpose(np.array(point1)))
                #print("Point, ", point)
                color_val = color[index[0], index[1]]
                rgb = struct.unpack('I', struct.pack('BBBB', color_val[2], color_val[1], color_val[0], 255))[0]
                point.append(rgb)
                transformedPoint = np.append(transformedPoint[:-1], rgb)
                #print(point)
                #print(transformedPoint)
                #print("Point+RGB," , point)
                point_cloud.append(point)
                point_cloud_transform.append(transformedPoint)
                
                

            
            header = stdMsg.Header(frame_id = 'map')
            fields = [sensorMsg.PointField(name = 'x', offset = 0, datatype = sensorMsg.PointField.FLOAT32, count =1),
                    sensorMsg.PointField(name = 'y', offset = 4, datatype = sensorMsg.PointField.FLOAT32, count = 1),
                    sensorMsg.PointField(name ='z', offset = 8, datatype = sensorMsg.PointField.FLOAT32, count = 1),
                    sensorMsg.PointField(name ='rgb', offset = 12, datatype = sensorMsg.PointField.UINT32, count = 1),
                    ]
            
            cloud = point_cloud2.create_cloud(header, fields, point_cloud)
            self.point_cloud_publisher.publish(cloud)
            cloud = point_cloud2.create_cloud(header, fields, point_cloud_transform)
            self.point_tcloud_publisher.publish(cloud)
        


    #Receive color footage aligned with depth_footage
    def color_post_depth(self, num_frames = 0):
        frame = None
        align = rs.align(rs.stream.color)
        #Perform post-processing.
        spatial = rs.spatial_filter()
        #spatial.set_option(rs.option.holes_fill, 3)
        hole_filling = rs.hole_filling_filter()
        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)
        if num_frames > 0:
            for x in range(num_frames):
                try:
                    frameset = self.pipeline.wait_for_frames()
                except:
                    print("Frames did not return in 5 seconds.")
                    break
                frameset = align.process(frameset)
                frame = frameset.get_depth_frame()
                frame = depth_to_disparity.process(frame)
                frame = spatial.process(frame)
                frame = disparity_to_depth.process(frame)
                frame = hole_filling.process(frame)
        else:
            try:
                frameset = self.pipeline.wait_for_frames()
            except:
                print("Frames did not return in 5 seconds.")
            else:
                frameset = align.process(frameset)
                frame = frameset.get_depth_frame()
                frame = depth_to_disparity.process(frame)
                frame = spatial.process(frame)
                frame = disparity_to_depth.process(frame)
                frame = hole_filling.process(frame)
        if frame is None:
            return None, None
        else:
            return frame, frameset.get_color_frame()

    #Model.

def iou_score(mask1, mask2):
    #plt.figure(1)
    #plt.title("New Mask")
    #plt.imshow(mask1)
    #plt.figure(2)
    #plt.title("Old masks.")
    #plt.imshow(mask2)
    #plt.show()
    intersection = np.logical_and(mask1, mask2).sum()
    union = np.logical_or(mask1, mask2).sum()
    iou_score = intersection / (union + 1e-6) #So we don't divide by zero.
    return iou_score 

def convert_masks(masks, labels, colors, boxes, size):
    color_masks = []
    new_labels = []
    new_masks = []
    new_boxes = []
    combinedMask = None
    combinedColorMask = None
    for i in range(len(labels)):
        color = np.full((720, 1280, 3), colors[labels[i]])
        mask = torch.round(masks[i])
        mask = np.uint8(mask)
        if (np.count_nonzero(mask) < size):
            continue
        copyFlag = 0
        for j in range(len(new_masks)):
            score = iou_score(mask, new_masks[j])
            if score > .8:
                copyFlag = 1
                break
        if copyFlag ==1:
            continue
        
        color_masks.append(cv2.bitwise_and(color, color, mask = mask))
        new_labels.append(labels[i])
        new_masks.append(mask)

        new_boxes.append(boxes[i])


        if combinedMask is None:
            combinedMask = new_masks[-1]
            combinedColorMask = color_masks[-1]
        else:
            combinedMask = combinedMask + new_masks[-1]
            combinedColorMask = combinedColorMask + color_masks[-1]
    return (combinedMask), combinedColorMask, new_labels, new_boxes

def visualize(image, masks, boxes, labels, class_names, scores, colors, score_threshold = .5, size_threshold = 100):
    #print("COLORS", colors)
    idx = np.where(scores >= score_threshold)
    masks = masks[idx]
    boxes = boxes[idx]
    scores = scores[idx]
    labels = labels[idx]
    combinedMask, combinedColorMask, new_labels, new_boxes = convert_masks(masks, labels, colors, boxes, size_threshold)
    if(combinedMask is not None):
        combinedColorMask = np.uint8(combinedColorMask*255)
        image = cv2.addWeighted(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), .5, combinedColorMask, .5, 0)
        for i in range(len(new_labels)):
            color = tuple(i*255 for i in colors[new_labels[i]])
            cv2.rectangle(image, (new_boxes[i][0], new_boxes[i][1]), (new_boxes[i][2], new_boxes[i][3]), color, 2)
            cv2.putText(image, str(class_names[new_labels[i]]), (new_boxes[i][0], new_boxes[i][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2, cv2.LINE_AA)
    return image, combinedMask, combinedColorMask, new_labels

def main():
    rclpy.init()
    print("Initializing camera.")
    camera = CameraService()
    camera.pipeline.start(camera.cfg)
    rclpy.spin(camera)


    print("Exiting...")
    camera.pipeline.stop()
    rclpy.stop()


if __name__ == '__main__':
    main() 