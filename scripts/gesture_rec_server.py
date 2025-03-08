#!/usr/bin/env python3
import sys

import torch

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import Empty

from situated_hri_interfaces.msg import CategoricalDistribution
from situated_hri_interfaces.srv import GestureRec

# Find gesture recognition package and import model architecture
gesture_rec_dir = get_package_share_directory('gesture_recognition_ros2')
sys.path.append(f'{gesture_rec_dir}/include') 
from hri_cacti_gestures.classifier import LSTMGestureClassifier

class GestureRecServer(Node):

    def __init__(self):

        # ROS objects
        super().__init__('gesture_rec_server')

        # ROS objects
        self.srv = self.create_service(GestureRec, 'gesture_rec', self.gesture_rec_callback)
        self.reconfigure_srv = self.create_service(Empty, '~/reconfigure', self.reconfigure_callback)

        # Gesture data parameters 
        self.declare_parameter('window_length', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('command_list', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('command_idx', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('model_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('keypoints_per_frame', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('keypoint_dimension', rclpy.Parameter.Type.INTEGER)
        
        self.keypoints_per_frame = self.get_parameter('keypoints_per_frame').get_parameter_value().integer_value
        self.keypoint_dimension = self.get_parameter('keypoint_dimension').get_parameter_value().integer_value
        self.window_length = self.get_parameter('window_length').get_parameter_value().integer_value 
        self.command_list = self.get_parameter('command_list').get_parameter_value().string_array_value
        self.command_idx = self.get_parameter('command_idx').get_parameter_value().integer_array_value 
        self.command_idx_tensor = torch.tensor(self.command_idx)
        self.model_path = f"{gesture_rec_dir}/{self.get_parameter('model_path').get_parameter_value().string_value}"

        self.model_input_dim = self.keypoints_per_frame*self.keypoint_dimension
        self.model_output_dim = len(self.command_idx)

        # Torch objects
        self.torch_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = LSTMGestureClassifier(self.model_input_dim, self.model_output_dim)
        self.model.load_state_dict(torch.load(self.model_path,map_location='cuda:0'))
        self.model.to(self.torch_device)
        self.input_tensor = torch.zeros(1, self.window_length, self.model_input_dim)
        self.input_tensor.to(self.torch_device)

    def gesture_rec_callback(self, req, resp):

        self.input_tensor = torch.Tensor(req.keypoint_data).reshape(1, self.window_length, self.model_input_dim).to(self.torch_device)
        self.input_tensor = self.input_tensor.to(torch.float32)
        output_tensor = self.model(self.input_tensor)

        resp.stamp = req.stamp
        resp.object_id = req.object_id
        resp.comms.variable = "gesture"
        resp.comms.categories = self.command_list
        
        resp.comms.probabilities = output_tensor[0].cpu().tolist()

        return resp

    def reconfigure_callback(self, _, resp):

        self.get_logger().info("Reconfiguring")

        self.keypoints_per_frame = self.get_parameter('keypoints_per_frame').get_parameter_value().integer_value
        self.keypoint_dimension = self.get_parameter('keypoint_dimension').get_parameter_value().integer_value
        self.window_length = self.get_parameter('window_length').get_parameter_value().integer_value 
        self.command_list = self.get_parameter('command_list').get_parameter_value().string_array_value
        self.command_idx = self.get_parameter('command_idx').get_parameter_value().integer_array_value 
        self.command_idx_tensor = torch.tensor(self.command_idx)
        self.model_path = f"{gesture_rec_dir}/{self.get_parameter('model_path').get_parameter_value().string_value}"

        self.model_input_dim = self.keypoints_per_frame*self.keypoint_dimension
        self.model_output_dim = len(self.command_idx)

        # Torch objects
        self.torch_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = LSTMGestureClassifier(self.model_input_dim, self.model_output_dim)
        self.model.load_state_dict(torch.load(self.model_path,map_location='cuda:0'))
        self.model.to(self.torch_device)
        self.input_tensor = torch.zeros(1, self.window_length, self.model_input_dim)
        self.input_tensor.to(self.torch_device)

        return resp


def main(args=None):
    rclpy.init(args=args)

    gesture_rec_server = GestureRecServer()
    rclpy.spin(gesture_rec_server)

    gesture_rec_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()