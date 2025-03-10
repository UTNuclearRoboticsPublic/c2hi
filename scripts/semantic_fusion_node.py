#!/usr/bin/env python3

# import time
import queue
import numpy as np
import torch
import gtsam

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time, Duration
import tf2_ros

from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel

from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

from tracking_msgs.msg import Tracks3D
from ros_audition.msg import SpeechAzSources
from foxglove_msgs.msg import SceneUpdate

from situated_hri_interfaces.msg import Auth, Comm, Comms, Identity, CategoricalDistribution, HierarchicalCommands
from situated_hri_interfaces.srv import ObjectVisRec, GestureRec

from c2hi.assignment import compute_az_match, compute_pos_match, compute_az_from_pos, compute_unit_vec_from_pos, compute_delta_az, compute_delta_pos, solve_assignment_matrix, compute_delta_vec
from c2hi.datatypes import DiscreteVariable, SemanticObject
from c2hi.output import foxglove_visualization, publish_hierarchical_commands
from c2hi.utils import pmf_to_spec, normalize_vector, load_object_params, initialize_sensors, process_sensor_update, delete_sensors, time_to_float

class SemanticTrackerNode(Node):

    def __init__(self):
        super().__init__('semantic_tracker_node')

        self.update_var_cb_group = MutuallyExclusiveCallbackGroup()
        self.pub_timer_cb_group = MutuallyExclusiveCallbackGroup()
        # self.update_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        initialize_sensors(self)
        
        # Subscribe to object tracks
        self.subscription_tracks = self.create_subscription(
            Tracks3D,
            'tracks',
            self.tracks_callback,
            10, callback_group=self.update_var_cb_group)
        self.subscription_tracks  # prevent unused variable warning

        # 2D-3D camera projection model for gesture keypoint matchind
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.camera_info_callback,
            10)
        self.subscription_camera_info  # prevent unused variable warning
        
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # Define pubs
        self.semantic_scene_pub = self.create_publisher(
            SceneUpdate,
            'semantic_scene',
            10)

        self.hierarchical_cmd_pub = self.create_publisher(
            HierarchicalCommands,
            'hierarchical_commands',
            10)
        
        # Create timers
        self.declare_parameter('update_loop_time_sec', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pub_loop_time_sec', rclpy.Parameter.Type.DOUBLE)
        self.update_loop_time_sec = self.get_parameter('update_loop_time_sec').get_parameter_value().double_value
        self.pub_loop_time_sec = self.get_parameter('pub_loop_time_sec').get_parameter_value().double_value
        self.update_timer = self.create_timer(self.update_loop_time_sec, self.update_timer_callback, callback_group=self.update_var_cb_group)
        self.pub_timer = self.create_timer(self.pub_loop_time_sec, self.pub_timer_callback, callback_group=self.pub_timer_cb_group)

        # Create clip service client
        self.clip_client = self.create_client(ObjectVisRec, 'clip_object_rec', callback_group=self.client_cb_group)
        self.clip_client.wait_for_service()
        self.clip_req = ObjectVisRec.Request()

        # Create gesture service client
        self.gesture_client = self.create_client(GestureRec, 'gesture_rec', callback_group=self.client_cb_group)
        self.gesture_client.wait_for_service()
        self.gesture_req = GestureRec.Request()

        # Create reset & reconfigure servers
        self.reset_srv = self.create_service(Empty, '~/reset', self.reset_callback)
        self.reconfigure_srv = self.create_service(Empty, '~/reconfigure', self.reconfigure_callback)
        
        # Create transform buffer/listener
        self.declare_parameter('tracker_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('mic_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('artag_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('gesture_kp_frame', rclpy.Parameter.Type.STRING)
        self.tracker_frame = self.get_parameter('tracker_frame').get_parameter_value().string_value
        self.mic_frame = self.get_parameter('mic_frame').get_parameter_value().string_value
        self.artag_frame = self.get_parameter('artag_frame').get_parameter_value().string_value
        self.gesture_kp_frame = self.get_parameter('gesture_kp_frame').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.tf_buffer.can_transform(self.mic_frame,self.tracker_frame,Duration(seconds=1)) # Block until mic -> tracker transform available
        self.tf_buffer.can_transform(self.artag_frame,self.tracker_frame,Duration(seconds=1)) # Block until artag -> tracker transform available 
        
        # Generate object att/state variable dictionary
        load_object_params(self)

        # Other parameters
        self.declare_parameter('x_label_offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('y_label_offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('z_label_offset', rclpy.Parameter.Type.DOUBLE)
        self.x_label_offset = self.get_parameter('x_label_offset').get_parameter_value().double_value
        self.y_label_offset = self.get_parameter('y_label_offset').get_parameter_value().double_value
        self.z_label_offset = self.get_parameter('z_label_offset').get_parameter_value().double_value

        # Define member variables
        self.semantic_objects = {}
        self.tracks_msg = Tracks3D()
        self.scene_out_msg = SceneUpdate()
        self.role_req_queue = queue.SimpleQueue()
        self.gesture_req_queue = queue.SimpleQueue()

        self.role_ids_to_recognize = []
        self.gesture_ids_to_recognize = []
        self.ready_to_send_role_rec_req = True
        self.ready_to_send_gesture_rec_req = True

    def reset_callback(self, _, resp):

        self.get_logger().info("Resetting")
        self.semantic_objects = {}
        self.tracks_msg = Tracks3D()
        self.scene_out_msg = SceneUpdate()

        self.tf_buffer.clear()

        self.role_req_queue = queue.SimpleQueue()
        self.gesture_req_queue = queue.SimpleQueue()
        self.role_ids_to_recognize = []
        self.gesture_ids_to_recognize = []
        self.ready_to_send_role_rec_req = True
        self.ready_to_send_gesture_rec_req = True

        return resp
    
    def reconfigure_callback(self, _, resp):

        self.get_logger().info("Reconfiguring")

        # Delete sensor subscribers
        delete_sensors(self)

        # Initialize subscribers and object parameters
        initialize_sensors(self)
        load_object_params(self)

        return resp

    def camera_info_callback(self, camera_info_msg):
        self.camera_model.fromCameraInfo(camera_info_msg)
        self.camera_info_received = True

    def vis_rec_req(self, id, stamp):
        clip_req = ObjectVisRec.Request()
        clip_req.object_id = id
        clip_req.class_string = self.semantic_objects[id].class_string
        clip_req.attributes_to_estimate = []
        clip_req.states_to_estimate = ['role']
        clip_req.estimate_comms = False
        clip_req.image = self.semantic_objects[id].image
        clip_req.stamp = stamp.to_msg()

        return clip_req

    def gesture_rec_req(self, id, stamp):

        gesture_req = GestureRec.Request()
        gesture_req.object_id = id
        gesture_req.stamp = stamp.to_msg()
        gesture_req.keypoint_data = self.semantic_objects[id].keypoint_buffer.flatten().tolist()

        return gesture_req

    def update_timer_callback(self):
    
        if 'visual' in self.role_rec_methods:

            # Check future status and update variable if needed
            if self.ready_to_send_role_rec_req==False:

                # Check if future is completed and update
                if self.role_future.done():

                    # Get role likelihood function
                    resp = self.role_future.result()

                    ### CHECK IF THE OBJECT IS BEING TRACKED AND CAN BE UPDATED
                    if resp.object_id in self.semantic_objects.keys():

                        for state in resp.states:
                            if state.variable=='role':
                                role_probs = state.probabilities

                        role_obs_idx = np.array(role_probs).argmax()

                        # Compute obs model, update state variable
                        role_var = self.semantic_objects[resp.object_id].states['role']
                        role_obs_model = gtsam.DiscreteConditional([self.sensor_dict['clip_role_rec']['role_obs_symbol'],len(self.sensor_dict['clip_role_rec']['role_obs_labels'])],[[role_var.var_symbol,len(role_var.var_labels)]],self.sensor_dict['clip_role_rec']['role_obs_spec'])
                        role_likelihood = role_obs_model.likelihood(role_obs_idx)

                        role_var.update(role_likelihood, resp.stamp)      

                        # del self.ids_to_recognize[self.ids_to_recognize.index(resp.object_id)]

                    self.ready_to_send_role_rec_req = True

            if self.ready_to_send_role_rec_req == True:

                # Check if objects need update
                now_stamp = self.get_clock().now()

                # For each object, check if state is stale. If so, send state update request.
                for id in self.semantic_objects.keys():

                    obj = self.semantic_objects[id]

                    ### CHECK IF OBJECT NEEDS AN UPDATE
                    role_not_initialized = (obj.states['role'].last_updated is None)
                    if role_not_initialized:
                        role_is_stale = True

                    else: # role was initialized
                        time_since_update = (now_stamp - Time.from_msg(obj.states['role'].last_updated))
                        sec_since_update = time_to_float(0.,time_since_update.nanoseconds)
                        role_is_stale = (sec_since_update > self.sensor_dict['clip_role_rec']['update_threshold'])

                    ### CHECK IF UPDATE REQUEST HAS ALREADY BEEN SENT
                    req_in_queue = (id in self.role_ids_to_recognize) 
                    needs_role_update = (role_not_initialized | role_is_stale) & (req_in_queue == False)

                    update_is_possible = obj.new_image_available

                    if (needs_role_update & update_is_possible):
                        self.role_req_queue.put(self.vis_rec_req(id,now_stamp))
                        self.role_ids_to_recognize.append(id)
                

                    ### SEND REQUESTS
                    if self.role_req_queue.empty() == False:

                        del self.role_ids_to_recognize[self.role_ids_to_recognize.index(id)]
                        clip_req = self.role_req_queue.get()
                        self.role_future = self.clip_client.call_async(clip_req)

                        self.ready_to_send_role_rec_req = False

        if 'gesture' in self.command_rec_methods:

            # Check future status and update variable if needed
            if self.ready_to_send_gesture_rec_req==False:

                # Check if future is completed and update
                if self.gesture_future.done():

                    # Get role likelihood function
                    resp = self.gesture_future.result()
                    self.get_logger().info("Got response %s" % str(resp))

                    ### CHECK IF THE OBJECT IS BEING TRACKED AND CAN BE UPDATED
                    if resp.object_id in self.semantic_objects.keys():

                        # Get comms symbol
                        comm_var = self.semantic_objects[resp.object_id].comms
                        self.get_logger().info("Prior comm dist: %s" % str(comm_var.probs))
                        comm_obs_idx = np.array(resp.comms.probabilities).argmax()

                        comm_obs_model = gtsam.DiscreteConditional([self.sensor_dict['lstm_gesture_rec']['comm_obs_symbol'],
                                                                    len(self.sensor_dict['lstm_gesture_rec']['comm_obs_labels'])],
                                                                    [[comm_var.var_symbol,len(comm_var.var_labels)]],
                                                                    self.sensor_dict['lstm_gesture_rec']['comm_obs_spec'])
                        comm_likelihood = comm_obs_model.likelihood(comm_obs_idx)
                        comm_var.update(comm_likelihood, resp.stamp)
                        self.get_logger().info("Posterior comm dist: %s" % str(comm_var.probs))

                        self.semantic_objects[resp.object_id].last_gesture_update = resp.stamp

                    self.ready_to_send_gesture_rec_req = True

            if self.ready_to_send_gesture_rec_req == True:

                # Check if objects need update
                now_stamp = self.get_clock().now()

                # For each object, check if state is stale. If so, send state update request.
                for id in self.semantic_objects.keys():

                    obj = self.semantic_objects[id]

                    # Check if gesture needs updated

                    ### CHECK IF GESTURE NEEDS AN UPDATE
                    gesture_not_initialized = (obj.last_gesture_update is None)
                    if gesture_not_initialized:
                        gesture_is_stale = True

                    else: # gesture was initialized
                        time_since_update = (now_stamp - Time.from_msg(obj.last_gesture_update))
                        sec_since_update = time_to_float(0.,time_since_update.nanoseconds)
                        gesture_is_stale = (sec_since_update > self.sensor_dict['lstm_gesture_rec']['update_threshold'])

                    ### CHECK IF UPDATE REQUEST HAS ALREADY BEEN SENT
                    gesture_req_in_queue = (id in self.gesture_ids_to_recognize) 
                    needs_gesture_update = (gesture_not_initialized | gesture_is_stale) & (gesture_req_in_queue == False)

                    if (needs_gesture_update):
                        self.gesture_req_queue.put(self.gesture_rec_req(id,now_stamp))
                        self.gesture_ids_to_recognize.append(id)
                
                    ### SEND REQUESTS
                    if self.gesture_req_queue.empty() == False:

                        del self.gesture_ids_to_recognize[self.gesture_ids_to_recognize.index(id)]
                        gesture_req = self.gesture_req_queue.get()
                        self.gesture_future = self.gesture_client.call_async(gesture_req)

                        self.ready_to_send_gesture_rec_req = False

    def pub_timer_callback(self):
        foxglove_visualization(self)
        publish_hierarchical_commands(self)

    def tracks_callback(self, msg):
        self.tracks_msg = msg

        tracked_object_ids = []

        for tracked_object in self.tracks_msg.tracks:

            if tracked_object.class_string not in self.objects_of_interest:
                continue

            tracked_object_ids.append(tracked_object.track_id)

            # Initialize object and add to dict if not currently tracked
            if tracked_object.track_id not in self.semantic_objects.keys():
                self.semantic_objects[tracked_object.track_id] = SemanticObject(tracked_object, self.object_params[tracked_object.class_string])
            else:
                # Update existing track
                self.semantic_objects[tracked_object.track_id].update_spatial_state(tracked_object)

        # Remove untracked objects
        semantic_objects_to_remove = []
        for semantic_obj_key in self.semantic_objects.keys():
            if semantic_obj_key not in tracked_object_ids:
                semantic_objects_to_remove.append(semantic_obj_key)

        for key in semantic_objects_to_remove:
            self.semantic_objects.pop(key)

    def ar_callback(self, msg, sensor_name):

        if process_sensor_update(self.sensor_dict[sensor_name])==False:
            return

        # Compute number of comms and role markers to assign
        comm_markers = {}
        role_markers = {}

        # Compute TF
        artag_tracker_tf = self.tf_buffer.lookup_transform(self.tracker_frame,self.artag_frame,time=rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1.))

        for marker in msg.markers:

            if marker.id in self.sensor_dict[sensor_name]['ar_tag_dict'].keys():

                # Get semantic meaning, word type
                type = self.sensor_dict[sensor_name]['ar_tag_dict'][marker.id]['type']
                word = self.sensor_dict[sensor_name]['ar_tag_dict'][marker.id]['word']

                # Compute marker position in tracker frame
                pos_in_ar_frame = PointStamped()
                pos_in_ar_frame.point.x = marker.pose.pose.position.x
                pos_in_ar_frame.point.y = marker.pose.pose.position.y
                pos_in_ar_frame.point.z = marker.pose.pose.position.z
                pos_in_tracker_frame = do_transform_point(pos_in_ar_frame,artag_tracker_tf)
                marker_pos = np.array([pos_in_tracker_frame.point.x,pos_in_tracker_frame.point.y,pos_in_tracker_frame.point.z])

                if type == 'command':
                    comm_markers[marker.id] = {}
                    comm_markers[marker.id]['pos'] = marker_pos
                    comm_markers[marker.id]['marker'] = marker

                elif type == 'role':
                    role_markers[marker.id] = {}
                    role_markers[marker.id]['pos'] = marker_pos
                    role_markers[marker.id]['marker'] = marker

        # Initialize, populate, and solve comm assignment matrix
        if 'artag' in self.command_rec_methods:
            comm_assignment_matrix = np.zeros((len(comm_markers.keys()),len(self.semantic_objects.keys())))
            for marker_idx, marker_key in enumerate(comm_markers.keys()):

                for jj, object_key in enumerate(self.semantic_objects.keys()):
                
                    object = self.semantic_objects[object_key]
                    marker_pos = comm_markers[marker_key]['pos']
                    object_pos = np.array([object.pos_x,object.pos_y,object.pos_z])
                    delta_pos = compute_delta_pos(marker_pos, object_pos)
                    comm_assignment_matrix[marker_idx,jj] += delta_pos

            comm_assignments = solve_assignment_matrix('greedy', comm_assignment_matrix, self.sensor_dict[sensor_name]['match_threshold'])

            # Handle comm matches
            for assignment in comm_assignments:

                object_idx = assignment[1]
                object_key = list(self.semantic_objects.keys())[object_idx]
                marker_idx = assignment[0]
                marker_id = list(comm_markers.keys())[marker_idx]
                comm_word = self.sensor_dict[sensor_name]['ar_tag_dict'][marker_id]['word']

                # Get comms symbol
                comm_var = self.semantic_objects[object_key].comms
                comm_obs_idx = self.sensor_dict[sensor_name]['comm_obs_labels'].index(comm_word)

                comm_obs_model = gtsam.DiscreteConditional([self.sensor_dict[sensor_name]['comm_obs_symbol'],len(self.sensor_dict[sensor_name]['comm_obs_labels'])],[[comm_var.var_symbol,len(comm_var.var_labels)]],self.sensor_dict[sensor_name]['comm_obs_spec'])
                comm_likelihood = comm_obs_model.likelihood(comm_obs_idx)

                comm_var.update(comm_likelihood, msg.header.stamp)

            # Handle objects with no speech
            for jj, object_key in enumerate(self.semantic_objects.keys()):

                if jj not in comm_assignments[:,1]: # If track is unmatched, handle it as a missed detection

                    comm_word = 'none'

                    # Get comms symbol
                    comm_var = self.semantic_objects[object_key].comms
                    comm_obs_idx = self.sensor_dict[sensor_name]['comm_obs_labels'].index(comm_word)

                    comm_obs_model = gtsam.DiscreteConditional([self.sensor_dict[sensor_name]['comm_obs_symbol'],len(self.sensor_dict[sensor_name]['comm_obs_labels'])],[[comm_var.var_symbol,len(comm_var.var_labels)]],self.sensor_dict[sensor_name]['comm_obs_spec'])
                    comm_likelihood = comm_obs_model.likelihood(comm_obs_idx)

                    comm_var.update(comm_likelihood, msg.header.stamp)

        if 'artag' in self.role_rec_methods:
            # Initialize, populate, and solve role assignment matrix
            role_assignment_matrix = np.zeros((len(role_markers.keys()),len(self.semantic_objects.keys())))
            for marker_idx, marker_key in enumerate(role_markers.keys()):
                for jj, object_key in enumerate(self.semantic_objects.keys()):

                    object = self.semantic_objects[object_key]
                    marker_pos = role_markers[marker_key]['pos']
                    object_pos = np.array([object.pos_x,object.pos_y,object.pos_z])
                    delta_pos = compute_delta_pos(marker_pos, object_pos)
                    role_assignment_matrix[marker_idx,jj] += delta_pos

            role_assignments = solve_assignment_matrix('greedy', role_assignment_matrix, self.sensor_dict[sensor_name]['match_threshold'])

            # Handle role matches
            for assignment in role_assignments:

                object_idx = assignment[1]
                object_key = list(self.semantic_objects.keys())[object_idx]
                marker_idx = assignment[0]
                marker_id = list(role_markers.keys())[marker_idx]
                role_word = self.sensor_dict[sensor_name]['ar_tag_dict'][marker_id]['word']

                # Get variable symbol
                role_var = self.semantic_objects[object_key].states['role']
                role_obs_idx = self.sensor_dict[sensor_name]['role_obs_labels'].index(role_word)

                # Compute obs model, update state variable
                role_obs_model = gtsam.DiscreteConditional([self.sensor_dict[sensor_name]['role_obs_symbol'],len(self.sensor_dict[sensor_name]['role_obs_labels'])],[[role_var.var_symbol,len(role_var.var_labels)]],self.sensor_dict[sensor_name]['role_obs_spec'])
                role_likelihood = role_obs_model.likelihood(role_obs_idx)

                role_var.update(role_likelihood, msg.header.stamp)

    def speech_az_callback(self, msg, sensor_name):
        if 'verbal' in self.command_rec_methods:

            # Compute the assignment matrix
            assignment_matrix = np.zeros((len(msg.sources),len(self.semantic_objects.keys())))

            for ii, speech_src in enumerate(msg.sources):
                for jj, object_key in enumerate(self.semantic_objects.keys()):
                    object = self.semantic_objects[object_key]

                    semantic_object_az = compute_az_from_pos(self.tf_buffer,msg.header.frame_id,self.tracker_frame,object)
                    delta_az = compute_delta_az(speech_src.azimuth, semantic_object_az)
                    assignment_matrix[ii,jj] += delta_az

            # Solve assignment matrix
            comm_assignments = solve_assignment_matrix('greedy', assignment_matrix, self.sensor_dict[sensor_name]['match_threshold'])

            # Handle comm matches
            for assignment in comm_assignments:

                object_idx = assignment[1]
                object_key = list(self.semantic_objects.keys())[object_idx]
                speech_idx = assignment[0]
                comm_word = msg.sources[speech_idx].transcript

                if comm_word not in self.sensor_dict[sensor_name]['comm_obs_labels']:
                    continue

                # Get comms symbol
                comm_var = self.semantic_objects[object_key].comms
                comm_obs_idx = self.sensor_dict[sensor_name]['comm_obs_labels'].index(comm_word)

                comm_obs_model = gtsam.DiscreteConditional([self.sensor_dict[sensor_name]['comm_obs_symbol'],len(self.sensor_dict[sensor_name]['comm_obs_labels'])],[[comm_var.var_symbol,len(comm_var.var_labels)]],self.sensor_dict[sensor_name]['comm_obs_spec'])
                comm_likelihood = comm_obs_model.likelihood(comm_obs_idx)

                comm_var.update(comm_likelihood, msg.header.stamp)

            # Handle objects with no speech
            for jj, object_key in enumerate(self.semantic_objects.keys()):

                if jj not in comm_assignments[:,1]: # If track is unmatched, handle it as a missed detection

                    comm_word = ''

                    # Get comms symbol
                    comm_var = self.semantic_objects[object_key].comms
                    comm_obs_idx = self.sensor_dict[sensor_name]['comm_obs_labels'].index(comm_word)

                    comm_obs_model = gtsam.DiscreteConditional([self.sensor_dict[sensor_name]['comm_obs_symbol'],len(self.sensor_dict[sensor_name]['comm_obs_labels'])],[[comm_var.var_symbol,len(comm_var.var_labels)]],self.sensor_dict[sensor_name]['comm_obs_spec'])
                    comm_likelihood = comm_obs_model.likelihood(comm_obs_idx)

                    comm_var.update(comm_likelihood, msg.header.stamp)

    def gesture_kp_callback(self, msg, sensor_name):

        if 'gesture' in self.command_rec_methods:

            if self.camera_info_received == False:
                return

            # Compute the assignment matrix
            assignment_matrix = np.zeros((len(msg.keypoints),len(self.semantic_objects.keys())))

            for ii, gesture_kps in enumerate(msg.keypoints):
                for jj, object_key in enumerate(self.semantic_objects.keys()):
                    object = self.semantic_objects[object_key]

                    vec_to_obj = compute_unit_vec_from_pos(self.tf_buffer,self.gesture_kp_frame,self.tracker_frame,object)
                    vec_to_kp_centroid = self.camera_model.projectPixelTo3dRay((gesture_kps.bbox_mean_x,gesture_kps.bbox_mean_y))
                    delta_vec = compute_delta_vec(vec_to_obj, vec_to_kp_centroid)
                    assignment_matrix[ii,jj] += delta_vec

            # Solve assignment matrix
            comm_assignments = solve_assignment_matrix('greedy', assignment_matrix, self.sensor_dict[sensor_name]['match_threshold'])

            # Handle comm matches
            for assignment in comm_assignments:

                object_idx = assignment[1]
                object_key = list(self.semantic_objects.keys())[object_idx]
                kp_idx = assignment[0]

                # Add keypoints to the person's keypoint buffer
                self.semantic_objects[object_key].keypoint_buffer = torch.roll(self.semantic_objects[object_key].keypoint_buffer, shifts = -1, dims=1)
                self.semantic_objects[object_key].keypoint_buffer[:,-1,:] = torch.Tensor(msg.keypoints[kp_idx].keypoint_data)

            # Handle objects with no keypoints
            for jj, object_key in enumerate(self.semantic_objects.keys()):

                if jj not in comm_assignments[:,1]: # If track is unmatched, handle it as a missed detection

                    self.semantic_objects[object_key].keypoint_buffer = torch.roll(self.semantic_objects[object_key].keypoint_buffer, shifts = -1, dims=1)
                    self.semantic_objects[object_key].keypoint_buffer[:,-1,:] = torch.zeros(51) # TODO - make this a param

            # NOTE
            # Shifts = -1, buffer[:, -1, :] puts new points at bottom row, old points at top row
            # Shifts = 1, buffer[:, 0, :] puts new points at top row, old points at bottom row 

def main(args=None):
    rclpy.init(args=args)

    semantic_tracker_node = SemanticTrackerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(semantic_tracker_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        semantic_tracker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
