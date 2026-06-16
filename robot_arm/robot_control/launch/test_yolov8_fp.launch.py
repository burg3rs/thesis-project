# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from isaac_ros_examples import IsaacROSLaunchFragment
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Expected number of input messages in 1 second
INPUT_IMAGES_EXPECT_FREQ = 30
# Number of input messages to be dropped in 1 second
INPUT_IMAGES_DROP_FREQ = 28

YOLOV8_MODEL_INPUT_SIZE = 640

VISUALIZATION_DOWNSCALING_FACTOR = 10

REFINE_MODEL_PATH = '/tmp/refine_model.onnx'
REFINE_ENGINE_PATH = '/tmp/refine_trt_engine.plan'
SCORE_MODEL_PATH = '/tmp/score_model.onnx'
SCORE_ENGINE_PATH = '/tmp/score_trt_engine.plan'


class IsaacROSFoundationPoseLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:

        # Drop node parameters
        input_images_expect_freq = LaunchConfiguration('input_images_expect_freq')
        input_images_drop_freq = LaunchConfiguration('input_images_drop_freq')
        # FoundationPose parameters
        mesh_file_path = LaunchConfiguration('mesh_file_path')
        texture_path = LaunchConfiguration('texture_path')
        refine_model_file_path = LaunchConfiguration('refine_model_file_path')
        refine_engine_file_path = LaunchConfiguration('refine_engine_file_path')
        score_model_file_path = LaunchConfiguration('score_model_file_path')
        score_engine_file_path = LaunchConfiguration('score_engine_file_path')
        # YOLOv8 parameters
        input_width = interface_specs['camera_resolution']['width']
        input_height = interface_specs['camera_resolution']['height']
        
        input_to_YOLOv8_ratio = input_width / YOLOV8_MODEL_INPUT_SIZE

        # Yolov8 parameters
        model_file_path = LaunchConfiguration('model_file_path')
        engine_file_path = LaunchConfiguration('engine_file_path')
        input_tensor_names = LaunchConfiguration('input_tensor_names')
        input_binding_names = LaunchConfiguration('input_binding_names')
        output_tensor_names = LaunchConfiguration('output_tensor_names')
        output_binding_names = LaunchConfiguration('output_binding_names')
        verbose = LaunchConfiguration('verbose')
        force_engine_update = LaunchConfiguration('force_engine_update')

        # YOLOv8 Decoder parameters
        confidence_threshold = LaunchConfiguration('confidence_threshold')
        nms_threshold = LaunchConfiguration('nms_threshold')
        
        return {
            # Drops input_images_expect_freq out of input_images_drop_freq input messages
            'drop_node':  ComposableNode(
                name='drop_node',
                package='isaac_ros_nitros_topic_tools',
                plugin='nvidia::isaac_ros::nitros::NitrosCameraDropNode',
                parameters=[{
                    'X': input_images_drop_freq,
                    'Y': input_images_expect_freq,
                    'mode': 'mono+depth',
                    'depth_format_string': 'nitros_image_mono16'
                }],
                remappings=[
                    ('image_1', 'camera/camera/color/image_raw'),
                    ('camera_info_1', '/camera/camera/colorcamera_info'),
                    ('depth_1', '/camera/camera/depth/image_rect_raw'),
                    ('image_1_drop', 'rgb/image_rect_color'),
                    ('camera_info_1_drop', 'rgb/camera_info'),
                    ('depth_1_drop', 'depth_image_drop'),
                ]
            ),

	    'convert_metric_node': ComposableNode(
		package='isaac_ros_depth_image_proc',
		plugin='nvidia::isaac_ros::depth_image_proc::ConvertMetricNode',
		remappings=[
		    ('image_raw', 'depth_image_drop'),
		    ('image', 'depth_image')
	        ]),

            # YOLOv8 objection detection pipeline
            'tensor_rt_node': ComposableNode(
                name='tensor_rt',
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                parameters=[{
                    'model_file_path': model_file_path,
                    'engine_file_path': engine_file_path,
                    'output_binding_names': output_binding_names,
                    'output_tensor_names': output_tensor_names,
                    'input_tensor_names': input_tensor_names,
                    'input_binding_names': input_binding_names,
                    'verbose': verbose,
                    'force_engine_update': force_engine_update
                }]
            ),
            'yolov8_decoder_node': ComposableNode(
                name='yolov8_decoder_node',
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                parameters=[{
                    'confidence_threshold': confidence_threshold,
                    'nms_threshold': nms_threshold,
                }]	
            ),

            # Create a binary segmentation mask from a Detection2DArray published by RT-DETR.
            # The segmentation mask is of size
            # int(IMAGE_WIDTH/input_to_RT_DETR_ratio) x int(IMAGE_HEIGHT/input_to_RT_DETR_ratio)
            'detection2_d_to_mask_node': ComposableNode(
                name='detection2_d_to_mask',
                package='isaac_ros_foundationpose',
                plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
                parameters=[{
                    'mask_width': 640,
                    'mask_height': 640}],
                remappings=[('detection2_d_array', 'detections_output'),
                            ('segmentation', 'yolov8_segmentation')]),
                            
             'crop_mask_node': ComposableNode(
	        name='crop_mask_node',
	        package='isaac_ros_image_proc',
	        plugin='nvidia::isaac_ros::image_proc::CropNode',
	        parameters=[{
		'input_width': 640,
		'input_height': 640,
		'crop_width': input_width,   # Add crop width
		'crop_height': input_height, # Add crop height
		'crop_mode': 'CENTER',           # Add crop mode (adjust as needed)
	    }],
	    remappings=[
		('image', 'yolov8_segmentation'),
		('camera_info', 'rgb/camera_info'),
		('crop/image', 'segmentation'),  # Fix remapping to match output topic
		('crop/camera_info', 'camera_info_segmentation')]),

            'resize_left_viz': ComposableNode(
                name='resize_left_viz',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'input_width': input_width,
                    'input_height': input_height,
                    'output_width': int(input_width/VISUALIZATION_DOWNSCALING_FACTOR) * 2,
                    'output_height': int(input_height/VISUALIZATION_DOWNSCALING_FACTOR) * 2,
                    'keep_aspect_ratio': False,
                    'encoding_desired': 'rgb8',
                    'disable_padding': False
                }],
                remappings=[
                    ('image', 'rgb/image_rect_color'),
                    ('camera_info', 'rgb/camera_info'),
                    ('resize/image', 'rgb/image_rect_color_viz'),
                    ('resize/camera_info', 'rgb/camera_info_viz')
                ]
            ),

            'foundationpose_node': ComposableNode(
                name='foundationpose_node',
                package='isaac_ros_foundationpose',
                plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
                parameters=[{
                    'mesh_file_path': mesh_file_path,
                    'texture_path': texture_path,

                    'refine_model_file_path': refine_model_file_path,
                    'refine_engine_file_path': refine_engine_file_path,
                    'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                    'refine_input_binding_names': ['input1', 'input2'],
                    'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
                    'refine_output_binding_names': ['output1', 'output2'],

                    'score_model_file_path': score_model_file_path,
                    'score_engine_file_path': score_engine_file_path,
                    'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                    'score_input_binding_names': ['input1', 'input2'],
                    'score_output_tensor_names': ['output_tensor'],
                    'score_output_binding_names': ['output1'],
                }],
                remappings=[
                    ('pose_estimation/depth_image', 'depth_image'),
                    ('pose_estimation/image', 'rgb/image_rect_color'),
                    ('pose_estimation/camera_info', 'rgb/camera_info'),
                    ('pose_estimation/segmentation', 'segmentation'),
                    ('pose_estimation/output', 'output')]
            ),
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> \
            Dict[str, launch.actions.OpaqueFunction]:
        
        network_image_width = LaunchConfiguration('network_image_width')
        network_image_height = LaunchConfiguration('network_image_height')
        image_mean = LaunchConfiguration('image_mean')
        image_stddev = LaunchConfiguration('image_stddev')

        encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

        return {
            'input_images_expect_freq': DeclareLaunchArgument(
                'input_images_expect_freq',
                default_value=str(INPUT_IMAGES_EXPECT_FREQ),
                description='Expected number of input messages in 1 second'),

            'input_images_drop_freq': DeclareLaunchArgument(
                'input_images_drop_freq',
                default_value=str(INPUT_IMAGES_DROP_FREQ),
                description='Number of input messages to be dropped in 1 second'),

            'mesh_file_path': DeclareLaunchArgument(
                'mesh_file_path',
                default_value='',
                description='The absolute file path to the mesh file'),

            'texture_path': DeclareLaunchArgument(
                'texture_path',
                default_value='',
                description='The absolute file path to the texture map'),

            'refine_model_file_path': DeclareLaunchArgument(
                'refine_model_file_path',
                default_value=REFINE_MODEL_PATH,
                description='The absolute file path to the refine model'),

            'refine_engine_file_path': DeclareLaunchArgument(
                'refine_engine_file_path',
                default_value=REFINE_ENGINE_PATH,
                description='The absolute file path to the refine trt engine'),

            'score_model_file_path': DeclareLaunchArgument(
                'score_model_file_path',
                default_value=SCORE_MODEL_PATH,
                description='The absolute file path to the score model'),

            'score_engine_file_path': DeclareLaunchArgument(
                'score_engine_file_path',
                default_value=SCORE_ENGINE_PATH,
                description='The absolute file path to the score trt engine'),
                
	    'network_image_width': DeclareLaunchArgument(
               'network_image_width',
                default_value='640',
                description='The input image width that the network expects'),
                
            'network_image_height': DeclareLaunchArgument(
                'network_image_height',
                default_value='640',
                description='The input image height that the network expects'),
                
            'image_mean': DeclareLaunchArgument(
                'image_mean',
                default_value='[0.0, 0.0, 0.0]',
                description='The mean for image normalization'),
                
            'image_stddev': DeclareLaunchArgument(
                'image_stddev',
                default_value='[1.0, 1.0, 1.0]',
                description='The standard deviation for image normalization'),
                
            'model_file_path': DeclareLaunchArgument(
                'model_file_path',
                description='The absolute file path to the ONNX file'),
                
            'engine_file_path': DeclareLaunchArgument(
                'engine_file_path',
                default_value='',
                description='The absolute file path to the TensorRT engine file'),
                
            'input_tensor_names': DeclareLaunchArgument(
                'input_tensor_names',
                default_value='["input_tensor"]',
                description='A list of tensor names to bound to the specified input binding names'),
                
            'input_binding_names': DeclareLaunchArgument(
                'input_binding_names',
                default_value='["images"]',
                description='A list of input tensor binding names (specified by model)'),
                
            'output_tensor_names': DeclareLaunchArgument(
                'output_tensor_names',
                default_value='["output_tensor"]',
                description='A list of tensor names to bound to the specified output binding names'),
                
            'output_binding_names': DeclareLaunchArgument(
                'output_binding_names',
                default_value='["output0"]',
                description='A list of output tensor binding names (specified by model)'),
                
            'verbose': DeclareLaunchArgument(
                'verbose',
                default_value='False',
                description='Whether TensorRT should verbosely log or not'),
                
            'force_engine_update': DeclareLaunchArgument(
                'force_engine_update',
                default_value='False',
                description='Whether TensorRT should update the TensorRT engine file or not'),
                
            'confidence_threshold': DeclareLaunchArgument(
                'confidence_threshold',
                default_value='0.25',
                description='Confidence threshold to filter candidate detections during NMS'),
                
            'nms_threshold': DeclareLaunchArgument(
                'nms_threshold',
                default_value='0.45',
                description='NMS IOU threshold'),

            'yolov8_encoder_launch': IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
                ),
                launch_arguments={
                    'input_image_width': str(interface_specs['camera_resolution']['width']),
                    'input_image_height': str(interface_specs['camera_resolution']['height']),
                    'network_image_width': network_image_width,
                    'network_image_height': network_image_height,
                    'image_mean': image_mean,
                    'image_stddev': image_stddev,
                    'attach_to_shared_component_container': 'True',
                    'component_container_name': '/isaac_ros_examples/container',
                    'dnn_image_encoder_namespace': 'yolov8_encoder',
                    'image_input_topic': '/rgb/image_rect_color',
                    'camera_info_input_topic': '/rgb/camera_info',
                    'tensor_output_topic': '/tensor_pub',
                }.items(),
            ),	
        }


def generate_launch_description():
    foundationpose_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='foundationpose_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=IsaacROSFoundationPoseLaunchFragment
        .get_composable_nodes().values(),
        output='screen'
    )

    realsense_camera_node = Node(
        name='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        #plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[{
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_color': True,
            'enable_depth': True,
            'align_depth': True,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',#'1280x720x15',
            'depth_module.infra_profile': '640x480x30',
            'enable_gyro': False,
            'enable_accel': False,
            'gyro_fps': 200,
        }],
    )


    return launch.LaunchDescription(
        [foundationpose_container, realsense_camera_node] +
        IsaacROSFoundationPoseLaunchFragment.get_launch_actions().values())
