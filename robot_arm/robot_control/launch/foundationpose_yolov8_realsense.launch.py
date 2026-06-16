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

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Expected number of Realsense messages in 1 second
INPUT_IMAGES_DROP_FREQ = 0
INPUT_IMAGES_EXPECT_FREQ = 15

REALSENSE_IMAGE_WIDTH = 640
REALSENSE_IMAGE_HEIGHT = 480

VISUALIZATION_DOWNSCALING_FACTOR = 10

YOLOV8_MODEL_INPUT_SIZE = 640

#foundation pose location
MESH_FILE_NAME = '/workspaces/isaac_ros-dev/src/models/tennis_ball/tennis_ball_scaled.obj'
TEXTURE_MAP_NAME = '/workspaces/isaac_ros-dev/src/models/tennis_ball/Texture.png'

REFINE_MODEL_NAME = '/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/refine_model.onnx'
REFINE_ENGINE_NAME = '/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan'

SCORE_MODEL_NAME = '/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/score_model.onnx'
SCORE_ENGINE_NAME = '/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/score_trt_engine.plan'
#yolov8 model locations
YOLOV8_MODEL_PATH = '/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/yolov8s.onnx'
YOLOV8_ENGINE_PATH = '/workspaces/isaac_ros-dev/isaac_ros_assets/models/yolov8/yolov8s.plan'


def generate_launch_description():
    """Generate launch description for testing relevant nodes."""
    rviz_config_path = os.path.join(
        get_package_share_directory('isaac_ros_foundationpose'),
        'rviz', 'foundationpose.rviz')

    launch_args = [

		DeclareLaunchArgument(
            'input_images_drop_freq',
            default_value=str(INPUT_IMAGES_DROP_FREQ),
            description='Expected number of Realsense messages in 1 second'),

       DeclareLaunchArgument(
            'input_images_expect_freq',
            default_value=str(INPUT_IMAGES_EXPECT_FREQ),
            description='Expected number of input messages in 1 second'),

        DeclareLaunchArgument(
            'mesh_file_path',
            default_value=MESH_FILE_NAME,
            description='The absolute file path to the mesh file'),

        DeclareLaunchArgument(
            'texture_path',
            default_value=TEXTURE_MAP_NAME,
            description='The absolute file path to the texture map'),

        DeclareLaunchArgument(
            'refine_model_file_path',
            default_value=REFINE_MODEL_NAME,
            description='The absolute file path to the refine model'),

        DeclareLaunchArgument(
            'refine_engine_file_path',
            default_value=REFINE_ENGINE_NAME,
            description='The absolute file path to the refine trt engine'),

        DeclareLaunchArgument(
            'score_model_file_path',
            default_value=SCORE_MODEL_NAME,
            description='The absolute file path to the score model'),

        DeclareLaunchArgument(
            'score_engine_file_path',
            default_value=SCORE_ENGINE_NAME,
            description='The absolute file path to the score trt engine'),

        DeclareLaunchArgument(
            'yolov8_model_file_path',
            default_value=YOLOV8_MODEL_PATH,),
        
        DeclareLaunchArgument(
            'yolov8_engine_file_path',
            default_value=YOLOV8_ENGINE_PATH,),

        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.25',),

        DeclareLaunchArgument(
            'nms_threshold',
            default_value='0.45',),

        DeclareLaunchArgument(
            'mask_height',
            default_value='480',
            description='The height of the mask generated from the bounding box'),

        DeclareLaunchArgument(
            'mask_width',
            default_value='640',
            description='The width of the mask generated from the bounding box'),

        DeclareLaunchArgument(
            'launch_bbox_to_mask',
            default_value='False',
            description='Flag to enable bounding box to mask converter'),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='True',
            description='Flag to enable Rviz2 launch'),

    ]

    #foundatio pose
    mesh_file_path = LaunchConfiguration('mesh_file_path')
    texture_path = LaunchConfiguration('texture_path')
    refine_model_file_path = LaunchConfiguration('refine_model_file_path')
    refine_engine_file_path = LaunchConfiguration('refine_engine_file_path')
    score_model_file_path = LaunchConfiguration('score_model_file_path')
    score_engine_file_path = LaunchConfiguration('score_engine_file_path')
    
    #yolov8
    yolov8_model_file_path = LaunchConfiguration('yolov8_model_file_path')
    yolov8_engine_file_path = LaunchConfiguration('yolov8_engine_file_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')
   
    input_images_drop_freq = LaunchConfiguration('input_images_drop_freq')
    input_images_expect_freq = LaunchConfiguration('input_images_expect_freq')
     
    mask_height = LaunchConfiguration('mask_height')
    mask_width = LaunchConfiguration('mask_width')
    input_width = 640
    input_height = 480
    input_to_YOLOv8_ratio = input_width / YOLOV8_MODEL_INPUT_SIZE


    launch_rviz = LaunchConfiguration('launch_rviz')

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
 
    yolov8_encoder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': '640',
            'input_image_height': '480',
            'network_image_width': '640',
            'network_image_height': '640',
            'image_mean': '[0.0, 0.0, 0.0]',
            'image_stddev': '[1.0, 1.0, 1.0]',
            'attach_to_shared_component_container': 'True',
            'component_container_name': '/foundationpose_container',
            'dnn_image_encoder_namespace': 'yolov8_encoder',
 
            # Remap RealSense image here
            'image_input_topic': '/camera/camera/color/image_raw',
            'camera_info_input_topic': '/camera/camera/color/camera_info',
 
            'tensor_output_topic': '/tensor_pub',
            'input_encoding': 'rgb8',
        }.items()
    )

    drop_node =  ComposableNode(
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
            ('image_1', '/camera/camera/color/image_raw'),
            ('camera_info_1', '/camera/camera/color/camera_info'),
            ('depth_1', '/camera/camera/depth/image_rect_raw'),
            ('image_1_drop', '/rgb/image_rect_color'),
            ('camera_info_1_drop', '/rgb/camera_info'),
            ('depth_1_drop', '/depth_image_drop'),
        ]
    )
    convert_metric_node = ComposableNode(
        package='isaac_ros_depth_image_proc',
        plugin='nvidia::isaac_ros::depth_image_proc::ConvertMetricNode',
        remappings=[
            ('image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('image', '/depth_image')
        ]
    )

    camera_config_file = os.path.join(
        get_package_share_directory('robot_control'),
        'params',
        'realsense_config.json'
    )


    realsense_camera_node = Node(
        name='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        #plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[{
            'json_file_path': camera_config_file,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_color': True,
            'enable_depth': True,
            #'align_depth': True,
            'align_depth.enable': True,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',#'1280x720x15',
            'depth_module.infra_profile': '640x480x30',
            'enable_gyro': False,
            'enable_accel': False,
            'gyro_fps': 200,
        }],
    )


    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': LaunchConfiguration('yolov8_model_file_path'),
            'engine_file_path': LaunchConfiguration('yolov8_engine_file_path'),
            'force_engine_update':False,
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['output0']
        }]
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': 0.85,
            'nms_threshold': 0.45,
            'desiered_class_id':'32',
        }]
    )

    foundationpose_node = ComposableNode(
        name='foundationpose',
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
            ('pose_estimation/depth_image', '/depth_image'),
            ('pose_estimation/image', '/camera/camera/color/image_raw'),
            ('pose_estimation/camera_info', '/camera/camera/color/camera_info'),
            ('pose_estimation/segmentation', '/segmentation'),
            ('pose_estimation/output', 'output')])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(launch_rviz))

    detection2_d_array_filter_node = ComposableNode(
        name='detection2_d_array_filter',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Detection2DArrayFilter',
        parameters=[{
            'desired_class_id':'32'}],
        remappings=[('detection2_d_array', '/detections_output')]
    )

    detection2_d_to_mask_node = ComposableNode(
        name='detection2_d_to_mask',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
        parameters=[{
            'mask_width': 640,
            'mask_height': 640
        }],
        remappings=[('segmentation', '/yolov8_segmentation'),]
    )

    crop_mask_node = ComposableNode(
        name='crop_mask_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::CropNode',
        parameters=[{
            'input_width': 640,
            'input_height': 640,
            'crop_width': 640,#REALSENSE_IMAGE_WIDTH,
            'crop_height': 480,#REALSENSE_IMAGE_HEIGHT,
            'crop_mode':'CENTER',
            'encoding_desired': 'mono8'
            #'keep_aspect_ratio': False,
            #'disable_padding': False
        }],
        remappings=[
            ('image', '/yolov8_segmentation'),
            ('camera_info', '/camera/camera/color/camera_info'),
            ('crop/image', '/segmentation'),
            ('crop/camera_info', '/camera_info_segmentation')
        ]
    )

    resize_mask_node = ComposableNode(
        name='resize_mask_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
           'input_width': 640,
            'input_height': 640,
            'output_width': input_width,
            'output_height': input_height,
            'keep_aspect_ratio': False,
            'disable_padding': False
        }],
        remappings=[
            ('image', 'yolov8_segmentation'),
            ('camera_info', '/camera/camera/color/camera_info'),
            ('resize/image', 'segmentation'),
            ('resize/camera_info', 'camera_info_segmentation')
        ]
    )

    resize_left_viz = ComposableNode(
        name='resize_left_viz',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': input_width,
            'input_height': input_height,
            'output_width': int(input_width/VISUALIZATION_DOWNSCALING_FACTOR) ,
            'output_height': int(input_height/VISUALIZATION_DOWNSCALING_FACTOR),
            'keep_aspect_ratio': False,
            'encoding_desired': 'rgb8',
            'disable_padding': False
        }],
        remappings=[
            ('image', '/rgb/image_rect_color'),
            ('camera_info', '/rgb/camera_info'),
            ('resize/image', '/rgb/image_rect_color_viz'),
            ('resize/camera_info', '/rgb/camera_info_viz')
        ]
    )

    foundationpose_container = ComposableNodeContainer(
        name='foundationpose_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            #drop_node,
            convert_metric_node,
            tensor_rt_node,
            yolov8_decoder_node,
            detection2_d_array_filter_node,
            detection2_d_to_mask_node,
            crop_mask_node,
            #resize_mask_node,
            #resize_left_viz,
            foundationpose_node],
        output='screen',
    )

    return launch.LaunchDescription(launch_args + [foundationpose_container, yolov8_encoder,
                                                   realsense_camera_node])
