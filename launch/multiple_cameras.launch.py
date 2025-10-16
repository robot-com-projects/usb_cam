#!/usr/bin/env python3

"""Simple multiple camera launch file based on working camera.launch.py pattern."""

import os
import sys
import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

# Add current directory to path to import local modules
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
usb_configs_path = os.path.join(dir_path, 'usb_configs')
sys.path.insert(0, usb_configs_path)  # Insert at beginning to prioritize this config

from camera_config import USB_CAM_DIR
# Import USB camera configs explicitly from the usb_configs directory
import importlib.util
spec = importlib.util.spec_from_file_location("usb_configs", os.path.join(usb_configs_path, "usb_configs.py"))
usb_configs_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usb_configs_module)
usb_cam_configs = usb_configs_module.configs

# Try to import optional dependencies
try:
    from python_utils.vision_utils import printlog
except ImportError:
    def printlog(msg, msg_type="INFO"):
        colors = {
            "OKGREEN": "\033[92m",
            "ERROR": "\033[91m", 
            "INFO": "\033[94m",
            "ENDC": "\033[0m"
        }
        color = colors.get(msg_type, colors["INFO"])
        print(f"{color}[{msg_type}] {msg}{colors['ENDC']}")


def generate_launch_description():
    """Generate launch description for multiple USB cameras - simplified version."""
    
    # Get robot type from environment
    robot_type = os.getenv('ROBOT_TYPE', 'bimanual_i2rt')
    
    printlog(f"Using robot configuration: {robot_type}", "INFO")
    
    # Get configuration
    if robot_type not in usb_cam_configs:
        printlog(f"Robot type '{robot_type}' not found in USB camera configs, using 'default'", "ERROR")
        robot_type = 'default'
    
    config = usb_cam_configs[robot_type]
    cameras_config = config.get('cameras', [])
    
    printlog(f"Found {len(cameras_config)} USB cameras in configuration", "INFO")
    printlog(f"Cameras configuration: {cameras_config}", "INFO")
    
    # Create camera nodes from configuration - simplified approach
    camera_nodes = []
    for camera_cfg in cameras_config:
        # Get camera parameters
        camera_name = camera_cfg['name']
        device = camera_cfg['device']
        param_file = camera_cfg.get('param_file') or camera_cfg.get('param_name')
        topic_name = camera_cfg.get('topic_name', camera_name)
        namespace = camera_cfg.get('namespace', None)
        
        # Build parameter file path
        param_path = Path(USB_CAM_DIR, 'config', param_file)
        
        if not param_path.exists():
            printlog(f"Parameter file not found for {camera_name}: {param_path}", "ERROR")
            continue
        
        printlog(f"Launching USB camera: {camera_name} (device: {device}, topic: {topic_name})", "OKGREEN")
        
        # Create remappings for topics
        remappings = [
            ('image_raw', f'{topic_name}/image_raw'),
            ('image_raw/compressed', f'{topic_name}/image_raw/compressed'),
            ('image_raw/compressedDepth', f'{topic_name}/image_raw/compressedDepth'),
            ('image_raw/theora', f'{topic_name}/image_raw/theora'),
            ('camera_info', f'{topic_name}/camera_info'),
        ]
        
        # Create node - same pattern as working camera.launch.py
        camera_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            name=camera_name,
            namespace=namespace,
            parameters=[str(param_path)],
            remappings=remappings
        )
        
        camera_nodes.append(camera_node)
    
    # Use GroupAction like the working launch file
    camera_group = GroupAction(camera_nodes)
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(camera_group)
    
    return ld
