# Copyright 2023 usb_cam Authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the usb_cam Authors nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch multiple USB cameras based on robot type configuration."""
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Add current directory to path to import local modules
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
sys.path.append(os.path.join(dir_path, 'config'))

from camera_config import USB_CAM_DIR
from config.configs import configs

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


def setup_multi_camera_launch(context, *args, **kwargs):
    """Setup multiple USB cameras based on robot type configuration."""
    # Get robot type from launch configuration or environment
    robot_type = LaunchConfiguration('robot_type').perform(context)
    if not robot_type or robot_type == "''":
        robot_type = os.getenv('ROBOT_TYPE', 'default')
    
    printlog(f"Using robot configuration: {robot_type}", "INFO")
    
    # Get configuration
    if robot_type not in configs:
        printlog(f"Robot type '{robot_type}' not found in configs, using 'default'", "ERROR")
        robot_type = 'default'
    
    config = configs[robot_type]
    cameras_config = config.get('cameras', [])
    
    printlog(f"Found {len(cameras_config)} USB cameras in configuration", "INFO")
    
    # Create camera nodes from configuration
    camera_nodes = []
    for camera_cfg in cameras_config:
        # Get camera parameters
        camera_name = camera_cfg['name']
        device = camera_cfg['device']
        # Support both 'param_file' and 'param_name' for backward compatibility
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
        
        # Create node
        camera_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=camera_name,
            namespace=namespace,
            output='screen',
            parameters=[str(param_path)],
            remappings=remappings,
        )
        
        camera_nodes.append(camera_node)
    
    return camera_nodes


def generate_launch_description():
    """
    Generate launch description for multiple USB cameras.
    
    Launch arguments:
    - robot_type: Type of robot configuration to use (default: from ROBOT_TYPE env var or 'default')
    """
    
    # Declare launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='',
        description='Robot type configuration to use from configs.py (defaults to ROBOT_TYPE env var)'
    )
    
    # Use OpaqueFunction to evaluate LaunchConfiguration at runtime
    camera_launch = OpaqueFunction(function=setup_multi_camera_launch)
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(robot_type_arg)
    ld.add_action(camera_launch)
    
    return ld

