"""
This file contains the configurations for multiple USB cameras.

The configs dict maps robot types to their camera configurations.
Each camera is defined by:
- name: Unique identifier for the camera
- device: Video device path (e.g., "/dev/video0") or device number (e.g., 0, 4)
- param_file or param_name: Parameter YAML file name in config/ directory
- topic_name: Base topic name for the camera (optional, defaults to camera name)
- namespace: ROS namespace (optional)
"""



configs = {
    "bimanual_i2rt": {
        "cameras": [
            # {
            #     "name": "main_camera",
            #     "device": "/dev/video12",
            #     "param_name": "params_main_camera_simple.yaml",
            #     "topic_name": "/main_camera/main_camera/color",
            # },
            {
                "name": "left_wrist_camera",
                "device": "/dev/video8",
                "param_name": "params_left_wrist_simple.yaml",
                "topic_name": "/left_wrist_camera/left_wrist_camera/color",
            },
            {
                "name": "right_wrist_camera", 
                "device": "/dev/video0",
                "param_name": "params_right_wrist_simple.yaml",
                "topic_name": "/right_wrist_camera/right_wrist_camera/color",
            },
        ]
    },
    "default": {
        "cameras": [
            {
                "name": "main_camera",
                "device": "/dev/video0",
                "width": 640,
                "height": 360,
                "param_name": "params_main_camera_simple.yaml",
                "topic_name": "/main_camera/main_camera/color",
            },
        ]
    }
}
