# Yolov8 ROS2 Node

This is a ROS2 node example for integrating Yolov8 in Rust. **Please note that this project is a work in progress.**

## Prerequisites

- ROS2 jazzy or later
- Python 3.8 or later
- ROS2_rust
- Candle

## Setup

1. Clone this repository:
    ```sh
    git clone git@github.com:BrandonSimoncic/yolov8_ros2_node.git
    cd yolov8_node
    ```

2. Download the `yolov8n.safetensors` file from the official source and place it in the `launch` directory.

3. Build the package:
    ```sh
    colcon build --packages-up-to yolov8_ros
    ```

4. Source the setup file:
    ```sh
    source install/setup.bash
    ```

## Usage

To launch the node, run:
```sh
ros2 launch yolov8_ros yolov8_ros
```

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License.

## Current TODO

1. Clean Warnings
1. Add Config file
1. Write Launch File
1. Improve performance on Yolo Candle implementation
1. Clean up Comments
