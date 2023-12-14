GES low level development based on OmniGibson.

# 开发代码位置

- Fetching example & FetchAdvanced get_image(rgb, depth) [omnigibson/examples/custom/grasping_mode_example.py]
- Ik example（TODO） [omnigibson/examples/custom/ik_example.py]
- robot: FetchAdvanced [omnigibson/robots/fetch_advanced.py] (如果需要新加机器人，需要修改omnigibson/robots/__init__.py)
- 把tiago双臂机器人加到了家庭场景里面，见模拟器文档 Gibson环境搭建 部分和 GES-low-level/omnigibson/examples/custom/interactive_scene.py
- robot: TiagoAdvanced [omnigibson/robots/tiago_advanced.py]
- TiagoAdvanced fetching & camera tracking fetching goal [omnigibson/examples/custom/interactive_scene_fetch_ik.py]
- 把fetchAdvanced机器人加到了家庭场景里面，加了可交互的object，键盘控制移动，见模拟器文档 Gibson环境搭建 部分和 GES-low-level/omnigibson/examples/custom/grasp-interactive_scene.py
- 双臂机器人，目前想先用上层指令替换键盘指令，接进来。先实现移动，见没写完的 GES-low-level/omnigibson/examples/custom/merge_scene_and_move_EnvWrap.py （现在双臂键盘的移动有问题）

在FetchAdvanced类里面实现了get_image()来获取RGBD，在 grasping_mode_example.py 有使用样例

## API

### Get Image

1. `get_image()`
    - Description: Get the visual information of the robot from the current perspective
    - Parameter: None
    - Return: a Dict, with these key-value pairs:
        1. `image_rgb`: Get the RGB image seen by the robot from the current perspective.
        2. `image_depth`:  Get the depth image seen by the robot from the current perspective.


### Move Body

1. `move_x(distance: float)`
    - Description: The robot moves forward a distance d. If d is negative, the robot moves backward.
    - Parameter: 
        1. `distance`: The distance the robot needs to move forward or backward. `distance` in [-5, -0.1] means move back. `distance` in [0.1, 5] means move forward. The unit of distance is **meters**.
    - Return: 
        1. `success`: a bool indicating whether the operation is successful.

2. `move_y(distance: float)`
    - Description: The robot translates a distance d to the left. If d is negative, the robot translates to the right.
    - Parameter: 
        1. `distance`: The distance the robot needs to translate to the left or right. `distance` in [-1, -0.1] means move right. `distance` in [0.1, 1] means move left. The unit of distance is **meters**.
    - Return: 
        1. `success`: a bool indicating whether the operation is successful. 

3. `turn(angle: int)`
    - Description: The robot as a whole rotates counterclockwise by an angle. If the angle is negative, the robot rotates clockwise.
    - Parameter: 
        1. `angle`: The angle the robot needs to rotate counterclockwise(a.k.a. turn left). If it is negative, the robot rotates clockwise(a.k.a. turn right). `angle` is an integer in [-180, 180] expressed in **angle system**. `angle` in [-180, -1] means turn right. `angle` in [1, 180] means turn left.
    - Return: 
        1. `success`: a bool indicating whether the operation is successful.

