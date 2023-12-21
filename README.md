GES low level development based on OmniGibson.

# 运行

- 创建python 3.10的环境，用于运行high level（以及用于启动low level的程序）。（这个环境称作high level环境）。这个环境可能需要安装依赖`pip install pyyaml numpy psutil`。
- 创建一个python 3.7的环境，安装isaac sim 2022.2.0（具体见模拟器安装文档），以及`pip install -e .`当前仓库。模拟器实际上是在这个环境中运行的。（这个环境称作low level环境）
- 将conda的路径`CONDA_DIR_PATH`和**low level环境**的名字`CONDA_ENV_NAME`，希望运行的环境类型的config`ENV_CONFIG_FILE`写到根目录的`start_up_cfg.yaml`中
- **activate high level环境**（也就是python 3.10的环境），然后在仓库根目录下运行`python main.py`即可。他会创建新进程，并在新进程中activate low level环境、启动模拟器比如`omnigibson/custom_env/body_simulator.py`。
- 如果有显存泄露或端口占用的问题，运行`python main.py`之前多运行几次`killall python`。


# 开发代码位置

- `omnigibson/custom_env/body_simulator.py`: 模拟器环境的代码。如果要进行修改，可以复制一份，在复制的代码中进行修改，主要修改开头的技能集合`skillset`，按照样例增加需要的skill名和(开始函数，执行和结束函数)，在`omnigibson/custom_env/skills`这里写对应skill的实现。在`omnigibson/custom_env/body_simulator_cfg.yaml`中可以修改配置（包括机器人、物体、场景的config）。
- 一些历史开发代码在`omnigibson/examples/custom`中，可以参考。
  - Fetching example & FetchAdvanced get_image(rgb, depth) [omnigibson/examples/custom/grasping_mode_example.py]
  - Ik example（TODO） [omnigibson/examples/custom/ik_example.py]
  - robot: FetchAdvanced [omnigibson/robots/fetch_advanced.py] (如果需要新加机器人，需要修改omnigibson/robots/__init__.py)
  - 把tiago双臂机器人加到了家庭场景里面，见模拟器文档 Gibson环境搭建 部分和 GES-low-level/omnigibson/examples/custom/interactive_scene.py
  - robot: TiagoAdvanced [omnigibson/robots/tiago_advanced.py]
  - TiagoAdvanced fetching & camera tracking fetching goal [omnigibson/examples/custom/interactive_scene_fetch_ik.py]
  - 把fetchAdvanced机器人加到了家庭场景里面，加了可交互的object，键盘控制移动，见模拟器文档 Gibson环境搭建 部分和 GES-low-level/omnigibson/examples/custom/grasp-interactive_scene.py
  - 双臂机器人，目前想先用上层指令替换键盘指令，接进来。先实现移动，见没写完的 GES-low-level/omnigibson/examples/custom/merge_scene_and_move_EnvWrap.py （现在双臂键盘的移动有问题）


OmniGibson需要在一个python 3.7的环境下运行。
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
        1. `distance`: The distance the robot needs to move forward or backward. `distance` in [-1, -0.1] means move back. `distance` in [0.1, 1] means move forward. The unit of distance is **meters**.
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

