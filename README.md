GES low level development based on OmniGibson.

# 运行

- 创建python 3.10的环境，用于运行high level（以及用于启动low level的程序）。（这个环境称作high level环境）。这个环境可能需要安装依赖`pip install pyyaml numpy psutil`。
- 创建一个python 3.7的环境，安装isaac sim 2022.2.0（具体见模拟器安装文档），以及`pip install -e .`当前仓库。模拟器实际上是在这个环境中运行的。（这个环境称作low level环境）
- 将conda的路径`CONDA_DIR_PATH`和**low level环境**的名字`CONDA_ENV_NAME`，希望运行的环境类型的config`ENV_CONFIG_FILE`写到根目录的`start_up_cfg.yaml`中
- **activate high level环境**（也就是python 3.10的环境），然后在仓库根目录下运行`python main.py`即可。他会创建新进程，并在新进程中activate low level环境、启动模拟器比如`omnigibson/custom_env/body_simulator.py`。
- 如果有显存泄露或端口占用的问题，运行`python main.py`之前多运行几次`killall python`。
- 如果要测试机器人的skill执行，可以在`main.py`中注释掉Start Test Skills的若干行， 然后运行`python main.py`，此时在命令行输入skill名字，就可以执行对应的skill了，skill样例参见`main.py`里面写的注释。

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
        2. `image_depth`:  Get the depth image seen by the robot from the current perspective. The values are not normalized.


### Move Body

1. `move_forward(mode: str)`
    - Description: Control the robot to move forward. The robot will
        continue to move forward until next command is received or
        the robot is blocked by an obstacle.
    - Parameter:
        1. `mode`: a string indicating the moving mode. It can be
            'run', 'walk', or 'crawl'.
    - Return:
        1. `success`: a bool indicating whether the robot has started moving.

2. `move_backward(mode: str)`
   - Description: Control the robot to move backward. The robot will
       continue to move backward until next command is received or
       the robot is blocked by an obstacle.
   - Parameter:
       1. `mode`: a string indicating the moving mode. It can be
           'walk' or 'crawl'. 'run' is NOT supported for safety.
   - Return:
       1. `success`: a bool indicating whether the robot has started moving.

3. `move_left(mode: str)`
    - Description: Control the robot to move left. The robot will
        continue to move left until next command is received or
        the robot is blocked by an obstacle.
    - Parameter:
        1. `mode`: a string indicating the moving mode. It can be
            'walk' or 'crawl'. 'run' is NOT supported for safety.
    - Return:
        1. `success`: a bool indicating whether the robot has started moving.


4. `move_right(mode: str)`
    - Description: Control the robot to move right. The robot will
        continue to move right until next command is received or
        the robot is blocked by an obstacle.
    - Parameter:
        1. `mode`: a string indicating the moving mode. It can be
            'walk' or 'crawl'. 'run' is NOT supported for safety.
    - Return:
        1. `success`: a bool indicating whether the robot has started moving.


5. `turn_left(angle: int)`
    - Description: Control the robot to turn left by an angle.
    - Parameter:
        1. `angle`: a POSITIVE INTEGER in [1, 180]
            indicating the degrees that robot needs to turn left.
    - Return:
        1. `success`: a bool indicating whether the robot has completed turning.

6. `turn_right(angle: int)`
    - Description: Control the robot to turn right by an angle.
    - Parameter:
        1. `angle`: a POSITIVE INTEGER in [1, 180]
            indicating the degrees that robot needs to turn right.
    - Return:
        1. `success`: a bool indicating whether the robot has completed turning.

7. `brake()`
    - Description: Control the robot to brake. The robot will
        stop immediately.
    - Parameter: None
    - Return:
        1. `success`: a bool indicating whether the operation is successful.

