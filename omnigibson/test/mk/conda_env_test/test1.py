import subprocess
import os
import numpy as np
import pickle
import time

data = {
    'haha': np.array([0, 0, 0])
}
data_bin = pickle.dumps(data)

CONDA_ROOT_DIR = '/home/mad0g4/miniconda3/'
CONDA_ENV_NAME = 'py310'
CONDA_ENV_VERSION = 'python3.10'
env_variables = {
    "PATH": f"{CONDA_ROOT_DIR}envs/{CONDA_ENV_NAME}/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin",
    "PYTHONPATH": f"{CONDA_ROOT_DIR}envs/{CONDA_ENV_NAME}/lib/{CONDA_ENV_VERSION}/site-packages",
    "LD_LIBRARY_PATH": "/usr/local/lib",
    "LANG": "en_US.UTF-8",
    "LC_ALL": "en_US.UTF-8"
}
# 启动子进程
p = subprocess.Popen('python3 test2.py',  # 这里要写成Python3，不然如果env_variables环境不存在，就会导致自动使用python2而导致错误。
                    stdin=subprocess.PIPE, 
                    stdout=subprocess.PIPE, 
                    env= dict(os.environ, **env_variables),
                    shell=True)

time.sleep(2)

stdout_data_bin, stderr_data = p.communicate(input=data_bin)
# p.stdout.write(data_bin)
# p.stdout.flush()
# stdout_data_bin = p.stdin.read()

stdout_data = pickle.loads(stdout_data_bin)

# 处理结果数据
print(f"test1: {stdout_data}")