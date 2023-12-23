import sys
# import cv2
import pickle

# 读取标准输入的二进制数据流
data_bin = sys.stdin.buffer.read()


print("success\n")
# 从二进制数据中解码图片
# image_np = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)

# # 处理图片
# # ...
data_dict = pickle.loads(data_bin)
print(f"test2: {data_dict}")
# # 将结果二进制数据写入标准输出

success_message = 'test2 success!'
success_message_bin = pickle.dumps(success_message)

sys.stdout.buffer.write(success_message_bin)
sys.stdout.buffer.flush()