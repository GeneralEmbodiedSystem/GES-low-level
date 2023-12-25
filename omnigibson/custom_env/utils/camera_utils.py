import numpy as np
import cv2

from omnigibson.sensors import VisionSensor


# Transfer rgba image to rgb image
def rgba2rgb(rgba, background=(255,255,255)):
    row, col, ch = rgba.shape
    if ch != 4:
        raise ValueError('RGBA image should have 4 channels.')
    rgb = np.zeros((row, col, 3), dtype='float32')
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]
    a = np.asarray(a, dtype='float32') / 255.0
    R, G, B = background
    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray(rgb, dtype='uint8')

def execute_record(vision_sensor: VisionSensor, buffer: dict, saved_vedio_path = None, fps = 60, end = False):
    '''
    record a frame when invoke with end=False
    empty cache & save vedio when end=True && saved_vedio_path is not None
    '''
    if 'record_frames' not in buffer:
        buffer['record_frames'] = list()
    record_frames = buffer['record_frames']
    
    sensor_obs = vision_sensor.get_obs()
    rgba_image = sensor_obs['rgb']
    rgb_image = rgba2rgb(rgba_image)
    
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    
    # cv2.imshow('Image', bgr_image)
    # # 等待按键后关闭窗口
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    record_frames.append(bgr_image)
    
    if end and len(record_frames) > 0:
        height, width, layers = record_frames[0].shape
        size = (width, height)
        out = cv2.VideoWriter(saved_vedio_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        for frame in record_frames:
            out.write(frame)
        out.release()
        
        buffer.pop('record_frames')