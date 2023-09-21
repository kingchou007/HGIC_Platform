import timeit
import numpy as np
from model import KeyPointClassifier, PointHistoryClassifier

static_label_path='./model/static/keypoint_classifier_label.csv'
dynamic_label_path='./model/dynamic/point_history_classifier_label.csv'

# 创建一个示例输入
sample_landmark_list = np.random.rand(21, 2).astype(np.float32)

# 创建KeyPointClassifier的实例
classifier = KeyPointClassifier()

# 使用timeit来测量执行时间
num_runs = 1000
timer = timeit.Timer(lambda: classifier(sample_landmark_list))
total_time = timer.timeit(number=num_runs)

average_time = total_time / num_runs
print(f"Average execution time: {average_time*1000:.2f} milliseconds")
