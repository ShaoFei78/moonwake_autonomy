#必须包含标准的Header,用于记录时间戳和坐标系
std_msgs/Header header
#状态码定义常量
uint8 STATUS_DETECTED=0
uint8 STATUS_NO_DETECTION=1
uint8 STATUS_INPUT_ERROR=2
uint8 STATUS_INPUT_STALE=3
uint8 STATUS_CONFIG_ERROR =4
#模型加载失败 / 推理(forward)抛异常：不得伪装成 NO_DETECTION
uint8 STATUS_MODEL_ERROR=5
#本帧处理状态
uint8 status 
#检测到的目标列表（**只放本帧真实观测到的**）
Detection[] detections 

#本帧"只有预测、没有观测"的目标（轨迹处于 LOST 状态）。
#  - 它们的位置来自卡尔曼预测，本帧没有任何检测支撑；
#  - track_state 一定是 TRACK_LOST，score 等于 quality（没有检测置信度可言）；
#  - 用途：短时遮挡时告诉下游"它可能还在那儿"，以及"它什么时候彻底失效"
#    （看 valid_until）。
#  - 下游如果只想要真实观测，可以完全忽略这个数组。
Detection[] predicted_tracks
#本帧算法处理耗时(ms)
float32 process_time_ms
#数据年龄(ms)
float32 data_age_ms