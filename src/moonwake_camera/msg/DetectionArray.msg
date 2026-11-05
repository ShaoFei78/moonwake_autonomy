#必须包含标准的Header,用于记录时间戳和坐标系
std_msgs/Header header
#状态码定义常量
uint8 STATUS_DETECTED=0
uint8 STATUS_NO_DETECTION=1
uint8 STATUS_INPUT_ERROR=2
uint8 STATUS_INPUT_STALE=3
uint8 STATUS_CONFIG_ERROR =4
#本帧处理状态
uint8 status 
#检测到的目标列表
Detection[] detections 
#本帧算法处理耗时(ms)
float32 process_time_ms
#数据年龄(ms)
float32 data_age_ms