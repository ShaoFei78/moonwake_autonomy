#目标类别名称
string class_name
#目标中心点(2D像素坐标)
geometry_msgs/Point center
#原图像素框[x_min,y_min，x_max,y_max]
float32[4] bbox
#质量分数(0.0~1.0)。语义由检测后端定义，跨后端不可直接比较：
#  HSV  后端 = 连通域填充率（面积/外接框面积），衡量"形状方不方正"
#  YOLO 后端 = 模型置信度，衡量"有多像浮标"
#下游按 score 过滤时，必须按后端分别设阈值。
float32 score

#------------------------ 跟踪 ------------------------
#轨迹编号：同一个目标在不同帧保持同一个 id。
#  -1 表示这一帧没有跟踪信息（只有 track_enable:=false 时才会出现）
#  刚出现、还没确认的目标也会分到一个新 id，只是 track_state=TRACK_TENTATIVE
int32 track_id -1

#轨迹状态常量
uint8 TRACK_TENTATIVE=0
uint8 TRACK_CONFIRMED=1
uint8 TRACK_LOST=2
#本帧该目标的轨迹状态
uint8 track_state 0

#------------------------ 时效与质量 ------------------------
#这条轨迹已经存在多久（秒）。刚出现是 0；跟得越久越可信。
float32 age_s 0.0

#观测的有效期（ROS 时间的秒数）。
#  下游使用前必须检查：如果当前的 ROS 时间 > valid_until，这条观测已过期，应丢弃。
#  = 最后一次见到它的时刻 + 允许丢失的时长。
#  0.0 表示没有时效信息（例如 track_enable:=false）。
float64 valid_until 0.0

#质量分 0~1。它综合了"命中次数"和"丢失次数"：
#  刚出现（未确认）时较低；连续命中会升高；一旦开始丢帧就会下降。
#  注意：它不是检测置信度，而是"这条轨迹有多可信"。
float32 quality 0.0