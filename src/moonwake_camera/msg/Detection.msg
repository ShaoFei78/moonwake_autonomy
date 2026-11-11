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
