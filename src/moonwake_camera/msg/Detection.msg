#目标类别名称
string class_name
#目标中心点(2D像素坐标)
geometry_msgs/Point center
#原图像素框[x_min,y_min，x_max,y_max]
float32[4] bbox
#质量分数(0.0~1.0)
float32 score