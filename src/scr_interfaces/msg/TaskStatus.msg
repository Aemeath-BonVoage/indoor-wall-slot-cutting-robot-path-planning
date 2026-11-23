# 技能服务器的当前阶段（实验记录器据此标注真值轨迹）
builtin_interfaces/Time stamp   # 阶段生效时刻；按时间表提前发布时在未来（机械臂连续轨迹的各段）
int32 station        # 当前停靠站序号；-1 表示不在站上
string phase         # idle / align / move / arrived / hold / cut / arm / scan
string detail        # 'scheduled'：按时间表提前发布，接收方应按 stamp 而不是到达时刻使用
