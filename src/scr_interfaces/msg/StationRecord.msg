# 一个停靠站的执行记录（行为树执行器在每站结束时生成）
int32 station
float64 u_c                  # 停靠站沿墙坐标（米）
int32 points
float64 move_dist_m          # 与上一站的沿墙距离
bool nav_leg                 # 本站是否先用 Nav2 导航
float64 station_s            # 本站总耗时（仿真秒）
float64 move_s               # 到站耗时（含 Nav2 段）
float64 cut_s                # 切割耗时
float64 sensed_lateral_mm    # 到站残差（传感器估计）
float64 sensed_standoff_mm
float64 sensed_yaw_deg
int32 clamped                # 补偿后被关节限位钳制的点数
float64 cut_cm               # 本站切割长度
bool moved                   # 横移是否按判据收敛
bool cut_ok
float64 origin_residual_mm   # 精对位停车时的沿墙残差（墙面原点）
int32 recoveries             # 本站触发的恢复次数（重试 / 重新对位）
