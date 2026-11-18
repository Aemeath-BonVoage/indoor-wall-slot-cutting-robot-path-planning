# 机械臂笛卡尔目标点（base_link 系）
float64 x            # 前伸（米）
float64 y            # 横向（米，向左为正）
float64 z            # 高度（米）
float64 rotation     # 刀具绕前伸轴转角（弧度）
bool at_depth        # 是否处于切深（用于残差补偿与“切割 / 移动”阶段标注）
