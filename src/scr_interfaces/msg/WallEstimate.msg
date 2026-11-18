# 墙面相对状态估计（技能服务器的 EKF，每个估计周期发布一次）
std_msgs/Header header
float64 d                    # 离墙距离（米）
float64 u                    # 沿墙位置，相对墙面原点（参考定位标记），面向墙面向右为正（米）
float64 psi                  # 朝向偏角（rad）
float64 sigma_d
float64 sigma_u
float64 sigma_psi
bool scale_estimated         # 是否估计横移比例修正
float64 lateral_scale        # 真实横移 = (1 + lateral_scale)·里程计横移
float64 sigma_scale
float64 odom_since_fix       # 自上次接受标记更新以来的沿墙里程计行程（米）
bool u_known                 # 沿墙位置是否已由标记确定
bool wall_ok                 # 本周期激光墙面更新是否被接受
string tag_state             # 标记更新状态说明
int32[] tag_ids              # 本周期画面中的地图内标记
geometry_msgs/Point[] tag_positions   # 对应标记中心在 base_link 中的位置（已做延迟补偿）
bool[] tag_accepted          # 对应更新是否通过门限
