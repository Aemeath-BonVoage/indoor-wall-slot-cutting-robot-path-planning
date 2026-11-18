# figstyle：数据图的统一样式

仓库中所有数据图的作图脚本都导入本模块（`docs/figstyle/__init__.py`），按 IEEE 论文版式输出英文图。图的中文说明写在各报告的图注与 [图集](../FIGURES.md) 中。

## 约定

| 项目 | 取值 |
|---|---|
| 宽度 | 单栏 3.5 in（`width='single'`），双栏 7.16 in（`width='double'`）；高度按图确定 |
| 字体 | Liberation Serif（与 Times New Roman 等宽度）；正文 8 pt，刻度与图例 7 pt，最小 7 pt；数学符号用 mathtext，同一字体 |
| 输出 | 同目录、同名的矢量 PDF（TrueType / Type 42 字体嵌入）与 300 dpi PNG；Markdown 文档引用 PNG |
| 网格 | 默认无网格；需要时用 `light_grid()` 画细浅网格 |
| 面板标签 | `panel_label(ax, 'a', '简短说明')` 写在左上角，形如 “(a) …”；图内不放大标题，说明写在图注 |
| 坐标轴 | 单位写在括号中：`label('Error', 'mm')` → “Error (mm)” |

## 配色（Okabe–Ito 色盲友好色板）

颜色带固定语义，所有图一致；线型、标记形状与柱的填充纹理与颜色一一对应，黑白打印时仍可区分。

| 角色 | 颜色 | 线型 / 标记 / 柱 | 含义 |
|---|---|---|---|
| `baseline` | 朱红 `#D55E00` | 虚线 / 方块 / 白底斜线柱 | v1、第一版、基线、规则路线、仅轮式里程计、NavFn + DWB |
| `proposed` | 蓝 `#0072B2` | 实线 / 圆点 / 实心柱 | v2、当前版本、联合规划、视觉（AprilTag + 激光 EKF）、Smac 2D + MPPI |
| `alt` | 绿 `#009E73` | 点划线 / 三角 / 点纹柱 | 第一个变体或备选（如 EKF 融合、2.5D 混合、间歇遮挡） |
| `alt2` | 橙 `#E69F00` | 点线 / 菱形 / 交叉纹柱 | 第二个变体或备选（如示教零位、IBVS） |
| `alt3`、`alt4` | 紫 `#CC79A7`、天蓝 `#56B4E9` | 其他线型 | 更多变体 |
| `neutral` | 灰 `#7F7F7F` | 点线 / 叉 / 竖线纹柱 | 中性参照（逐槽划分、标定数据、被排除的运行） |
| `reference` | 黑 | 点线 | 目标值、容差、真值、下界 |

## 用法

```python
import os
import sys
sys.path.insert(0, os.path.join(REPO, 'docs'))
import figstyle as fs

fig, ax = fs.subplots(width='single', height=2.4)
ax.plot(t, e1, label='v1 (colour blob + PBVS)', **fs.line('baseline'))
ax.plot(t, e2, label='v2 (AprilTag + LiDAR EKF)', **fs.line('proposed'))
ax.set_xlabel(fs.label('Time', 's'))
ax.set_ylabel(fs.label('Error', 'mm'))
ax.legend()
fs.save(fig, 'docs/experiments/<主题>/fig_name.png')   # 同时写出 fig_name.pdf
```

其他辅助函数：`bar(role)`、`marker(role)` 返回对应角色的作图参数；`plain_log(ax)` / `plain_ticks(ax)` 让对数轴刻度显示普通数字（避免上标小于 7 pt）；`fixed_layout(fig, ...)` 以英寸固定页边距与面板间距，使一行面板等宽、间距一致。

各图的生成脚本与重新生成命令见 [图集](../FIGURES.md) 末尾。
