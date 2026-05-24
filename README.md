# teb_local_planner

TEB (Timed Elastic Band) 局部路径规划算法，非ROS版本移植，可作为第三方库集成到项目中。

原始代码和论文参考: [rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)

## 编译

```bash
cd build && cmake .. && make -j$(nproc)
```

### 依赖

| 库 | 用途 |
|---|---|
| g2o | 图优化框架 |
| Eigen3 | 线性代数 |
| Boost (system, thread, graph) | 系统/线程/图工具 |
| Qt5 Widgets | GUI 可视化 |
| nlohmann/json | JSON 配置文件读写 (header-only, `apt install nlohmann-json3-dev`) |
| SuiteSparse | g2o 稀疏求解器依赖 |

## 运行

```bash
./build/teb
```

启动后显示 Qt5 窗口，包含:
- **轨迹显示区** (500x500) — 实时绘制规划路径
- **Start Pose** — 起点 x/y (QDoubleSpinBox) 和 theta (QSlider, -π ~ π)
- **Goal Pose** — 终点 x/y (QDoubleSpinBox) 和 theta (QSlider, -π ~ π)
- **Edit Config** 按钮 — 打开 JSON 编辑器，直接修改所有 ~80 个 TEB 参数，保存后即时生效

首次运行自动生成 `teb_config.json`（包含所有默认参数），后续启动自动加载。

![example](example.png)

## 库调用流程

```cpp
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"

using namespace teb_local_planner;

// 1. 加载配置
TebConfig config;                          // 默认参数，或 config.loadFromFile("teb_config.json");

// 2. 设置障碍物
std::vector<ObstaclePtr> obst_vector;
obst_vector.emplace_back(boost::make_shared<PointObstacle>(0, 0));

// 3. 设置机器人模型
RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.4);

// 4. 构造规划器
TebVisualizationPtr visual(new TebVisualization(config));
ViaPointContainer via_points;
TebOptimalPlanner planner(config, &obst_vector, robot_model, visual, &via_points);

// 5. 规划路径
PoseSE2 start(-2, 0, 0);
PoseSE2 goal(2, 0, 0);
planner.plan(start, goal);

// 6. 获取轨迹
std::vector<Eigen::Vector3f> path;
planner.getFullTrajectory(path);
// path[i] = [x, y, theta]
```
