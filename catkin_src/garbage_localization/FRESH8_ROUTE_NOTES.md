# fresh8 动线记录与待改项

> 迁移包提示（2026-08-28）：这份文件是 Gazebo-only 历史路线记录，不是实验室地图或真机导航方案。实验室迁移时只用于理解曾经的任务层问题，不要直接复用其中的房间坐标、forbidden strip 或 fresh8 结论。

日期：2026-08-17
场景：`four_paper_ball` Gazebo-only 四房间增强仿真

## 用户现场观察

- 用户在 RViz 中明确看到机器人有几次走到出发入口外面，再掉头进入房间。
- 这条观察按真实现象记录；不能用后补的 AMCL CSV 覆盖或否定。

## fresh8 程序结果

- 任务流最终完成：`SEQUENTIAL_NAVIGATION_COMPLETE`。
- 请求目标数：4；完成目标数：4。
- 每个目标后均到达 `trash_bin`：4 次 `DROPOFF_REACHED`。
- 最终离场完成：`EXIT_REACHED`，出口为 `left_dining_room_exit`，排除了出发口 `bottom_living_room_entrance`。
- 任务层候选过滤已生效：日志中有 10 个候选被 `forbidden_path_region / outside_bottom_living_room_entrance` 拒绝。

## 动线摘要

fresh8 最终执行顺序：

1. `paper_ball` -> `trash_bin`
2. `box` -> `trash_bin`
3. `paper_ball` -> `trash_bin`
4. `box` -> `trash_bin`
5. `left_dining_room_exit` 离场

注意：这次视觉选择仍不是理想的“客厅瓶子 + 厨房瓶子 + 卧室纸团 + 客厅盒子”严格类别平衡，而是低置信度 `paper_ball/box` 多目标；这属于后续识别/目标选择要改的点。

## 这次记录的局限

- AMCL 轨迹记录器是在 fresh8 已运行一段时间后才启动的，首条样本已经在室内中部附近，不覆盖出发阶段。
- 因此 AMCL CSV 中 `outside_start_entrance_strip_samples=0` 只能说明“记录开始之后没有再采到入口外侧 strip”，不能说明整轮任务从未外绕。
- 用户在 RViz 中看到的入口外掉头，很可能发生在 AMCL 记录器启动前，或发生在局部规划控制阶段而不是候选终点本身。

## 需要改的地方

1. **记录必须提前启动**：下一轮测试要在发送任务目标前启动 AMCL 轨迹记录，覆盖从起点到最终出口的全程。
2. **入口外绕不能只看候选终点**：当前任务层过滤已能拒绝部分经过入口外 strip 的 A* 候选，但仍可能被局部规划/掉头动作带到入口外；需要进一步收紧起点入口附近的任务目标与路径判定。
3. **扩大入口外 forbidden strip 或加入口内缓冲线**：现有区域 `x=5.80..7.70, y=-1.20..0.15` 可能不够约束局部掉头；下一步建议把入口附近室内侧也作为高风险缓冲，不选会诱导外绕的 approach goal。
4. **保留 task-layer-only 原则**：不改 `nav_pkg`、`move_base`、costmap、planner；只在 `run_robocup_task_flow.py` 的候选选择和安全检查里处理。
5. **目标类别还要收紧**：后续要避免低置信度 `paper_ball/box` 假目标替代两个瓶子，最好做类别配额或按场景目标清单过滤。
6. **不做拾取、不碰物体**：继续保持 approach standoff，避免再把纸团推出房间。

## 下一轮验证标准

- 每次测试前重新启动 Gazebo 和 RViz，保持只有一个 RViz。
- 任务开始前先启动轨迹记录器。
- 完成后同时检查：任务 JSON、完整 AMCL CSV、入口外 forbidden strip 样本数、候选拒绝原因、最终出口状态。
- 如果 RViz 再观察到入口外掉头，即使 CSV 没采到，也按问题保留并继续收紧任务层路径过滤。
