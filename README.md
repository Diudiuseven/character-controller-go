# character-controller-go

一个确定性的 2D 固定步长角色运动控制器。角色为 AABB（轴对齐包围盒），
世界由静态盒体（`Box`）、单向斜坡面（`Slope`）与动态承载平台
（`Platform`）组成。相同初始状态 + 相同输入序列 ⇒ 逐帧完全一致的轨迹。

## 快速开始

```go
cfg := controller.DefaultConfig()          // 可修改后需通过 Validate()
world := &controller.World{
    Boxes:  []controller.Box{{Min: controller.V(-50, -1), Max: controller.V(50, 0)}},
    Slopes: []controller.Slope{{A: controller.V(5, 0), B: controller.V(10, 2)}},
    Platforms: []*controller.Platform{
        {ID: 1, Center: controller.V(0, 3), Half: controller.V(2, 0.5), Vel: controller.V(1, 0)},
    },
}
sim, err := controller.NewSim(cfg, world, controller.NewCharacter(controller.V(0, 0.5)))
if err != nil { /* 非法配置，错误定位到具体参数 */ }

for {
    sim.Step(controller.Input{Move: readMove(), Jump: readJumpEdge()})
}

snap, _ := sim.Snapshot()   // 任意检查点可序列化
_ = sim.Restore(snap)       // 续跑轨迹与连续模拟逐位一致
```

## 时间模型

- 模拟只通过 `Sim.Step(input)` 推进，每步恰好 `Config.Dt` 秒。
  渲染帧率、调度间隔与模拟无关：调用方按累积时间决定调用多少次
  `Step`，轨迹只取决于输入序列。
- 每步内部相位顺序固定：平台运动 → 平台承载 → 计时器（土狼时间/跳跃
  缓冲）→ 水平速度（加速/摩擦/阻力/滑坡动力学）→ 重力与终端速度 →
  跳跃 → 移动与碰撞（等分子步进）→ 斜坡交互 → 支撑状态更新。
- 单步位移按 `min(Width, Height)/2` 自动切分为确定数量的等长子步，
  任何速度下都不可能穿透薄碰撞体（防隧穿），子步数只是状态的纯函数，
  不影响确定性。
- 全部运算为固定顺序的 `float64` 运算，无随机数、无 map 迭代、无
  时钟读取；同一平台上回放与实时模拟逐位一致。

## 接触语义

- **静态盒体（Box）**：四面皆实体。水平移动被墙阻挡时水平速度归零，
  竖直运动不受影响（沿墙平滑滑动）；从上方落到盒顶即接地；头部碰撞
  天花板时竖直速度归零。相邻盒体接缝处不会卡住（站立时脚底与盒顶
  恰好接触，严格不等式判定不产生水平阻挡）。
- **台阶（Step-up）**：接地状态下被低矮障碍（高差 ≤ `MaxStepHeight`
  且头顶有空间）阻挡时自动抬升越过；更高的障碍视为墙。
- **斜坡（Slope）**：单向表面，只从上方接触（从下方跳穿不阻挡）。
  - 角度 ≤ `MaxSlopeAngleDeg`：可行走，上坡自动贴合抬升，下坡自动
    吸附（`GroundSnapDistance`），不会弹跳。
  - 角度 > `MaxSlopeAngleDeg + SlopeHysteresisDeg`：陡坡。行走接近时
    表现为墙（水平阻挡）；从空中落上陡坡则进入 `StateSliding`，
    速度被约束为沿坡面向下并受重力沿坡分量加速（上限为终端速度）。
  - 滞回带：已进入滑坡状态后，角度需降到 `MaxSlopeAngleDeg -
    SlopeHysteresisDeg` 以下才恢复站立，临界角度附近不会来回抖动。
- **平台（Platform）**：运动学承载物，每步先按 `Vel`/`AngularVel`
  运动。站在平台上的角色先被施加平台的刚体位移（绕平台中心旋转 +
  平移），再进行自身运动与碰撞求解。平台碰撞体始终是 AABB（旋转只
  影响承载，不倾斜表面）；角色被携带后若脱离表面会被吸附回台面
  （在容差内）。平台变速、转向或消失时不会给角色注入任何额外速度，
  角色只保留自身速度。
- **接地容错**：离开地面后 `CoyoteTime` 内仍允许跳跃；落地前
  `JumpBufferTime` 内按下的跳跃会在落地瞬间触发。
- 状态机：`StateGrounded` / `StateAirborne` / `StateSliding`，
  每步结束由支撑探测唯一确定。

## 参数（Config）

| 参数 | 含义 | 约束 |
|---|---|---|
| `Dt` | 固定步长（秒） | (0, 0.1] |
| `Gravity` | 重力加速度（向下，正值） | > 0 |
| `MaxRunSpeed` | 输入驱动的水平速度上限 | > 0，且 `MaxRunSpeed*Dt ≤ Width` |
| `GroundAccel` / `AirAccel` | 地面/空中水平加速度 | ≥ 0 |
| `GroundFriction` | 地面无输入时的水平减速度 | ≥ 0 |
| `AirDrag` | 空中水平阻力（每秒比例） | [0, 1) |
| `JumpSpeed` | 起跳初速度 | (0, TerminalVelocity] |
| `TerminalVelocity` | 下落终端速度 | > 0 |
| `MaxSlopeAngleDeg` | 可行走坡度上限（度） | (0, 90) |
| `SlopeHysteresisDeg` | 坡度滞回带宽（度） | [0, MaxSlopeAngleDeg)，且两者之和 < 90 |
| `MaxStepHeight` | 可迈上的台阶最大高度 | [0, Height) |
| `GroundSnapDistance` | 下坡吸附距离 | ≥ 0 |
| `CoyoteTime` | 离地后仍可跳跃的容错窗口 | ≥ 0 |
| `JumpBufferTime` | 落地前跳跃输入缓冲窗口 | ≥ 0 |
| `Width` / `Height` | 角色碰撞盒尺寸 | > 0 |

`Config.Validate()` 拒绝非法或相互冲突的配置，返回 `*ConfigError`，
其 `Field` 字段与错误信息均定位到具体参数。`NewSim` 在构造时强制校验。

加减速使用 `approach`（永不超调），阻力为比例衰减，下落速度被终端
速度钳制：参数取极值时不会发散，也不会出现反向运动。输入 `Move`
被钳制到 [-1, 1]，NaN 输入按 0 处理。

## 确定性与序列化

- `Sim.Snapshot()` 序列化全部动态状态（角色位置/速度/状态机/计时器、
  各平台中心与角度、步数计数），`Sim.Restore()` 从任意检查点继续，
  续跑轨迹与连续模拟逐位一致（见 `TestCheckpointResumeBitExact`）。
- 恢复时世界几何（盒体、斜坡、平台形状与 ID）必须与快照一致，
  缺失的平台会返回具名错误。

## 测试

```
go test ./...
```

覆盖：确定性与回放、重力/摩擦/阻力/终端速度、极值参数不发散、
土狼时间与跳跃缓冲、墙体滑动/接缝/台阶/天花板/角落、斜坡上下行走、
陡坡阻挡与下滑、临界角度滞回、平台平移/旋转/变速/消失承载、
检查点恢复逐位一致、非法配置逐参数定位。
