-- safety_limits_startup.lua
-- 一次性设置：软限位 / 力上限 / 加速度上限（速度上限由每次 mc.move 的 speed 参数控制）
-- 注意：这些设置 **掉电丢失**，建议将本脚本设为“上电自启动”。

local MIN_LIMIT_MM  = 10.0    -- 软限位（负向，距离最小张口端的安全边界），示例 10 mm
local PLUS_LIMIT_MM = 90.0    -- 软限位（正向，距离最大张口端的安全边界），示例 90 mm
local FORCE_LIMIT_N = 40.0    -- 力上限（保守值，避免过载），示例 40 N
local ACC_LIMIT     = 1000.0  -- 加速度上限（mm/s^2），官方示例值 1000 mm/s²

-- 可选：将速度上限作为“推荐运行速度”的参考值，真正的速度仍由 mc.move(pos, speed) 指定
local RECOMMENDED_SPEED = 50.0  -- mm/s，供外部控制节点参考

-- 工具函数：打印当前设定
local function print_current()
  local neg, pos = mc.softlimits()
  local acc = mc.acceleration()
  local _, fmax = mc.force()
  printf("[Safety] SoftLimits: minus=%.2f mm, plus=%.2f mm", neg, pos)
  printf("[Safety] AccLimit: %.1f mm/s^2", acc)
  printf("[Safety] ForceLimit: %.1f N", fmax)
end

-- 1) 设置软限位（自动启用软限位检查）
printf("[Safety] Setting soft limits to %.1f / %.1f mm ...", MIN_LIMIT_MM, PLUS_LIMIT_MM)
local neg, pos = mc.softlimits(MIN_LIMIT_MM, PLUS_LIMIT_MM)  -- 同时启用检查
-- 如需显式开关：mc.softlimits_en(true)  -- 参见文档 2.6.12

-- 2) 设置加速度上限（STOP/FAST STOP 不受其限；掉电丢失）
printf("[Safety] Setting acceleration limit to %.1f mm/s^2 ...", ACC_LIMIT)
mc.acceleration(ACC_LIMIT)

-- 3) 设置力上限
printf("[Safety] Setting force limit to %.1f N ...", FORCE_LIMIT_N)
mc.force(FORCE_LIMIT_N)

-- 4) 回读确认
print_current()

-- 5) 运行提示
printf("[Safety] Recommended run speed (for mc.move): %.1f mm/s", RECOMMENDED_SPEED)
printf("[Safety] NOTE: All limits are lost on power-down; keep this script in autorun.")
