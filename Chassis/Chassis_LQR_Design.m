% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 底盘力控LQR增益计算工具 % % == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == ==
    % % 功能： % % 1. 基于底盘物理参数设计LQR控制器 % % 2. 计算速度→力 /
            扭矩的最优增益 % % 3. 生成嵌入式代码参数 % % % % 使用方法： % %
            1. 修改下方的底盘物理参数 % % 2. 运行本脚本 % %
            3. 复制生成的参数到chassis.c % % % %
            作者：RoboMaster EC Team(基于Gimbal LQR改编) % % 日期：2025
        - 01 - 04 % %
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == ==

    clear;
close all;
clc;

% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 第1步：底盘物理参数定义 % % == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == fprintf('========== 步骤1：底盘物理参数 ==========\n');

% 底盘基本参数（从robot_def.h获取） M_chassis = 17.0;
% 底盘质量[kg] r_wheel = 0.077;
% 轮子半径[m] wheel_base = 0.560;
% 纵向轴距[m] track_width = 0.330;
% 横向轮距[m]

    % 阻尼系数（需要实测估算，或从经验值开始） %
    阻尼来源：轮子摩擦、空气阻力、传动损耗等 b_linear = 15.0;
% 平移阻尼系数[N·s / m]，经验值 b_angular = 2.0;
% 旋转阻尼系数[N·m·s / rad]，经验值

    % 转动惯量估算（简化为矩形平板） % J ≈ M *(L² + W²) /
    12，更精确需CAD计算 L = wheel_base;
W = track_width;
J_chassis = M_chassis * (L ^ 2 + W ^ 2) / 12;
% [kg·m²]

    fprintf('底盘质量: M = %.1f kg\n', M_chassis);
fprintf('平移阻尼: b_linear = %.1f N·s/m\n', b_linear);
fprintf('旋转惯量: J = %.4f kg·m²\n', J_chassis);
fprintf('旋转阻尼: b_angular = %.2f N·m·s/rad\n\n', b_angular);

% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 第2步：状态空间模型建立 % % == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == fprintf('========== 步骤2：状态空间建模 ==========\n');

% % 2.1 平移方向（X / Y方向相同） % 动力学方程 : M *dv / dt =
    F - b *v % 状态方程 : dv / dt = -b / M *v + 1 / M *F % 状态 : x = v，控制
    : u = F

          A_linear = -b_linear / M_chassis;
B_linear = 1 / M_chassis;

fprintf('平移系统:\n');
fprintf('  状态方程: dv/dt = %.4f * v + %.4f * F\n', A_linear, B_linear);

sys_linear = ss(A_linear, B_linear, 1, 0);
fprintf('  传递函数: v(s)/F(s) = %.4f / (s + %.4f)\n', B_linear, -A_linear);
fprintf('  时间常数: τ = %.3f s\n\n', 1 / (-A_linear));

% % 2.2 旋转方向 % 动力学方程 : J *dω / dt =
    τ - b *ω % 状态方程 : dω / dt = -b / J *ω + 1 / J *τ % 状态 : x = ω，控制
    : u = τ

          A_angular = -b_angular / J_chassis;
B_angular = 1 / J_chassis;

fprintf('旋转系统:\n');
fprintf('  状态方程: dω/dt = %.4f * ω + %.4f * τ\n', A_angular, B_angular);

sys_angular = ss(A_angular, B_angular, 1, 0);
fprintf('  传递函数: ω(s)/τ(s) = %.4f / (s + %.4f)\n', B_angular, -A_angular);
fprintf('  时间常数: τ = %.3f s\n\n', 1 / (-A_angular));

% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 第3步：LQR设计（参数搜索） % % == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == ==
    fprintf('========== 步骤3：LQR参数优化 ==========\n');

% % 3.1 平移方向LQR设计 fprintf('平移方向LQR设计:\n');

best_score_linear = inf;
for Q_linear = [1000, 2000, 5000, 10000]
    for R_linear = [0.1, 0.5, 1.0, 2.0]
        % LQR求解
        K_linear = lqr(A_linear, B_linear, Q_linear, R_linear);

% 闭环系统 A_cl = A_linear - B_linear * K_linear;

% 检查稳定性 if A_cl <
    0 % 一阶系统，极点需为负 % 评估响应速度（时间常数小）和控制量适中 tau_cl =
    1 / (-A_cl);
score = tau_cl + 0.001 * K_linear ^ 2;
% 权衡速度和控制量

        if score <
    best_score_linear best_score_linear = score;
K_best_linear = K_linear;
Q_best_linear = Q_linear;
R_best_linear = R_linear;
end end end end

    fprintf('  最优参数: Q = %.0f, R = %.2f\n', Q_best_linear, R_best_linear);
fprintf('  LQR增益: K_velocity = %.2f N/(m/s)\n', K_best_linear);

% 闭环性能分析 A_cl_linear = A_linear - B_linear * K_best_linear;
sys_cl_linear = ss(A_cl_linear, B_linear, 1, 0);
info_linear = stepinfo(sys_cl_linear);
fprintf('  闭环时间常数: %.3f s\n', 1 / (-A_cl_linear));
fprintf('  建议时间: %.1f ms\n\n', info_linear.RiseTime * 1000);

% % 3.2 旋转方向LQR设计 fprintf('旋转方向LQR设计:\n');

best_score_angular = inf;
for Q_angular = [500, 1000, 2000, 5000]
    for R_angular = [0.1, 0.5, 1.0, 2.0]
        K_angular = lqr(A_angular, B_angular, Q_angular, R_angular);
A_cl = A_angular - B_angular * K_angular;

if A_cl
  < 0 tau_cl = 1 / (-A_cl);
score = tau_cl + 0.001 * K_angular ^ 2;

if score
  < best_score_angular best_score_angular = score;
K_best_angular = K_angular;
Q_best_angular = Q_angular;
R_best_angular = R_angular;
end end end end

    fprintf('  最优参数: Q = %.0f, R = %.2f\n', Q_best_angular, R_best_angular);
fprintf('  LQR增益: K_velocity = %.2f N·m/(rad/s)\n', K_best_angular);

A_cl_angular = A_angular - B_angular * K_best_angular;
sys_cl_angular = ss(A_cl_angular, B_angular, 1, 0);
info_angular = stepinfo(sys_cl_angular);
fprintf('  闭环时间常数: %.3f s\n', 1 / (-A_cl_angular));
fprintf('  建立时间: %.1f ms\n\n', info_angular.RiseTime * 1000);

% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 第4步：性能仿真 % % == == == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == fprintf('========== 步骤4：阶跃响应仿真 ==========\n');

figure('Position', [ 100, 100, 1200, 400 ]);

% 平移方向阶跃响应 subplot(1, 2, 1);
step(sys_cl_linear, 0 : 0.001 : 0.5);
title(sprintf('平移LQR阶跃响应\nK=%.1f N/(m/s)', K_best_linear));
xlabel('时间 (s)');
ylabel('速度 (m/s)');
grid on;

% 旋转方向阶跃响应 subplot(1, 2, 2);
step(sys_cl_angular, 0 : 0.001 : 0.5);
title(sprintf('旋转LQR阶跃响应\nK=%.1f N·m/(rad/s)', K_best_angular));
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
grid on;

fprintf('阶跃响应图已生成。\n\n');

% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 第5步：生成嵌入式代码 % % == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == fprintf('========== 嵌入式代码参数 ==========\n\n');

fprintf('/* 底盘LQR参数 - 基于物理模型设计 */\n');
fprintf('// 底盘质量: M=%.1f kg, 平移阻尼: b=%.1f N·s/m\n', M_chassis,
        b_linear);
fprintf('// 转动惯量: J=%.4f kg·m², 旋转阻尼: b=%.2f N·m·s/rad\n', J_chassis,
        b_angular);
fprintf('\n');

fprintf('// X/Y方向力控LQR参数\n');
fprintf('#define LQR_LINEAR_K_VELOCITY    %.2ff  // [N/(m/s)]\n',
        K_best_linear);
fprintf(
    '#define LQR_LINEAR_K_INTEGRAL    %.2ff  // [N/(m·s)]，建议为K_velocity的3-5%%\n',
    K_best_linear * 0.05);
fprintf('#define LQR_LINEAR_MAX_FORCE     %.2ff  // [N]\n\n', 150.0);

fprintf('// 旋转扭矩LQR参数\n');
fprintf('#define LQR_ANGULAR_K_VELOCITY   %.2ff  // [N·m/(rad/s)]\n',
        K_best_angular);
fprintf(
    '#define LQR_ANGULAR_K_INTEGRAL   %.2ff  // [N·m/rad]，建议为K_velocity的3-5%%\n',
    K_best_angular * 0.05);
fprintf('#define LQR_ANGULAR_MAX_TORQUE   %.2ff  // [N·m]\n\n', 40.0);

fprintf('/* 建议的完整配置代码 */\n');
fprintf('LQR_Velocity_Init_Config_s force_x_lqr_config = {\n');
fprintf('  .K_velocity = %.2ff,\n', K_best_linear);
fprintf('  .K_integral = %.2ff,\n', K_best_linear * 0.05);
fprintf('  .max_out = 150.0f,\n');
fprintf('  .enable_integral = 1,\n');
fprintf('  .integral_limit = 20.0f,\n');
fprintf('  .integral_deadband = 0.01f,\n');
fprintf('  .integral_decay_coef = 0.3f,\n');
fprintf('};\n\n');

% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == == % % 第6步：参数调整建议 % % == == == == ==
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == fprintf('========== 参数调整建议 ==========\n\n');

fprintf('1. 阻尼系数标定:\n');
fprintf('   - 推动底盘到某速度后释放，记录减速过程\n');
fprintf('   - 用 v(t) = v0 * exp(-b/M * t) 拟合数据得到b\n\n');

fprintf('2. LQR增益微调:\n');
fprintf('   - 响应太慢: 增大Q（相对于R）\n');
fprintf('   - 震荡: 减小Q或增大R\n');
fprintf('   - 稳态误差: 增大K_integral\n\n');

fprintf('3. 积分参数调整:\n');
fprintf('   - integral_limit: 建议为max_out的10-20%%\n');
fprintf('   - integral_deadband: 避免小误差时积分累积\n');
fprintf('   - integral_decay_coef: 0.2-0.4，防止大误差时积分饱和\n\n');

fprintf('完成！请复制参数到 Chassis/application/chassis/chassis.c\n');
