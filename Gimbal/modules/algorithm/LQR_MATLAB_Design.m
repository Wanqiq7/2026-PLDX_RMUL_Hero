% % == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == ==
    % % 云台LQR控制器设计工具 -
        最终完整版 % % % % 功能： % % 1. 从Ozone CSV数据进行系统辨识 % %
            2. 自动设计LQR增益 % % 3. 生成嵌入式代码 % % % % 使用方法： % %
            1. 在Ozone中导出系统辨识数据（CSV格式） % % 2. 运行本脚本 % %
            3. 手动选择有效数据区间 % % 4. 复制生成的参数到gimbal.c % % % %
            作者：RoboMaster EC Team % % 日期：2025 -
        11 - 03 % %
    == == == == == == == == == == == == == == == == == == == == == == == == ==
    == == == == == == == == == == ==

    clear;
close all;
clc;

% % == == == == == == == == == == 电机参数定义 == == == == == == == == == ==
    % GM6020无刷电机参数（从手册确认） Kt = 0.741;
% 力矩常数[N·m / A] GearRatio = 1;
% 减速比（直驱） CAN_MAX = 16384;
% CAN控制值最大值 MaxCurrent_A = 20;
% 最大额定电流[A]

        % %
    == == == == == == == == == == 第1步：读取数据 == == == == == == == == == ==
    fprintf('========== 步骤1：读取CSV数据 ==========\n');

[ filename, pathname ] = uigetfile('*.csv', '选择Ozone导出的CSV文件');
if isequal (filename, 0)
  error('取消文件选择');
end

    data_table = readtable(fullfile(pathname, filename));
fprintf('读取文件: %s\n', filename);

% 智能提取列 try if ismember ('sysid_datastep_input',
                              data_table.Properties.VariableNames) time =
    data_table.Time;
input_CAN = data_table.sysid_datastep_input;
output_omega_raw = data_table.sysid_datamotor_output;
elseif ismember('Time',
                data_table.Properties.VariableNames) time = data_table.Time;
input_CAN = data_table{ :, 3};
output_omega_raw = data_table{ :, 4};
else error('无法识别列名，请检查CSV格式');
end catch ME error('数据提取失败: %s', ME.message);
end

    fprintf('✓ 数据点数: %d\n\n', length(time));

% % == == == == == == == == == == 第2步：单位转换 == == == == == == == == == ==
    fprintf('========== 步骤2：单位转换 ==========\n');

% CAN值 → 实际电流(A) input_current_A = input_CAN / CAN_MAX * MaxCurrent_A;

% 角速度方向修正（电机反转 = 陀螺仪正转） output_omega = -output_omega_raw;

fprintf('电流转换: CAN值 → 实际电流(A)\n');
fprintf('  转换系数: %.6f A/CAN\n', MaxCurrent_A / CAN_MAX);
fprintf('  范围: %.2f~%.2f A\n', min(input_current_A), max(input_current_A));
fprintf('\n角速度方向修正: 已取反\n\n');

% % == == == == == == == == == == 第3步：数据选择 == == == == == == == == == ==
    fprintf('========== 步骤3：选择有效数据 ==========\n');

figure('Position', [ 100, 100, 1400, 600 ]);
subplot(1, 2, 1);
plot(time, input_current_A, 'b', 'LineWidth', 1.5);
title('输入: 电流 [A]');
xlabel('时间 (s)');
ylabel('电流 (A)');
grid on;

subplot(1, 2, 2);
plot(time, output_omega, 'r', 'LineWidth', 1.5);
title('输出: 角速度 [rad/s]');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
grid on;

fprintf('请点击两次选择数据区间...\n');
[ x, ~] = ginput(2);
x = sort(x);

start_idx = find(time >= x(1), 1);
end_idx = find(time <= x(2), 1, 'last');

valid_time = time(start_idx : end_idx) - time(start_idx);
valid_current = input_current_A(start_idx : end_idx);
valid_omega = output_omega(start_idx : end_idx);

% 重采样处理非等距数据 dt_array = diff(valid_time);
if std (dt_array)
  / mean(dt_array) > 0.01 t_uni = (0 : mean(dt_array) : valid_time(
      end))'; valid_current = interp1(valid_time, valid_current, t_uni); valid_omega =
      interp1(valid_time, valid_omega, t_uni);
valid_time = t_uni;
end

    fprintf('✓ 选择: %d 点, %.2f s\n\n', length(valid_time), valid_time(end));

% % == == == == == == == == == == 第4步：系统辨识 == == == == == == == == == ==
    fprintf('========== 步骤4：系统辨识 ==========\n');

Ts = mean(diff(valid_time));
data_id = iddata(valid_omega, valid_current, Ts);

sys_I_to_omega = tfest(data_id, 1, 0);

fprintf('传递函数 ω(s)/I(s):\n');
tf(sys_I_to_omega)

    [num, den] = tfdata(sys_I_to_omega, 'v');
K_omega = num(end);
a = den(2);

% 物理参数 J = Kt / K_omega;
b = a * J;

fprintf('物理参数:\n');
fprintf('  J = %.6f kg·m²\n', J);
fprintf('  b = %.6f N·m·s/rad\n\n', b);

% 验证拟合度 y_sim = lsim(sys_I_to_omega, valid_current, valid_time);
fit = 100 *
      (1 - norm(valid_omega - y_sim) / norm(valid_omega - mean(valid_omega)));
fprintf('拟合度: %.1f%%\n\n', fit);

% % == == == == == == == == == == 第5步：LQR设计 == == == == == == == == == ==
    fprintf('========== 步骤5：LQR设计 ==========\n');

A = [ 0, 1; 0, -b / J ];
B = [0; Kt / J];

% 参数搜索 best_score = inf;
for R = [0.5, 1, 2]
    for q1 = [500, 1000, 2000, 5000]
        for q2 = [1, 2, 5, 10]
            Q = diag([q1, q2]);
[ K, ~, ~] = lqr(A, B, Q, R);

omega_n = sqrt(K(1) * B(2));
zeta = K(2) * B(2) / (2 * omega_n);

if zeta
  > 0.3 && zeta < 0.9 && K(1) < 50 score = (1.8 / omega_n / 0.05) ^ 2;
if score
  < best_score best_score = score;
K_best = K;
Q_best = Q;
R_best = R;
end end end end end

    fprintf('最优参数:\n');
fprintf('  Q = diag([%.0f, %.0f])\n', Q_best(1, 1), Q_best(2, 2));
fprintf('  R = %.1f\n', R_best);
fprintf('  K = [%.4f, %.4f]\n\n', K_best(1), K_best(2));

% % == == == == == == == == == == 第6步：性能仿真 == == == == == == == == == ==
    A_cl = A - B * K_best;
sys_cl = ss(A_cl, B, [ 1, 0 ], 0);

figure;
step(sys_cl);
title('LQR闭环阶跃响应');
info = stepinfo(sys_cl);

fprintf('预测性能:\n');
fprintf('  上升时间: %.1f ms\n', info.RiseTime * 1000);
fprintf('  超调量: %.1f%%\n\n', info.Overshoot);

% % == == == == == == == == == == 生成嵌入式代码 == == == == == == == == == ==
    fprintf('========== 嵌入式代码 ==========\n\n');

fprintf('// LQR参数 - 基于系统辨识\n');
fprintf('// J=%.6f, b=%.6f, 拟合度=%.1f%%%%\n', J, b, fit);
fprintf('#define LQR_K_ANGLE     %.6ff\n', K_best(1));
fprintf('#define LQR_K_VELOCITY  %.6ff\n', K_best(2));
fprintf('#define LQR_MAX_CURRENT %.2ff\n\n', MaxCurrent_A * 0.8);

fprintf('完成！请复制参数到代码中。\n');
