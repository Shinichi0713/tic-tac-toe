%% 流体解析アニメーション可視化 (MATLAB mコード)
% 流体シミュレーションの結果を動画としてアニメーション表示
% バーガース方程式・キャビティ流れ・ポテンシャル流れの時間発展を可視化

clear; clc; close all;

%% ============================================================
% アニメーション1: バーガース方程式の波の時間発展
%% ============================================================
% 非線形移流と粘性拡散の競合による衝撃波形成と平滑化

fprintf('=== アニメーション1: バーガース方程式 ===\n');

% パラメータ
L = 2*pi;
Nx = 256;
dx = L / Nx;
x = linspace(0, L, Nx+1); x = x(1:end-1);
nu = 0.02;
dt = 0.5 * dx^2 / nu;
Nt = 1000;
skip = 20;  % 描画スキップ（高速化のため）

% 初期条件
u = sin(x);

% 図の準備
figure('Name', 'バーガース方程式アニメーション', 'Position', [100 400 800 400]);
h_plot = plot(x, u, 'b-', 'LineWidth', 2);
hold on;
h_initial = plot(x, u, 'r--', 'LineWidth', 1, 'DisplayName', '初期波形');
xlabel('x [rad]'); ylabel('u(x,t)');
title(sprintf('バーガース方程式（ν = %.2f）', nu));
legend('Location', 'best');
grid on;
xlim([0 L]); ylim([-1.2 1.2]);

% アニメーションループ
for n = 1:Nt
    ip = [2:Nx, 1];
    im = [Nx, 1:Nx-1];

    adv = u .* (u(ip) - u(im)) / (2*dx);
    diff = nu * (u(ip) - 2*u + u(im)) / dx^2;
    u = u + dt * (-adv + diff);

    if mod(n, skip) == 0
        set(h_plot, 'YData', u);
        title(sprintf('バーガース方程式（ν = %.2f） t = %.2f', nu, n*dt));
        drawnow limitrate;
    end
end

fprintf('バーガース方程式のアニメーション完了\n');

%% ============================================================
% アニメーション2: バーガース方程式の時空間ヒートマップ
%% ============================================================
% 時間軸を縦軸にとった2Dヒートマップをスクロール表示

fprintf('\n=== アニメーション2: バーガース方程式ヒートマップ ===\n');

% 再計算（履歴を保存）
u = sin(x);
history = zeros(200, Nx);
history(1, :) = u;
history_idx = 1;

for n = 1:800
    ip = [2:Nx, 1];
    im = [Nx, 1:Nx-1];
    adv = u .* (u(ip) - u(im)) / (2*dx);
    diff = nu * (u(ip) - 2*u + u(im)) / dx^2;
    u = u + dt * (-adv + diff);

    if mod(n, 4) == 0 && history_idx < 200
        history_idx = history_idx + 1;
        history(history_idx, :) = u;
    end
end

figure('Name', 'バーガース方程式ヒートマップ', 'Position', [100 50 800 400]);
h_surf = imagesc(x, (1:history_idx)*dt*4, history(1:history_idx, :));
set(gca, 'YDir', 'normal');
xlabel('x [rad]'); ylabel('時間 t');
title('バーガース方程式：時空間発展');
colormap(jet); colorbar; caxis([-1 1]);

% 時間軸をスクロールするアニメーション
figure('Name', 'バーガース方程式スクロールヒートマップ', 'Position', [920 50 600 400]);
window = 50;  % 表示する時間ステップ数
for start_idx = 1:5:history_idx-window
    imagesc(x, (start_idx:start_idx+window-1)*dt*4, history(start_idx:start_idx+window-1, :));
    set(gca, 'YDir', 'normal');
    xlabel('x [rad]'); ylabel('時間 t');
    title(sprintf('バーガース方程式：時間窓 t = %.2f 〜 %.2f', start_idx*dt*4, (start_idx+window)*dt*4));
    colormap(jet); caxis([-1 1]);
    drawnow limitrate;
end

fprintf('ヒートマップアニメーション完了\n');

%% ============================================================
% アニメーション3: 移動するソースによるポテンシャル流れ
%% ============================================================
% ソースが時間的に移動する様子を速度ベクトル場で可視化

fprintf('\n=== アニメーション3: 移動ソースのポテンシャル流れ ===\n');

x_f = linspace(-5, 5, 40);
y_f = linspace(-5, 5, 40);
[X_f, Y_f] = meshgrid(x_f, y_f);

U_inf = 1.0;
Q_src = 3.0;

figure('Name', '移動ソースポテンシャル流れ', 'Position', [920 400 600 500]);

% ソースの移動軌道
src_path_x = linspace(-4, 4, 100);
src_path_y = 0.5 * sin(src_path_x);

for frame = 1:length(src_path_x)
    xs = src_path_x(frame);
    ys = src_path_y(frame);

    r_src = sqrt((X_f - xs).^2 + (Y_f - ys).^2);
    r_src(r_src < 0.2) = 0.2;

    u_src = (Q_src/(2*pi)) .* (X_f - xs) ./ r_src.^2;
    v_src = (Q_src/(2*pi)) .* (Y_f - ys) ./ r_src.^2;

    u_total = U_inf + u_src;
    v_total = v_src;

    clf;
    quiver(X_f, Y_f, u_total, v_total, 2, 'Color', [0.2 0.4 0.8]);
    hold on;
    plot(xs, ys, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    plot(src_path_x(1:frame), src_path_y(1:frame), 'r--', 'LineWidth', 1);
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('移動ソースのポテンシャル流れ（t = %d/100）', frame));
    axis equal; axis([-5 5 -5 5]); grid on;
    drawnow limitrate;
end

fprintf('移動ソースアニメーション完了\n');

%% ============================================================
% アニメーション4: キャビティ流れの定常化過程
%% ============================================================
% 初期状態から定常解に至るまでの流れの発達を可視化

fprintf('\n=== アニメーション4: キャビティ流れの定常化 ===\n');

N = 48;                 % 格子数（アニメーション用に少なめ）
L_cav = 1.0;
dx_c = L_cav / (N-1);
dy_c = L_cav / (N-1);
Re_cav = 100;
U_lid = 1.0;
nu_cav = U_lid * L_cav / Re_cav;
dt_c = 0.001;
Nt_c = 5000;
anim_skip = 100;

x_c = linspace(0, L_cav, N);
y_c = linspace(0, L_cav, N);

omega = zeros(N, N);
psi = zeros(N, N);
u = zeros(N, N);
v = zeros(N, N);

figure('Name', 'キャビティ流れ定常化', 'Position', [100 50 900 400]);

% 左側：流線関数
subplot(1,2,1);
h_psi = imagesc(x_c, y_c, psi);
set(gca, 'YDir', 'normal');
xlabel('x [m]'); ylabel('y [m]');
title('流線関数 ψ');
colormap(jet); colorbar; caxis([-0.1 0.1]);
axis equal; axis([0 L_cav 0 L_cav]);

% 右側：速度ベクトル
subplot(1,2,2);
h_quiver = quiver(x_c(1:3:end), y_c(1:3:end), u(1:3:end,1:3:end), v(1:3:end,1:3:end), 4);
hold on;
plot([0 L_cav L_cav 0 0], [0 0 L_cav L_cav 0], 'k-', 'LineWidth', 2);
xlabel('x [m]'); ylabel('y [m]');
title('速度ベクトル');
axis equal; axis([0 L_cav 0 L_cav]);

for n = 1:Nt_c
    % 境界条件
    omega(N, 2:N-1) = -2*psi(N-1, 2:N-1)/dy_c^2 - 2*U_lid/dy_c;
    omega(1, 2:N-1) = -2*psi(2, 2:N-1)/dy_c^2;
    omega(2:N-1, 1) = -2*psi(2:N-1, 2)/dx_c^2;
    omega(2:N-1, N) = -2*psi(2:N-1, N-1)/dx_c^2;

    % 渦度輸送
    for j = 2:N-1
        for i = 2:N-1
            dwdx = (omega(j, i+1) - omega(j, i-1)) / (2*dx_c);
            dwdy = (omega(j+1, i) - omega(j-1, i)) / (2*dy_c);
            conv = u(j,i)*dwdx + v(j,i)*dwdy;
            diff_w = (omega(j, i+1) - 2*omega(j, i) + omega(j, i-1))/dx_c^2 + ...
                     (omega(j+1, i) - 2*omega(j, i) + omega(j-1, i))/dy_c^2;
            omega(j,i) = omega(j,i) + dt_c * (-conv + nu_cav*diff_w);
        end
    end

    % ポアソン方程式（Gauss-Seidel）
    for iter = 1:50
        for j = 2:N-1
            for i = 2:N-1
                psi(j,i) = 0.25*(psi(j,i+1) + psi(j,i-1) + psi(j+1,i) + psi(j-1,i) + dx_c^2*omega(j,i));
            end
        end
    end

    % 速度更新
    u(2:N-1, 2:N-1) =  (psi(3:N, 2:N-1) - psi(1:N-2, 2:N-1)) / (2*dy_c);
    v(2:N-1, 2:N-1) = -(psi(2:N-1, 3:N) - psi(2:N-1, 1:N-2)) / (2*dx_c);
    u(N, :) = U_lid;

    % アニメーション更新
    if mod(n, anim_skip) == 0
        subplot(1,2,1);
        set(h_psi, 'CData', psi);
        title(sprintf('流線関数 ψ（t = %.2f）', n*dt_c));

        subplot(1,2,2);
        u_sub = u(1:3:end, 1:3:end);
        v_sub = v(1:3:end, 1:3:end);
        set(h_quiver, 'UData', u_sub, 'VData', v_sub);
        title(sprintf('速度ベクトル（t = %.2f）', n*dt_c));

        drawnow limitrate;
    end
end

fprintf('キャビティ流れアニメーション完了\n');

%% ============================================================
% アニメーション5: 管路流れの粒子追跡（ラグランジュ的視点）
%% ============================================================
% 流体粒子が管路内を流れる様子を追跡

fprintf('\n=== アニメーション5: 管路流れ粒子追跡 ===\n');

% 管路パラメータ
R_pipe = 0.05;
L_pipe = 2.0;
delta_p_p = 5000;
mu_p = 0.001;

% 粒子の初期位置
num_particles = 30;
theta_p = linspace(0, 2*pi, num_particles);
r_p_init = R_pipe * 0.7 * ones(1, num_particles);
x_p_init = zeros(1, num_particles);

% 粒子位置
x_p = x_p_init;
r_p = r_p_init;

figure('Name', '管路流れ粒子追跡', 'Position', [100 400 900 350]);

for step = 1:200
    % 各粒子位置での軸方向速度（ポアズイユ流れ）
    u_axial = (delta_p_p / (4*mu_p*L_pipe)) * (R_pipe^2 - r_p.^2);

    % 粒子移動
    x_p = x_p + u_axial * 0.001;

    % 周期境界（管路出口から入口に戻す）
    x_p(x_p > L_pipe) = x_p(x_p > L_pipe) - L_pipe;

    clf;
    % 管路の描画
    fill([0 L_pipe L_pipe 0], [R_pipe R_pipe -R_pipe -R_pipe], [0.9 0.95 1], 'EdgeColor', 'b');
    hold on;

    % 粒子の描画（半径位置で色分け）
    scatter(x_p, r_p, 50, r_p, 'filled');
    scatter(x_p, -r_p, 50, r_p, 'filled');
    colormap(jet); caxis([0 R_pipe]);

    xlabel('x [m]'); ylabel('r [m]');
    title(sprintf('管路流れ粒子追跡（ポアズイユ流れ）step = %d', step));
    axis equal; axis([0 L_pipe -R_pipe*1.5 R_pipe*1.5]); grid on;

    drawnow limitrate;
end

fprintf('粒子追跡アニメーション完了\n');

%% ============================================================
% アニメーション6: 渦の対消滅（Vortex Pair）
%% ============================================================
% 同強度・逆回転の2つの渦が接近して対消滅する様子

fprintf('\n=== アニメーション6: 渦の対消滅 ===\n');

x_v = linspace(-3, 3, 35);
y_v = linspace(-3, 3, 35);
[X_v, Y_v] = meshgrid(x_v, y_v);

Gamma_v = 5.0;

figure('Name', '渦の対消滅', 'Position', [920 400 500 500]);

for frame = 1:80
    % 渦の位置（時間的に接近）
    xv1 = -1.5 + frame * 0.03;
    xv2 = 1.5 - frame * 0.03;
    yv_pos = 0;

    r1 = sqrt((X_v - xv1).^2 + (Y_v - yv_pos).^2);
    r2 = sqrt((X_v - xv2).^2 + (Y_v - yv_pos).^2);
    r1(r1 < 0.15) = 0.15;
    r2(r2 < 0.15) = 0.15;

    % 渦1（正回転）
    u1 = -(Gamma_v/(2*pi)) .* (Y_v - yv_pos) ./ r1.^2;
    v1 =  (Gamma_v/(2*pi)) .* (X_v - xv1) ./ r1.^2;

    % 渦2（負回転）
    u2 =  (Gamma_v/(2*pi)) .* (Y_v - yv_pos) ./ r2.^2;
    v2 = -(Gamma_v/(2*pi)) .* (X_v - xv2) ./ r2.^2;

    u_vort = u1 + u2;
    v_vort = v1 + v2;

    clf;
    quiver(X_v, Y_v, u_vort, v_vort, 2.5, 'Color', [0.2 0.3 0.7]);
    hold on;
    plot(xv1, yv_pos, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(xv2, yv_pos, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

    % ストリームランの簡易表示（渦中心付近）
    theta_s = linspace(0, 2*pi, 50);
    for r_s = [0.3, 0.6, 1.0]
        plot(xv1 + r_s*cos(theta_s), yv_pos + r_s*sin(theta_s), 'r:', 'LineWidth', 0.8);
        plot(xv2 + r_s*cos(theta_s), yv_pos + r_s*sin(theta_s), 'b:', 'LineWidth', 0.8);
    end

    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('渦の対消滅（Γ = %.1f）frame = %d', Gamma_v, frame));
    axis equal; axis([-3 3 -3 3]); grid on;
    drawnow limitrate;
end

fprintf('渦の対消滅アニメーション完了\n');

%% ============================================================
% アニメーション7: レイノルズ数変化による流れの遷移
%% ============================================================
% 円柱周りの流れ（概念的）として、Reが増加するにつれて
% 流れが変化する様子を概念的に可視化

fprintf('\n=== アニメーション7: レイノルズ数による流れの遷移 ===\n');

figure('Name', 'レイノルズ数と流れの遷移', 'Position', [100 50 800 400]);

Re_range = [1, 10, 40, 100, 200, 1000];
for idx = 1:length(Re_range)
    Re_val = Re_range(idx);

    clf;
    subplot(1,2,1);
    % 円柱と流線（概念的）
    theta_c = linspace(0, 2*pi, 100);
    R_cyl = 0.3;
    fill(R_cyl*cos(theta_cyl), R_cyl*sin(theta_cyl), [0.5 0.5 0.5]);
    hold on;

    % 流線の概念的描画
    if Re_val <= 1
        % クリープ流：対称な流線
        y_stream = linspace(-1, 1, 8);
        for ys = y_stream
            if abs(ys) > R_cyl*1.2
                x_s = linspace(-2, 2, 50);
                plot(x_s, ys*ones(size(x_s)), 'b-', 'LineWidth', 1);
            end
        end
        title_text = sprintf('Re = %d: クリープ流（対称）', Re_val);

    elseif Re_val <= 40
        % 層流：後方に渦対が形成
        y_stream = linspace(-1, 1, 8);
        for ys = y_stream
            if abs(ys) > R_cyl*1.2
                x_s = linspace(-2, 2, 50);
                y_offset = ys * (1 + 0.1*exp(-(x_s-0.5).^2));
                plot(x_s, y_offset, 'b-', 'LineWidth', 1);
            end
        end
        % 後方渦対
        plot([0.4 0.7], [0.15 0.3], 'r-', 'LineWidth', 1.5);
        plot([0.4 0.7], [-0.15 -0.3], 'r-', 'LineWidth', 1.5);
        title_text = sprintf('Re = %d: 層流（後方渦対）', Re_val);

    else
        % カルマン渦列
        y_stream = linspace(-1, 1, 6);
        for ys = y_stream
            x_s = linspace(-2, 2, 50);
            y_offset = ys + 0.1*sin(3*x_s) .* exp(-0.3*(x_s-0.5).^2);
            plot(x_s, y_offset, 'b-', 'LineWidth', 0.8);
        end
        % 渦の概念表示
        for k = 1:4
            plot(0.6 + k*0.3, 0.4*(-1)^k, 'ro', 'MarkerSize', 8);
        end
        title_text = sprintf('Re = %d: カルマン渦列', Re_val);
    end

    xlabel('x [m]'); ylabel('y [m]');
    title(title_text);
    axis equal; axis([-2 2 -1.2 1.2]); grid on;

    subplot(1,2,2);
    % レイノルズ数ゲージ
    bar(Re_val, 'FaceColor', [0.2 0.5 0.8]);
    xlabel('流れの状態');
    ylabel('レイノルズ数');
    title(sprintf('レイノルズ数 Re = %d', Re_val));
    set(gca, 'XTick', 1, 'XTickLabel', {judge_flow_regime(Re_val)});
    ylim([0 1200]);

    drawnow;
    pause(1.5);  % 各状態を1.5秒表示
end

fprintf('レイノルズ数遷移アニメーション完了\n');

%% ============================================================
% 補助関数
%% ============================================================

function regime = judge_flow_regime(Re)
    if Re < 1
        regime = 'クリープ流';
    elseif Re < 40
        regime = '層流（対称）';
    elseif Re < 200
        regime = '層流（渦対）';
    elseif Re < 400
        regime = 'カルマン渦列';
    else
        regime = '乱流';
    end
end
