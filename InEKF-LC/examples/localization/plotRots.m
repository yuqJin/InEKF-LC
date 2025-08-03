N = length(ps);
dt = 0.01;
t = linspace(0, dt*N, 50);
lwdt=2;
% tap=100;
tap=round(100 / 0.83);

fig1 = figure;
% title('Position in $xy$-plan')
xlabel('$x$ /m')
ylabel('$y$ /m')
hold on;
grid on;
plot(ps(:, 1), ps(:, 2), 'k', 'LineWidth', 1.5);
scatter(ys(1, :), ys(2, :), 'r','linewidth',1.5);
% plot(ekf_ps(:, 1), ekf_ps(:, 2), 'm','linewidth',1.5);
% plot(iekf_ps(:, 1), iekf_ps(:, 2), 'c','linewidth',1.5);
% plot(ukf_ps(:, 1), ukf_ps(:, 2), 'b','linewidth',1.5);
% plot(iekfmc2_ps(:, 1), iekfmc2_ps(:, 2), 'm--','linewidth',1.5);
% plot(ukfgmc1_ps(:, 1), ukfgmc1_ps(:, 2), 'c--','linewidth',1.5);
% plot(ukfmc_ps(:, 1), ukfmc_ps(:, 2), 'r','linewidth',1.5);
% %plot(ukfimc_ps(:, 1), ukfimc_ps(:, 2), 'g','linewidth',1.5);
% plot(ukfgmc4_ps(:, 1), ukfgmc4_ps(:, 2), 'g','linewidth',1.5);
% %legend('true position', 'GPS measurements', 'EKF', 'InEKF', 'UKF-M','InEKF-MCC','InEKF-IMCC', 'UKF-M-MCC','UKF-M-IMCC');
plot(iekf_ps(:, 1), iekf_ps(:, 2), "Color",[0 0.4470 0.7410],'linewidth',1.5);
plot(ekf_ps(:, 1), ekf_ps(:, 2), "Color",[0.8500 0.3250 0.0980],'linewidth',1.5);
plot(ukf_ps(:, 1), ukf_ps(:, 2), "Color",[0.9290 0.6940 0.1250],'linewidth',1.5);
plot(iekfmc5_ps(:, 1), iekfmc5_ps(:, 2), "Color",[238 121 159]/255,'linewidth',1.5);
plot(iekfmc2_ps(:, 1), iekfmc2_ps(:, 2), "Color",[0.3010 0.7450 0.9330],'linewidth',1.5);
% plot(iekfmc3_ps(:, 1), iekfmc3_ps(:, 2), '--', "Color",[34 139 34]/255,'linewidth',1.5);
plot(ukfmc_ps(:, 1), ukfgmc1_ps(:, 2), "Color",[0.4660 0.6740 0.1880],'linewidth',1.5);
% plot(ukfmc_ps(:, 1), ukfmc_ps(:, 2), "Color",[0.3010 0.7450 0.9330],'linewidth',1.5);
% plot(ukfgmc4_ps(:, 1), ukfgmc4_ps(:, 2), "Color",[0.4660 0.6740 0.1880],'linewidth',1.5);
% legend('true position', 'GNSS measurements', 'EKF', 'InEKF', 'UKF-M', 'MCEKF-LG($\sigma=1$)',...
%     'MCEKF-LG($\sigma=2$)','GMCGF');
axis equal;
box on; % 添加边框

fig2 = figure;
%title('Robot orientation error (deg)')
% xlabel('时间 /s', 'Interpreter', 'none')
% ylabel('方向误差 /deg', 'Interpreter', 'none')
xlabel('时间 /s', 'FontName', 'SimSun', 'Interpreter', 'none')  % 中文部分用宋体
ylabel('方向误差 /deg', 'FontName', 'SimSun', 'Interpreter', 'none')  % 中文部分用宋体

hold on;
grid on;
plot(t, 180/pi*ekf_err_Rots(1:tap:end),"Color",[0 0.4470 0.7410], 'linewidth',lwdt);
plot(t, 180/pi*iekf_err_Rots(1:tap:end),"Color",[0.8500 0.3250 0.0980], 'linewidth',lwdt);
plot(t, 180/pi*ukf_err_Rots(1:tap:end), "Color",[0.9290 0.6940 0.1250], 'linewidth',lwdt);
% plot(t, 180/pi*iekfmc_err_Rots(1:tap:end), '--', "Color",[186 85 211]/255,'linewidth',lwdt);
% plot(t, 180/pi*iekfmc2_err_Rots(1:tap:end),'--', "Color",[0 229 238]/255, 'linewidth',lwdt);
% plot(t, 180/pi*iekfmc3_err_Rots, '--', "Color",[34 139 34]/255,'linewidth',lwdt);
plot(t, 180/pi*iekfmc5_err_Rots(1:tap:end), "Color",[238 121 159]/255,'linewidth',lwdt);
plot(t, 180/pi*iekfmc2_err_Rots(1:tap:end), "Color",[0.3010 0.7450 0.9330] ,'linewidth',lwdt);
plot(t, 180/pi*ukfmc_err_Rots(1:tap:end), "Color",[0.4660 0.6740 0.1880],'linewidth',lwdt);
% plot(t, 180/pi*ukfgmc10_err_Rots, "Color",[255 64 64]/255,'linewidth',lwdt);
% legend('EKF','InEKF','UKF-M','MCEKF-LG($\sigma=2$)', 'MCEKF-LG($\sigma=2.5$)','MCEKF-LG($\sigma=4$)', ...
%     'GMCCUKF-M($\alpha=1$)','GMCCUKF-M($\alpha=2$)','GMCCUKF-M($\alpha=4$)');
legend('EKF','InEKF','UKF-M','MCEKF-LG($\sigma=1$)','MCEKF-LG($\sigma=2$)',  ...
    'GMCGF');
box on; % 添加边框

fig3 = figure;
% xlabel('时间 /s)', 'Interpreter', 'none')
% ylabel('位置误差 /m)', 'Interpreter', 'none')
xlabel('时间 /s', 'FontName', 'SimSun', 'Interpreter', 'none')  % 中文部分用宋体
ylabel('位置误差 /m', 'FontName', 'SimSun', 'Interpreter', 'none')  % 中文部分用宋体

hold on;
grid on;
plot(t, iekf_err_ps(1:tap:end),"Color",[0 0.4470 0.7410], 'linewidth',lwdt);
plot(t, ekf_err_ps(1:tap:end),"Color", [0.8500 0.3250 0.0980], 'linewidth',lwdt);
plot(t, ukf_err_ps(1:tap:end), "Color",[0.9290 0.6940 0.1250],'linewidth',lwdt);
% plot(t, iekfmc_err_ps(1:tap:end), '--', "Color",[186 85 211]/255, 'linewidth',lwdt);
% plot(t, iekfmc2_err_ps(1:tap:end), '--', "Color",[0 229 238]/255, 'linewidth',lwdt);
% plot(t, iekfmc3_err_ps, '--', "Color",[34 139 34]/255, 'linewidth',lwdt);
plot(t, iekfmc5_err_ps(1:tap:end),"Color",[238 121 159]/255,'linewidth',lwdt);
plot(t, iekfmc2_err_ps(1:tap:end), "Color",[0.3010 0.7450 0.9330],'linewidth',lwdt);
plot(t, ukfmc_err_ps(1:tap:end), "Color",[0.4660 0.6740 0.1880],'linewidth',lwdt);
% plot(t, ukfgmc10_err_ps, "Color",[255 64 64]/255,'linewidth',lwdt);
% legend('EKF','InEKF','SE(2) UKF','MCEKF-LG($\sigma=2$)', 'MCEKF-LG($\sigma=2.5$)','MCEKF-LG($\sigma=4$)',...
%     'GMCCUKF-M($\alpha=1$)','GMCCUKF-M($\alpha=2$)','GMCCUKF-M($\alpha=4$)');
legend('EKF','InEKF','UKF-M','MCEKF-LG($\sigma=1$)','MCEKF-LG($\sigma=2$)',...
    'GMCGF');
box on; % 添加边框

fig4 = figure;
% title('Position in $xyz$-plane')
xlabel('$x$ /m')
ylabel('$y$ /m')
zlabel('$z$ /m')
hold on;
grid on;
% 设置z轴值为0
z = zeros(size(ps, 1), 1);
% 绘制3D轨迹
plot3(ps(:, 1), ps(:, 2), z, 'k', 'LineWidth', 1.5);
scatter3(ys(1, :), ys(2, :), zeros(1, size(ys, 2)), 'r', 'linewidth', 1.5);
plot3(iekf_ps(:, 1), iekf_ps(:, 2), z, "Color", [0 0.4470 0.7410], 'linewidth', 1.5);
plot3(ekf_ps(:, 1), ekf_ps(:, 2), z, "Color", [0.8500 0.3250 0.0980], 'linewidth', 1.5);
plot3(ukf_ps(:, 1), ukf_ps(:, 2), z, "Color", [0.9290 0.6940 0.1250], 'linewidth', 1.5);
plot3(iekfmc5_ps(:, 1), iekfmc5_ps(:, 2), z, "Color", [238 121 159]/255, 'linewidth', 1.5);
plot3(iekfmc2_ps(:, 1), iekfmc2_ps(:, 2), z, "Color", [0.3010 0.7450 0.9330], 'linewidth', 1.5);
% plot3(iekfmc3_ps(:, 1), iekfmc3_ps(:, 2), z, '--', "Color", [34 139 34]/255, 'linewidth', 1.5);
plot3(ukfmc_ps(:, 1), ukfgmc1_ps(:, 2), z, "Color", [0.4660 0.6740 0.1880], 'linewidth', 1.5);
% plot3(ukfmc_ps(:, 1), ukfmc_ps(:, 2), z, "Color", [0.3010 0.7450 0.9330], 'linewidth', 1.5);
% plot3(ukfgmc4_ps(:, 1), ukfgmc4_ps(:, 2), z, "Color", [0.4660 0.6740 0.1880], 'linewidth', 1.5);
legend('真实位置', 'GNSS测量值', 'EKF', 'InEKF', 'UKF-M', 'MCEKF-LG($\sigma=1$)',...
    'MCEKF-LG($\sigma=2$)','GMCGF', 'Interpreter', 'latex');
axis equal;
view(3); % 设为3D视角
