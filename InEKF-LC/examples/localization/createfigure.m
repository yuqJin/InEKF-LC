function createfigure(X1, YMatrix1)
%CREATEFIGURE(X1, YMatrix1)
%  X1:  plot x 数据的向量
%  YMATRIX1:  plot y 数据的矩阵

%  由 MATLAB 于 13-Mar-2024 15:50:12 自动生成

% 创建 figure
figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot1 = plot(X1,YMatrix1,'LineWidth',2,'Parent',axes1);
set(plot1(1),'DisplayName','EKF','Color',[0 0.447 0.741]);
set(plot1(2),'DisplayName','InEKF','Color',[0.85 0.325 0.098]);
set(plot1(3),'DisplayName','SE(2) UKF','Color',[0.929 0.694 0.125]);
set(plot1(4),'DisplayName','MCEKF-LG($\sigma=2.5$)',...
    'Color',[0.729411764705882 0.333333333333333 0.827450980392157]);
set(plot1(5),'DisplayName','GMCCUKF-M($\alpha=1$)',...
    'Color',[0.933333333333333 0.474509803921569 0.623529411764706]);
set(plot1(6),'DisplayName','GMCCUKF-M($\alpha=2$)',...
    'Color',[0.301 0.745 0.933]);
set(plot1(7),'DisplayName','GMCCUKF-M($\alpha=4$)',...
    'Color',[0.466 0.674 0.188]);

% 创建 ylabel
ylabel('position error (m)','FontName','Arial');

% 创建 xlabel
xlabel('$t$ (s)','FontName','Arial');

% 取消以下行的注释以保留坐标区的 Y 范围
% ylim(axes1,[0 0.8]);
grid(axes1,'on');
hold(axes1,'off');
% 设置其余坐标区属性
set(axes1,'FontName','Arial','FontSize',16.2);
