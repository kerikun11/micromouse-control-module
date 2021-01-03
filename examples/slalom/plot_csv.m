%% cleaning
clear;
% close all;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 4);
figindex = 1;

%% load
rawdata_array = cell(5);
for i = 1 : 5
    try
        rawdata_array{i} = csvread(sprintf('../../build/slalom_%d.csv', i-1));
    catch
        prev = rawdata_array{i-1};
        rawdata_array{i} = prev(end, :);
    end
end

index = 1;
index_t = index; index = index + 1;
index_dddth = index; index = index + 1;
index_ddth = index; index = index + 1;
index_dth = index; index = index + 1;
index_th = index; index = index + 1;
index_dddx = index; index = index + 1;
index_ddx = index; index = index + 1;
index_dx = index; index = index + 1;
index_x = index; index = index + 1;
index_dddy = index; index = index + 1;
index_ddy = index; index = index + 1;
index_dy = index; index = index + 1;
index_y = index; index = index + 1;
index_v = index; index = index + 1;
index_w = index; index = index + 1;
index_dv = index; index = index + 1;
index_dw = index; index = index + 1;

%% angular ...
figure(figindex); clf(figindex); figindex = figindex + 1;
ylabels = {'[rad/s/s/s]', '[rad/s/s]', '[rad/s]', '[rad]'};
titles = {'Angular Jerk', 'Angular Acceleration', 'Angular Velocity', 'Angular Position'};
xlabelstr = '$t$ [s]';
data = [];
for i = 1:length(rawdata_array)
    data = [data; rawdata_array{i}];
end
t = data(:, index_t);
data = data(:, [index_dddth index_ddth index_dth index_th]);

for i = 1:4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    stairs(t, data(:, i), 'LineWidth', 2);
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end

%% x-y plot
figure(figindex); clf(figindex); figindex = figindex + 1;

for i = 1:length(rawdata_array)
    rawdata = rawdata_array{i};
    plot(rawdata(:, index_x), rawdata(:, index_y));
    hold on;
    grid on;
    axis equal;
    title('$(x, y)$ Trajectory');
    xlabel('$x$');
    ylabel('$y$');
end

xticks(-150:15:150);
yticks(xticks);
