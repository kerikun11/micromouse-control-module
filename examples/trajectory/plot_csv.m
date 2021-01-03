%% cleaning
clear;
close all;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
% set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 2);
figindex = 1;

%% load
rawdata = csvread('../../build/out.csv');
index = 1;
t = rawdata(:, index); index = index + 1;
dddth = rawdata(:, index); index = index + 1;
ddth = rawdata(:, index); index = index + 1;
dth = rawdata(:, index); index = index + 1;
th = rawdata(:, index); index = index + 1;
dddx = rawdata(:, index); index = index + 1;
ddx = rawdata(:, index); index = index + 1;
dx = rawdata(:, index); index = index + 1;
x = rawdata(:, index); index = index + 1;
dddy = rawdata(:, index); index = index + 1;
ddy = rawdata(:, index); index = index + 1;
dy = rawdata(:, index); index = index + 1;
y = rawdata(:, index); index = index + 1;
v = rawdata(:, index); index = index + 1;
w = rawdata(:, index); index = index + 1;
dv = rawdata(:, index); index = index + 1;
dw = rawdata(:, index); index = index + 1;

%% plot x, y, theta
ylabels = {'$j$ [mm/s/s/s]', '$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'};
titles = {'Jerk', 'Acceleration', 'Velocity', 'Position'};
xlabelstr = '$t$ [s]';

figure(figindex); figindex = figindex + 1;
data = [dddx ddx dx x];

for i = 1:4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    stairs(t, data(:, i));
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end

figure(figindex); figindex = figindex + 1;
data = [dddy ddy dy y];

for i = 1:4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    stairs(t, data(:, i));
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end

figure(figindex); figindex = figindex + 1;
data = [dddth ddth dth th];

for i = 1:4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    stairs(t, data(:, i));
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end

%% x-y plot
figure(figindex); figindex = figindex + 1;
plot(x, y);
grid on;
axis equal;
title('$(x, y)$ Trajectory');
xlabel('$x$');
ylabel('$y$');

%% Velocity
figure(figindex); figindex = figindex + 1;
subplotNum = 4;

titles = {'Translational Acceleration', 'Translational Velocity', 'Rotational Acceleration', 'Rotational Velocity'};
ylabels = {'$\dot{v}$ [mm/s/s]', '$v$ [mm/s]', '$\dot{\omega}$ [rad/s/s]', '$\omega$ [rad/s]'};
legends = {'Reference'};
data = {dv, v, dw, w};

for i = 1:subplotNum
    subplot(subplotNum, 1, i); hold off;
    stairs(t, data{i}); grid on;
    title(titles{i});
    xlabel('Time $t$ [s]');
    ylabel(ylabels{i});
    legend(legends);
end

%%
cos_th = cos(th);
sin_th = sin(th);
u1 = ddx;
u2 = ddy;
du1 = dddx;
du2 = dddy;
dxi = u1 .* cos_th + u2 .* sin_th;
