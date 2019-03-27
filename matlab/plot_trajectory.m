%% cleaning
clear;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
% set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 2);
figindex = 1;

%% load
rawdata = csvread('../build/out.csv');
index = 1;
t     = rawdata(:, index); index = index+1;
dddth = rawdata(:, index); index = index+1;
ddth  = rawdata(:, index); index = index+1;
dth   = rawdata(:, index); index = index+1;
th    = rawdata(:, index); index = index+1;
dddx  = rawdata(:, index); index = index+1;
ddx   = rawdata(:, index); index = index+1;
dx    = rawdata(:, index); index = index+1;
x     = rawdata(:, index); index = index+1;
dddy  = rawdata(:, index); index = index+1;
ddy   = rawdata(:, index); index = index+1;
dy    = rawdata(:, index); index = index+1;
y     = rawdata(:, index); index = index+1;

%% plot
ylabels= {'$j$ [mm/s/s/s]', '$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'};
titles= {'Jerk', 'Acceleration', 'Velocity', 'Position'};
xlabelstr = '$t$ [s]';

figure(figindex); figindex = figindex + 1;
data = [dddth ddth dth th];
for i = 1 : 4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    plot(t, data(:, i));
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end

figure(figindex); figindex = figindex + 1;
data = [dddx ddx dx x];
for i = 1 : 4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    plot(t, data(:, i));
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end

figure(figindex); figindex = figindex + 1;
data = [dddy ddy dy y];
for i = 1 : 4
    subplot(4, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    plot(t, data(:, i));
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
xlabel('$x$');
ylabel('$y$');

%% info
fprintf("n: %d\n", size(t, 1));
fprintf("x: %f\ty: %f\n", x(end), y(end));
