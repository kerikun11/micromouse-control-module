%% cleaning
clear;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
% set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 2);

%% load
rawdata = csvread('../build/out.csv');
t = rawdata(:, 1);
j = rawdata(:, 2);
a = rawdata(:, 3);
v = rawdata(:, 4);
x = rawdata(:, 5);

%% plot
legends= {'Jerk $j$ [mm/s/s/s]', 'Acceleration $a$ [mm/s/s]', 'Velocity $v$ [mm/s]', 'Position $x$ [mm]'};
ylabels= {'$j$ [mm/s/s/s]', '$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'};
titles= {'Jerk', 'Acceleration', 'Velocity', 'Position'};
xlabelstr = '$t$ [s]';

figure(1);
data = [j a v x];
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

%{
figure(2);
hold off;
plot(t, [a v x]);
grid on;
title('Accel Designer');
xlabel(xlabelstr);
legend(legends, 'Location', 'SouthWest');
%}
