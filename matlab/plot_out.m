%% cleaning
clear;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
% set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 4);

%% load
rawdata = csvread('../build/out.csv');
t = rawdata(:, 1);
a = rawdata(:, 2) / 10;
v = rawdata(:, 3);
x = rawdata(:, 4);
legends= {'Acceleration $a$ [mm/s/s]', 'Velocity $v$ [mm/s]', 'Position $x$ [mm]'};
ylabels= {'$a$ [mm/s/s]', '$v$ [mm/s]', '$x$ [mm]'};
titles= {'Acceleration', 'Velocity', 'Position'};
xlabelstr = '$t$ [s]';

figure(1);
hold off;
plot(t, [a v x]);
grid on;
title('Accel Designer');
xlabel(xlabelstr);
legend(legends, 'Location', 'SouthWest');

figure(2);
data = [a v x];
for i = 1 : 3
    subplot(3, 1, i);
    hold off; plot(nan, nan); % clean
    hold on;
    ax = gca; ax.ColorOrderIndex = i;
    plot(t, data(:, i));
    grid on;
    xlabel(xlabelstr);
    ylabel(ylabels(i));
    title(titles(i));
end
