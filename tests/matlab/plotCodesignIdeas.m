% Author: Luca Carlone
% Date: 2016-12-07

clear all
close all
clc

% speed, distanceTraveled, pxDisplacement
data = [0 0 0
1 0.025 8.30962
2 0.05 16.6193
3 0.075 24.9289
4 0.1 33.2385
5 0.125 41.5481
6 0.15 49.8578
7 0.175 58.1674
8 0.2 66.477
9 0.225 74.7866
10 0.25 83.0962
11 0.275 91.4059
12 0.3 99.7155
13 0.325 108.025
14 0.35 116.335
15 0.375 124.644
16 0.4 132.954
17 0.425 141.264
18 0.45 149.573
19 0.475 157.883
20 0.5 166.192];

fh = figure; hold on
set(gca,'fontsize', 18);
plot(data(:,1),data(:,3),'-b','linewidth',2)
grid on
xlabel('speed [m/s]'); ylabel('pixel displacement [pixels]');
saveas(fh,'speedVSpixel','epsc');

% speed, distanceTraveled, pxDisplacement
data = [0 0 0
0.174533 0.00436332 1.45031
0.349066 0.00872665 2.90068
0.523599 0.01309 4.35116
0.698132 0.0174533 5.8018
0.872665 0.0218166 7.25267
1.0472 0.0261799 8.70381
1.22173 0.0305433 10.1553
1.39626 0.0349066 11.6071
1.5708 0.0392699 13.0594
1.74533 0.0436332 14.5122
1.91986 0.0479966 15.9656
2.0944 0.0523599 17.4196
2.26893 0.0567232 18.8742
2.44346 0.0610865 20.3295
2.61799 0.0654498 21.7857
2.79253 0.0698132 23.2426
2.96706 0.0741765 24.7005
3.14159 0.0785398 26.1593
3.31613 0.0829031 27.6191
3.49066 0.0872665 29.0799];

fh = figure; hold on
set(gca,'fontsize', 18);
plot(data(:,1),data(:,3),'-b','linewidth',2)
grid on
xlabel('rotation rate [rad/s]'); ylabel('pixel displacement [pixels]');
saveas(fh,'rotrateVSpixel','epsc');