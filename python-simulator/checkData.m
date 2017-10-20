clear all
close all
clc

M = dlmread('./simulator1/groundtruth_states_noHead.csv');
figure; hold on
plot3(M(:,2),M(:,3),M(:,4),'-b')

figure; hold on
for t = 1:20:size(M,1)
    c = rand(1,3);
    plot3(M(t,2),M(t,3),M(t,4),'.','color',c)
    drawnow
end

119998000000
  3960000000