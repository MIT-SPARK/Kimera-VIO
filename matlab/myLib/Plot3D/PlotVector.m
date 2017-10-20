% PlotVector plot a 3D vector in a 3D space 
% built with PlotSpace command
% 
% PlotVector(vet,origin)
%  vet is the 3 x 1 vector to be drawn
%  origin is the 3 x 1 application point

%	Copright (C) Basilio Bona 2007

function PlotVector(vet,origin)
n=nargin;
if n == 0
    origin = [0 0 0]';
    vet=[1 1 1]';
elseif n == 1
    origin = [0 0 0]';
end
    n_point=2; % number of points in a vector segment plot
%  vector components
    vvet=[linspace(0,vet(1),n_point)+origin(1);linspace(0,vet(2),n_point)+origin(2);linspace(0,vet(3),n_point)+origin(3)] ;
    %hold;
    plot3(vvet(1,:), vvet(2,:), vvet(3,:),'m','LineWidth',5)
