function PlotSpace(DimSpace,OriginRef)

% PlotSpace	plots a 3D portion of the space
% to be filled with reference system (three unit vectors) 
% or other objects
%
% PlotSpace(DimSpace,OriginRef)
%  DimSpace is a 3 x 2 matrix with min an max dimensions of the space
%   [xmin xmax; ymin ymax; zmin zmax]
%   default value = [-1,5 +1.5] for all dimensions
%  OriginRef is the reference frame origin point
%   default value = [0 0 0]'

%	Copright (C) Basilio Bona 2007

n=nargin;
if n == 0
    DimSpace(1,1)=-1.5; DimSpace(1,2)=1.5; % xmin - xmax values 
    DimSpace(2,1)=-1.5; DimSpace(2,2)=1.5; % ymin - ymax values
    DimSpace(3,1)=-1.5; DimSpace(3,2)=1.5; % zmin - zmax values
    OriginRef=[0 0 0]';
end
if n == 1 
    OriginRef=[0 0 0]';
end    

xproj=DimSpace(1,2); yproj=DimSpace(2,2); zproj=DimSpace(3,1);

n_point=2; % number of points in a vector segment plot

% unit vectors of the reference frame
    ivet=[linspace(0,1,n_point)+OriginRef(1);linspace(0,0,n_point)+OriginRef(2);linspace(0,0,n_point)+OriginRef(3)] ;
    jvet=[linspace(0,0,n_point)+OriginRef(1);linspace(0,1,n_point)+OriginRef(2);linspace(0,0,n_point)+OriginRef(3)] ;
    kvet=[linspace(0,0,n_point)+OriginRef(1);linspace(0,0,n_point)+OriginRef(2);linspace(0,1,n_point)+OriginRef(3)] ;

    zerovet=[linspace(0,0,n_point);linspace(0,0,n_point);linspace(0,0,n_point)];
    projvet=[linspace(xproj,xproj,n_point);linspace(yproj,yproj,n_point);linspace(zproj,zproj,n_point)];

hold on

axis([DimSpace(1,1) DimSpace(1,2) DimSpace(2,1) DimSpace(2,2) DimSpace(3,1) DimSpace(3,2) ])

plot3(ivet(1,:), ivet(2,:), ivet(3,:))
%plot3(ivet(1,:), ivet(2,:), ivet(3,:),'-r','LineWidth',2)
% plot3(ivet(1,:), ivet(2,:), projvet(3,:),'-r','LineWidth',1.5)
% plot3(ivet(1,:), projvet(2,:), ivet(3,:),'-r','LineWidth',1.5)
% plot3(projvet(1,:), ivet(2,:), ivet(3,:),'*r','LineWidth',1.5)


plot3(jvet(1,:), jvet(2,:), jvet(3,:))
%plot3(jvet(1,:), jvet(2,:), jvet(3,:),'-g','LineWidth',2)
% plot3(jvet(1,:), jvet(2,:), projvet(3,:),'-g','LineWidth',1.5)
% plot3(jvet(1,:), projvet(2,:), jvet(3,:),'*g','LineWidth',1.5)
% plot3(projvet(1,:), jvet(2,:), jvet(3,:),'-g','LineWidth',1.5)


plot3(kvet(1,:), kvet(2,:), kvet(3,:))
%plot3(kvet(1,:), kvet(2,:), kvet(3,:),'-b','LineWidth',2)
% plot3(kvet(1,:), kvet(2,:), projvet(3,:),'*b','LineWidth',1.5)
% plot3(kvet(1,:), projvet(2,:), kvet(3,:),'-b','LineWidth',1.5)
% plot3(projvet(1,:), kvet(2,:), kvet(3,:),'-b','LineWidth',1.5)

axis square
grid on

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
view(-35,35)

hold off
