% PlotRef plot a 3D representation of a reference frame 
% given by three unit vectors
% 
% PlotRef(Ref,DimSpace,OriginRef)
%  Ref is a 3 x 3 orthogonal matrix representing the unit vectors
%   of the reference frame to be drawn
%  DimSpace is a 3 x 2 matrix with min an max dimensions of the space
%   [xmin xmax; ymin ymax; zmin zmax]
%   default value = [-1,5 +1.5] for all dimensions
%  OriginRef is the reference frame origin point
%   default value = [0 0 0]'

%	Copright (C) Basilio Bona 2007

function PlotRef(Ref,DimSpace,OriginRef)

n=nargin;
if n == 1
    DimSpace(1,1)=-1.5; DimSpace(1,2)=1.5;
    DimSpace(2,1)=-1.5; DimSpace(2,2)=1.5;
    DimSpace(3,1)=-1.5; DimSpace(3,2)=1.5;
    OriginRef=[0 0 0]';
end
if n == 2 
    OriginRef=[0 0 0]';
end    

xproj=DimSpace(1,2); yproj=DimSpace(2,2); zproj=DimSpace(3,1);

n_point=2; % number of points in a vector segment plot

% unit vectors of the reference frame
    xvet=[linspace(0,Ref(1,1),n_point)+OriginRef(1);linspace(0,Ref(2,1),n_point)+OriginRef(2);linspace(0,Ref(3,1),n_point)+OriginRef(3)] ;
    yvet=[linspace(0,Ref(1,2),n_point)+OriginRef(1);linspace(0,Ref(2,2),n_point)+OriginRef(2);linspace(0,Ref(3,2),n_point)+OriginRef(3)] ;
    zvet=[linspace(0,Ref(1,3),n_point)+OriginRef(1);linspace(0,Ref(2,3),n_point)+OriginRef(2);linspace(0,Ref(3,3),n_point)+OriginRef(3)] ;

    zerovet=[linspace(0,0,n_point);linspace(0,0,n_point);linspace(0,0,n_point)];
    projvet=[linspace(xproj,xproj,n_point);linspace(yproj,yproj,n_point);linspace(zproj,zproj,n_point)];

hold on

axis([DimSpace(1,1) DimSpace(1,2) DimSpace(2,1) DimSpace(2,2) DimSpace(3,1) DimSpace(3,2) ])

plot3(xvet(1,:), xvet(2,:), xvet(3,:),'-r','LineWidth',2)
plot3(xvet(1,:), xvet(2,:), projvet(3,:),':r','LineWidth',1.5)
plot3(xvet(1,:), projvet(2,:), xvet(3,:),':r','LineWidth',1.5)
plot3(projvet(1,:), xvet(2,:), xvet(3,:),':r','LineWidth',1.5)


plot3(yvet(1,:), yvet(2,:), yvet(3,:),'-g','LineWidth',2)
plot3(yvet(1,:), yvet(2,:), projvet(3,:),':g','LineWidth',1.5)
plot3(yvet(1,:), projvet(2,:), yvet(3,:),':g','LineWidth',1.5)
plot3(projvet(1,:), yvet(2,:), yvet(3,:),':g','LineWidth',1.5)


plot3(zvet(1,:), zvet(2,:), zvet(3,:),'-b','LineWidth',2)
plot3(zvet(1,:), zvet(2,:), projvet(3,:),':b','LineWidth',1.5)
plot3(zvet(1,:), projvet(2,:), zvet(3,:),':b','LineWidth',1.5)
plot3(projvet(1,:), zvet(2,:), zvet(3,:),':b','LineWidth',1.5)

grid on

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
view(-40,20)

hold off
