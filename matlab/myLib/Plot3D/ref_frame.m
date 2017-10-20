function ref_frame(Ref,OriginRef,colorCode,stemRatio,rhoRatio,axscale)

% ref_frame plots a 3D representation of a reference frame 
% given by three right orthogonal unit vectors
% 
% ref_frame(Ref,DimSpace,OriginRef) 
%
%  Ref is a 3 x 3 orthogonal matrix representing the unit vectors
%      of the reference frame to be drawn
%  OriginRef is the reference frame origin point
%      default value = [0 0 0]'
%  colorcode is the code color of each axis
%      default value = ['r','g','b']
%  rhoRatio is the ratio of the cylinder radius
%      default value = 0.05
%  stemRatio is the ratio of the stem length wrt the arrowhead length
%      default value = 0.75
%  axscale is the common scale factor that multiplies the axes lenght
%      default value = 1.0

%	Copyright (C) Basilio Bona 2007

n=nargin;
if n == 1
    OriginRef=[0 0 0]';
    colorCode=['r','g','b'];
    rhoRatio=0.05;
    stemRatio=0.75;
    axscale=1.00;
end
if n == 2
    colorCode=['r','g','b'];
    rhoRatio=0.05;
    stemRatio=0.75;
    axscale=1.00;
 end    
if n == 3 
    rhoRatio=0.05;
    stemRatio=0.75;
    axscale=1.00;
end    
if n == 4 
    rhoRatio=0.05;
    axscale=1.00;
end    
if n == 5 
    axscale=1.00;
end    

% xproj=DimSpace(1,2); yproj=DimSpace(2,2); zproj=DimSpace(3,1);

Xm=10;Ym=10;Zm=10;

    DimSpace(1,1)=-Xm; DimSpace(1,2)=Xm;
    DimSpace(2,1)=-Ym; DimSpace(2,2)=Ym;
    DimSpace(3,1)=-Zm; DimSpace(3,2)=Zm;
    OriginRef=[0 0 0]';

PlotSpace(DimSpace,OriginRef)

hold on;
arrow3d(OriginRef, axscale*Ref(:,1), colorCode(1), stemRatio, rhoRatio);
arrow3d(OriginRef, axscale*Ref(:,2), colorCode(2), stemRatio, rhoRatio);
arrow3d(OriginRef, axscale*Ref(:,3), colorCode(3), stemRatio, rhoRatio);

lighting phong; 
camlight left;

%PlotSpace
axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');

