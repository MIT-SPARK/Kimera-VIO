function [newx,newy,newz]=rotatedata(xdata,ydata,zdata,azel,alpha,origin)
% 
% ROTATEDATA rotate data about specified origin and direction.
% 
%   ROTATEDATA(Xdata,Ydata,Zdata,[THETA PHI],ALPHA,ORIGIN) rotates the objects with handles H
%   through angle ALPHA about an axis described by the 2-element
%   direction vector [THETA PHI] (spherical coordinates).  
%   All the angles are in degrees.  The handles in H must be children
%   of the same axes.
%
%   THETA is the angle in the xy plane counterclockwise from the
%   positive x axis.  PHI is the elevation of the direction vector
%   from the xy plane (see also SPH2CART).  Positive ALPHA is defined
%   as the righthand-rule angle about the direction vector as it
%   extends from the origin.
%
%   ROTATEDATA(Xdata,Ydata,Zdata,[X Y Z],ALPHA,ORIGIN) rotates the objects about the direction
%   vector [X Y Z] (Cartesian coordinates). The direction vector
%   is the vector from the center of the plot box to (X,Y,Z).
%
%   See also SPH2CART, CART2SPH.
%
%   Modified by ChangShun Deng
%   Email: heroaq_2002@163.com
%   Web-Log: http://waitingforme.yculblog.com
%   2005/3/4
%
%   Copyright 1984-2002 The MathWorks, Inc. 
%   $Revision: 5.17 $  $Date: 2002/06/05 20:05:16 $

%判断输入变量的个数
if nargin<6
    error('Not enough input arguments! Type ''help rotatedata'' to get some help!')
end
%找到旋转的单位轴向量
if prod(size(azel)) == 2 % theta, phi
    theta = pi*azel(1)/180;
    phi = pi*azel(2)/180;
    u = [cos(phi)*cos(theta); cos(phi)*sin(theta); sin(phi)];
elseif prod(size(azel)) == 3 % direction vector
    u = azel(:)/norm(azel);
end

alph = alpha*pi/180;
cosa = cos(alph);
sina = sin(alph);
vera = 1 - cosa;
x = u(1);
y = u(2);
z = u(3);
%旋转矩阵
rot = [cosa+x^2*vera x*y*vera-z*sina x*z*vera+y*sina; ...
        x*y*vera+z*sina cosa+y^2*vera y*z*vera-x*sina; ...
        x*z*vera-y*sina y*z*vera+x*sina cosa+z^2*vera]';
[m,n] = size(xdata);
if isempty(z)
    z=zeros(size(x));%在对二维数据进行旋转时，z参数输入可以是空矩阵
end
newxyz = [xdata(:)-origin(1), ydata(:)-origin(2), zdata(:)-origin(3)];
newxyz = newxyz*rot;
newx = origin(1) + reshape(newxyz(:,1),m,n);
newy = origin(2) + reshape(newxyz(:,2),m,n);
newz = origin(3) + reshape(newxyz(:,3),m,n);