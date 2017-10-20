% function hArrow = arrowPlot(initPoints, finishPoints, color)  
%  draw N arrow(s) using #d surfaces, from coordinate(s) |initPoints| (3 column x N rows)
%  to coordinate |finishPoints| (coordinates in cartesian).
%  |color| is optional, it is a 3 element row (1 color for all arrows) or a 3 columns by
%  N rows array where N is the number of arrows (and incidently the number of rows for
%  initPoints and finishPoints).
%  Note : color has implementation problems when colored surfaces are already presents.
%			might mess up the colormap => might need to be desactivated!!
%
%
%	Examples:
%
%	sphere(20);
%   hold on;
%	axis equal;
%	arrowPlot([0, 0, 1.2], [0+0.8, 0, 1.2], [1, 0, 0]);
%	arrowPlot([0, 0, 1.2], [0, 0+0.8, 1.2], [0, 1, 0]);
%	arrowPlot([0, 0, 1.2], [0, 0, 1.2+0.8], [0, 0, 1]); 
%	set(gca,'zlim', [-1, 2])
%
%
%		Draw a kind of reference frame centered on position [0, 0, 1.2]
%		above a colored sphere centered on [0,0,0] with radius 1,
%		The 3 reference axis are along X, Y, Z of the figure, with length 0.8 and
%		have each a different color.
%
%	sphere(20);
%   hold on;
%	axis equal;
%	arrowPlot([[0, 0, 1.2]; [0, 0, 1.2]; [0, 0, 1.2]], ...
%			  [[0+0.8, 0, 1.2];[0, 0+0.8, 1.2] ; [0, 0, 1.2+0.8]], ...
%			  [[1, 0, 0]; [0, 1, 0] ; [0, 0, 1] ]);
%	set(gca,'zlim', [-1, 2])
%
%		Does the same as previous example, but arrowPlots uses an array for the 3 vectors.
%	
%
%	TODO : allow for different input type, i.e. origin point & incidence, azimuth and
%		   length of the arrow from this point.
%			Change aspect ratio for the arrow parts
%
%	Author : Emmanuel P. Dinnat
%
%	Date : 2005
%


function hArrow = arrowPlot(initPoints, finishPoints, varargin)  
if length(varargin) == 1
	colorPoints = varargin{1};
else
	colorPoints = [0.1,0.1,0.1];
end
lineWidth = 0.01;
for iPoint = 1:size(initPoints, 1)
init = initPoints(iPoint,:);
finish = finishPoints(iPoint,:);
color = colorPoints(min([iPoint, size(colorPoints, 1)]),:);
%% make surface for the line of the arrow with a cylinder of height = 0.9 and width = lineWidth
	[xc, yc, zc] = cylinder(lineWidth);
	zc = zc*0.9;
%% make surface for the tip of the arrow
	% make a cone with a base 300% larger than the line and a tip = 0
	[xt, yt, zt] = cylinder(lineWidth*3 - [0:0.1:1]*lineWidth*3);	
	% decrease the tip height to be 10% of the total arrow height
	zt = zt/10;
%% join both part, putting tip above line,  to make arrow object
	xa = [xc; xt];
	ya = [yc; yt];
	za = [zc; zt + 0.9];
%% rotate, resize and translate arrow
	lengthArrow = norm(finish - init);
	xa = xa*lengthArrow;
	ya = ya*lengthArrow;
	za = za*lengthArrow;
	% bring the asked arrow coordinates to origin
	finish_temp = finish - init;
	% derive rotation angles
	[phi_rot, th_rot, R] = cart2sph(finish_temp(1), finish_temp(2), finish_temp(3));
	th_rot = pi/2 - th_rot;
	% apply rotations to arrow
	% rotation around Y-axis
	[phiArrowZX, R] = cart2pol(za, xa);
	phiArrowZX = phiArrowZX + th_rot;
	[za2, xa2] = pol2cart(phiArrowZX, R);
	ya2 = ya;
	% rotation around Z-axis
	[phiArrowXY, R] = cart2pol(xa2, ya2);
	phiArrowXY = phiArrowXY + phi_rot;
	[xa3, ya3] = pol2cart(phiArrowXY, R);
	za3 = za2;
	% apply translation to arrow
	xa3 = xa3 + init(1);
	ya3 = ya3 + init(2);
	za3 = za3 + init(3);
%% plot arrow
hArrow = surf(xa3, ya3, za3);
set(hArrow, 'facecolor', color, 'edgecolor', 'none', ...
	'diffuseStrength', 0.5, 'AmbientStrength',0.5, ...
	'SpecularStrength', 1,...
	'meshstyle', 'row')
hold on
% let's try to restore original colors to already existing colored surfaces !!
climval = get(gca, 'clim');
set(hArrow, 'Cdata', zeros(13,21)+climval(1));

end
