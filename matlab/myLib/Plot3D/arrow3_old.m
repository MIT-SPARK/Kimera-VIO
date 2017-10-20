function hn=arrow3(p1,p2,s,w,h,ip,alpha,beta)
% ARROW3
%   ARROW3(P1,P2) will draw vector lines (2D/3D) from P1 to P2 with
%   arrowheads, where P1 and P2 can be either nx2 matrices (for 2D),
%   or nx3 matrices (for 3D).
%
%   ARROW3(P1,P2,S,W,H,IP,ALPHA,BETA) can be used to specify properties
%   of the line and arrowhead.  S is a character string made with one
%   element from any or all of the following 3 columns:
%
%      Color Switches       LineStyle             LineWidth
%      ------------------   -------------------   --------------------
%      k  black (default)   -   solid (default)   0.5 points (default)
%      y  yellow            :   dotted            0   no lines
%      m  magenta           -.  dashdot           /   LineWidthOrder
%      c  cyan              --  dashed
%      r  red               *   LineStyleOrder
%      g  green
%      b  blue
%      w  white                                    __________ __
%      a  apple green                            ^           |
%      d  dark gray                             / \          |
%      e  evergreen                            /   \         |
%      f  fuchsia                             /     \        |
%      h  honey                              /       \    Height
%      i  indigo                            /         \      |
%      j  jade                             /           \     |
%      l  lilac                           /             \    |
%      n  nutbrown                       /______   ______\ __|__
%      p  pink                           |      | |      |
%      q  kumquat                        |---- Width ----|
%      s  sky blue                       |      | |      |
%      t  tan                                   | |
%      u  umber                                 | |
%      v  violet                                | |
%      z  zinc                               -->| |<--LineWidth
%      x  random named color                    | |
%      o  ColorOrder
%      |  magnitude
%
%   The components of S may be specified in any order.  Invalid
%   characters in S will be ignored and replaced by default settings.
%
%   Prefixing the color code with '_' produces a darker shade, e.g.
%   '_t' is dark tan; prefixing the color code with '^' produces a
%   lighter shade, e.g. '^q' is light kumquat.  The relative
%   brightness of light and dark color shades is controled by the
%   scalar parameter BETA.  Color code prefixes do not affect the
%   basic colors (kymcrgbw) or the special color switches (xo|).
%
%   ColorOrder may be achieved in two fashions:  The user may either
%   set the ColorOrder property (using RGB triples) or define the
%   global variable ColorOrder (using a string of valid color codes).
%   If the color switch is specified with 'o', and the global variable
%   ColorOrder is a string of color codes (color switches less 'xo|',
%   optionally prefixed with '_' or '^'), then the ColorOrder property
%   will be set to the sequence of colors indicated by the ColorOrder
%   variable.  If the color switch is specified with 'o', and the
%   global variable ColorOrder is empty or invalid, then the current
%   ColorOrder property will be used.  Note that the ColorOrder
%   variable takes precedence over the ColorOrder property.
%
%   The magnitude color switch is used to visualize vector magnitudes
%   in conjunction with a colorbar.  If the color switch is specified
%   with '|', colors are linearly interpolated from the current ColorMap
%   according to the length of the associated line.  This option sets
%   CLim to [MinM,MaxM], where MinM and MaxM are the minimum and maximum
%   magnitudes, respectively.
%
%   The current LineStyleOrder property will be used if LineStyle is
%   specified with '*'.  MATLAB cycles through the line styles defined
%   by the LineStyleOrder property only after using all colors defined
%   by the ColorOrder property.  If however, the global variable
%   LineWidthOrder is defined, and LineWidth is specified with '/',
%   then each line will be drawn with sequential color, linestyle, and
%   linewidth.
%
%   W is a vector of arrowhead widths.  For linear plots with equal
%   axes, the units of W (default = 1/72 of the PlotBox diagonal) are
%   the same as those of the coordinate data (P1,P2).  For linear plots
%   with unequal axes, the units of W are scaled to fit as if the axes
%   were equal.  For log plots, the units of W (default = 1) are 1/72
%   of the PositionRectangle diagonal.
%
%   H (default = 3W) is a vector of arrowhead heights.  If vector IP is
%   neither empty nor negative, initial point markers will be plotted
%   with diameter IP; for default diameter W, use IP = 0.  The units of
%   W, H, and IP are absolute for linear plots and relative to the
%   PositionRectangle for log plots.
%
%   ALPHA (default = 1) is a vector of FaceAlpha (MATLAB 6+) values
%   ranging between 0 (clear) and 1 (opaque).  FaceAlpha is a surface
%   (arrowhead and initial point marker) property and does not affect
%   lines.  FaceAlpha is not supported for 2D rendering.
%
%   BETA (default = 0.4) is a scalar that controls the relative
%   brightness of light and dark color shades, ranging between 0 (no
%   contrast) and 1 (maximum contrast).
%
%   Plotting lines with a single color, linestyle, and linewidth is
%   faster than plotting lines with multiple colors and/or linestyles.
%   Plotting lines with multiple linewidths is slower still.  ARROW3
%   chooses renderers that produce the best screen images; exported
%   or printed plots may benifit from different choices.
%
%   HN = ARROW3(P1,P2,...) returns a vector of handles to line and
%   surface objects created by ARROW3.
%
%   ARROW3 COLORS will plot a table of named colors with default
%   brightness.  ARROW3('colors',BETA) will plot a table of named
%   colors with brightness BETA.
%
%   If a particular aspect ratio is required, use DASPECT, PBASPECT,
%   AXIS, or XYZLIM commands before calling ARROW3.  Changing limits
%   or aspect ratios after calling ARROW3 may alter the appearance
%   of arrowheads and initial point markers.  ARROW3 sets XYZCLimMode
%   to manual for all plots, sets DataAspectRatioMode to manual for
%   linear plots, and sets PlotBoxAspectRatioMode to manual for log
%   plots and 3D plots.
% 
%   ARROW3 UPDATE will restore the the appearance of arrowheads and
%   initial point markers that have become corrupted by changes to
%   limits or aspect ratios.  ARROW3('update',SF) will redraw initial
%   point markers and arrowheads with scale factor SF.  If SF has one
%   element, SF scales W, H and IP.  If SF has two elements, SF(1)
%   scales W and IP, and SF(2) scales H.  If SF has three elements,
%   SF(1) scales W, SF(2) scales H, and SF(3) scales IP.
%
%   ARROW3 UPDATE COLORS will update the magnitude coloring of
%   arrowheads, initial point markers, and lines to conform to the
%   current colormap.
%
%   HN = ARROW3('update',...) returns a vector of handles to updated
%   objects.
%
%   Usage Examples:
%
%      % 2D vectors
%      Arrow3([0 0],[1 3])
%      Arrow3([0 0],[1 2],'-.e')
%      Arrow3([0 0],[10 10],'--x2',1)
%      Arrow3(zeros(10,2),50*rand(10,2),'x',1,3)
%      Arrow3(zeros(10,2),[10*rand(10,1),500*rand(10,1)],'u')
%      Arrow3(10*rand(10,2),50*rand(10,2),'x',1,[],1)
%
%      % 3D vectors
%      Arrow3([0 0 0],[1 1 1])
%      Arrow3(zeros(20,3),50*rand(20,3),'--x1.5',2)
%      Arrow3(zeros(100,3),50*rand(100,3),'x',1,3)
%      Arrow3(zeros(10,3),[10*rand(10,1),500*rand(10,1),50*rand(10,1)],'a')
%      Arrow3(10*rand(10,3),50*rand(10,3),'x',[],[],0)
%
%      % Just for fun
%      Arrow3(zeros(100,3),50*rand(100,3),'x',10,3,[],0.95)
%      light('Position',[-10 -10 -10],'Style','local')
%      light('Position',[60,60,60]), lighting gouraud
%
%      % ColorOrder variable, color code prefixes, and Beta
%      global ColorOrder, ColorOrder='^ui^e_hq^v';
%      theta=[0:pi/22:pi/2]';
%      Arrow3(zeros(12,2),[cos(theta),sin(theta)],'1.5o',0.1,[],[],[],0.5)
%
%      % ColorOrder property, LineStyleOrder, and LineWidthOrder
%      global ColorOrder, ColorOrder=[];
%      set(gca,'ColorOrder',[1,0,0;0,0,1;0.25,0.75,0.25;0,0,0])
%      set(gca,'LineStyleOrder',{'-','--','-.',':'})
%      global LineWidthOrder, LineWidthOrder=[1,2,4,8];
%      w=[5,10,15,20]; h=[20,40,30,40];
%      Arrow3(zeros(4,2),[10*rand(4,1),500*rand(4,1)],'o*/',w,h,10)
%
%      % Magnitude coloring
%      colormap spring
%      Arrow3(20*randn(20,3),50*randn(20,3),'|',[],[],0)
%      set(gca,'color',0.7*[1,1,1])
%      set(gcf,'color',0.5*[1,1,1]), grid on, colorbar
%      pause % change the colormap and update colors
%      colormap hot
%      Arrow3('update','colors')
%
%      % LogLog plot
%      set(gca,'xscale','log','yscale','log');
%      axis([1e2,1e8,1e-2,1e-1]); hold on
%      p1=repmat([1e3,2e-2],15,1);
%      q1=[1e7,1e6,1e5,1e4,1e3,1e7,1e7,1e7,1e7,1e7,1e7,1e6,1e5,1e4,1e3];
%      q2=1e-2*[9,9,9,9,9,7,5,4,3,2,1,1,1,1,1]; p2=[q1',q2'];
%      global ColorOrder, ColorOrder=[];
%      set(gca,'ColorOrder',rand(15,3))
%      Arrow3(p1,p2,'o'), grid on, hold off
%
%      % SemiLogX plot
%      set(gca,'xscale','log','yscale','linear');
%      axis([1e2,1e8,1e-2,1e-1]); hold on
%      p1=repmat([1e3,0.05],15,1);
%      q1=[1e7,1e6,1e5,1e4,1e3,1e7,1e7,1e7,1e7,1e7,1e7,1e6,1e5,1e4,1e3];
%      q2=1e-2*[9,9,9,9,9,7,5,4,3,2,1,1,1,1,1]; p2=[q1',q2'];
%      Arrow3(p1,p2,'x'), grid on, hold off
%
%      % SemiLogY plot
%      set(gca,'xscale','linear','yscale','log');
%      axis([2,8,1e-2,1e-1]); hold on
%      p1=repmat([3,2e-2],17,1);
%      q1=[7,6,5,4,3,7,7,7,7,7,7,7,7,6,5,4,3];
%      q2=1e-2*[9,9,9,9,9,8,7,6,5,4,3,2,1,1,1,1,1]; p2=[q1',q2'];
%      set(gca,'LineStyleOrder',{'-','--','-.',':'})
%      Arrow3(p1,p2,'*',1,[],0), grid on, hold off
%
%      % Color tables
%      Arrow3('colors')           % default color table
%      Arrow3('colors',0.3)       % low contrast color table
%      Arrow3('colors',0.5)       % high contrast color table
%
%      % Update initial point markers and arrowheads
%      Arrow3('update')           % redraw same size
%      Arrow3('update',2)         % redraw double size
%      Arrow3('update',0.5)       % redraw half size
%      Arrow3('update',[0.5,2,1]) % redraw W half size,
%                                 %        H double size, and
%                                 %        IP same size

%   Copyright(c)2002,2003 Version 4.58
%      Jeff Chang <cpmame@hotmail.com>
%      Tom Davis  <tdavis@eng.usf.edu>

%   Revision History:
%
%      12/17/03 - Semilog examples, CAXIS support, magnitude
%                 coloring, and color updating; use CData instead
%                 of FaceColor; minor bug fixes (TD)
%      07/17/03 - Changed 2D rendering from OpenGL to ZBuffer;
%                 defined HN for COLORS and UPDATE options (TD)
%      02/27/03 - Replaced calls to RANDPERM, VIEW, REPMAT, SPHERE,
%                 and CYLINDER; added ZBuffer for log plots, RESET
%                 for CLA and CLF, and ABS for W and H (TD)
%      02/01/03 - Added UPDATE scale factor and matlab version
%                 checking, replaced call to CROSS (TD)
%      12/26/02 - Added UserData and UPDATE option (TD)
%      11/16/02 - Added more named colors, color code prefix,
%                 global ColorOrder, ALPHA , and BETA (TD)
%      10/12/02 - Added global LineWidthOrder,
%                 vectorized W, H and IP (TD)
%      10/05/02 - Changed CLF to CLA for subplot support,
%                 added ColorOrder and LineStyleOrder support (TD)
%      04/27/02 - Minor log plot revisions (TD)
%      03/26/02 - Added log plot support (TD)
%      03/24/02 - Adaptive grid spacing control to trade off
%                 appearance vs. speed based on size of matrix (JC)
%      03/16/02 - Added "axis tight" for improved appearance (JC)
%      03/12/02 - Added initial point marker (TD)
%      03/03/02 - Added aspect ratio support (TD)
%      03/02/02 - Enchance program's user friendliness (JC)
%                 (lump Color, LineStyle, and LineWidth together)
%      03/01/02 - Replaced call to ROTATE (TD)
%      02/28/02 - Modified line plotting,
%                 added linewidth and linestyle (TD)
%      02/27/02 - Minor enhancements on 3D appearance (JC)
%      02/26/02 - Minor enhancements for speed (TD&JC)
%      02/26/02 - Optimise PLOT3 and SURF for speed (TD)
%      02/25/02 - Return handler, error handling, color effect,
%                 generalize for 2D/3D vectors (JC)
%      02/24/02 - Optimise PLOT3 and SURF for speed (TD)
%      02/23/02 - First release (JC&TD)

%==========================================================================
% Error Checking
oldver=0; v=version; if v(1)<'6', oldver=1; end % matlab version
if nargin<8 | isempty(beta), beta=0.4; end
beta=abs(beta(1)); if nargout, hn=[]; end
if strcmpi(p1,'colors')                         % plot color table
   if nargin>1, beta=abs(p2(1)); end
   LocalColorTable(1,beta); return
end
fig=gcf; ax=gca;
if strcmpi(p1,'update'), ud=get(ax,'UserData'); % update
   if size(ud,2)<13, error('Invalid UserData'), end
   set(ax,'UserData',[]); sf=[1,1,1]; flag=0;
   if nargin>1
      if strcmp(p2,'colors'), flag=1;           % update colors
      else                                      % update surfaces
         sf=p2(1)*sf; n=length(p2(:));
         if n>1, sf(2)=p2(2); if n>2, sf(3)=p2(3); end, end
      end
   end
   H=LocalUpdate(fig,ax,ud,sf,flag); if nargout, hn=H; end, return
end
InputError=['Invalid input, type HELP ',upper(mfilename),...
   ' for usage examples'];
if nargin<2, error(InputError), end
[r1,c1]=size(p1); [r2,c2]=size(p2); n=r1; Zeros=zeros(n,1);
if c1<2 | c1>3, error(InputError), end
if r1~=r2, error('P1 and P2 must have same number of rows'), end
if c1~=c2, error('P1 and P2 must have same number of columns'), end
if c1==2, p1=[p1,Zeros]; p2=[p2,Zeros];
elseif ~any([p1(:,3);p2(:,3)]), c1=2; end
L=get(ax,'LineStyleOrder'); C=get(ax,'ColorOrder');
ST=get(ax,'DefaultSurfaceTag'); LT=get(ax,'DefaultLineTag');
EC=get(ax,'DefaultSurfaceEdgeColor');
if strcmp(get(ax,'nextplot'),'add') & strcmp(get(fig,'nextplot'),'add')
   Xr=get(ax,'xlim'); Yr=get(ax,'ylim'); Zr=get(ax,'zlim');
   set(ax,'XLimMode','auto','YLimMode','auto','ZLimMode','auto',...
      'CLim',get(ax,'CLim'));
   xs=strcmp(get(ax,'xscale'),'log');
   ys=strcmp(get(ax,'yscale'),'log');
   zs=strcmp(get(ax,'zscale'),'log');
   if zs, error('Z log scale not supported'), end
   xys=xs+ys; restore=1;
   if xys & any([p1(:,3);p2(:,3)])
      error('3D log plot not supported')
   end
else, restore=0; cla reset; xys=0; set(fig,'nextplot','add');
   if c1==2, azel=[0,90]; else, azel=[-37.5,30]; end
   set(ax,'UserData',[],'nextplot','add','View',azel);
end

%==========================================================================
% Style Control
[vc,cn]=LocalColorTable(0); prefix=''; OneColor=0;
if nargin<3, [c,ls,lw]=LocalValidateCLSW; % default Color, LineStyle/Width
else, 
   [c,ls,lw]=LocalValidateCLSW(s);
   if length(c)>1, if sum('_^'==c(1)), prefix=c(1); end, c=c(2); end
   if c=='x'                              % random named color (less white)
      [ignore,i]=sort(rand(1,23)); c=cn(i,:);
   elseif c=='o', global ColorOrder       % ColorOrder
      if length(ColorOrder)
         [c,failed]=LocalColorMap(lower(ColorOrder),vc,cn,beta);
         if failed, warning(['Invalid ColorOrder variable, ',...
            'current ColorOrder property will be used'])
         else, C=c; end
      end, c=C;
   elseif c=='|', map=colormap;           % magnitude coloring
      M=(p1-p2); M=sqrt(sum(M.*M,2));
      minM=min(M); maxM=max(M); set(ax,'clim',[minM,maxM]);
      c=interp1(linspace(minM,maxM,size(map,1)),map,M);
      c(c<0)=0; c(c>1)=1;
   elseif ~sum(vc==c), c='k'; warning(['Invalid color switch, ',...
      'default color (black) will be used'])
   end
end
if length(c)==1                                    % single color
   c=LocalColorMap([prefix,c],vc,cn,beta); OneColor=1;
end
set(ax,'ColorOrder',c); c=LocalRepmat(c,[ceil(n/size(c,1)),1]);
if ls~='*', set(ax,'LineStyleOrder',ls); end       % LineStyleOrder
if lw=='/', global LineWidthOrder                  % LineWidthOrder
   if length(LineWidthOrder)
      lw=LocalRepmat(LineWidthOrder(:),[ceil(n/length(LineWidthOrder)),1]);
   else, lw=0.5; warning(['Undefined LineWidthOrder, ',...
      'default width (0.5) will be used'])
   end
end
if nargin<7 | isempty(alpha), alpha=1; end
a=LocalRepmat(alpha(:),[ceil(n/length(alpha)),1]); % FaceAlpha

%==========================================================================
% Log Plot
if xys
   units=get(ax,'units'); set(ax,'units','points');
   pos=get(ax,'position'); set(ax,'units',units);
   if strcmp(get(ax,'PlotBoxAspectRatioMode'),'auto')
      set(ax,'PlotBoxAspectRatio',[pos(3),pos(4),1]);
   end
   par=get(ax,'PlotBoxAspectRatio');
   set(ax,'DataAspectRatio',[par(2),par(1),par(3)]);
   % map coordinates onto unit square
   q=[p1;p2]; xr=Xr; yr=Yr;
   if xs, xr=log10(xr); q(:,1)=log10(q(:,1)); end
   if ys, yr=log10(yr); q(:,2)=log10(q(:,2)); end
   q=q-LocalRepmat([xr(1),yr(1),0],[2*n,1]);
   dx=xr(2)-xr(1); dy=yr(2)-yr(1);
   q=q*diag([1/dx,1/dy,1]);
   q1=q(1:n,:); q2=q(n+1:end,:);
else, xs=0; ys=0; dx=0; dy=0; xr=0; yr=0; end

%==========================================================================
% Line
set(ax,'DefaultLineTag','arrow3');
if length(lw)==1
   if lw>0
      if OneColor & ls~='*'   % single color, linestyle, and linewidth
         P=zeros(3*n,3); i=1:n;
         P(3*i-2,:)=p1(i,:); P(3*i-1,:)=p2(i,:); P(3*i,1)=NaN;
         H1=plot3(P(:,1),P(:,2),P(:,3),'LineWidth',lw);
      else                    % single linewidth
         H1=plot3([p1(:,1),p2(:,1)]',[p1(:,2),p2(:,2)]',...
            [p1(:,3),p2(:,3)]','LineWidth',lw);
      end
   else, H1=[]; end
else                          % use LineWidthOrder
   ls=LocalRepmat(cellstr(L),[ceil(n/size(L,1)),1]);
   H1=Zeros;
   for i=1:n
      H1(i)=plot3([p1(i,1),p2(i,1)],[p1(i,2),p2(i,2)],[p1(i,3),p2(i,3)],...
         ls{i},'Color',c(i,:),'LineWidth',lw(i));
   end
end

%==========================================================================
% Scale
ar=get(ax,'DataAspectRatio'); ar=sqrt(3)*ar/norm(ar);
set(ax,'DataAspectRatioMode','manual');
if nargin<4 | isempty(w)             % width
   if xys, w=1;
   else, xr=get(ax,'xlim'); yr=get(ax,'ylim'); zr=get(ax,'zlim');
      w=norm([xr(2)-xr(1),yr(2)-yr(1),zr(2)-zr(1)])/72;
   end
end
w=LocalRepmat(abs(w(:)),[ceil(n/length(w)),1]);
if nargin<5 | isempty(h), h=3*w; end % height
h=LocalRepmat(abs(h(:)),[ceil(n/length(h)),1]);
if nargin>5 & ~isempty(ip)           % ip
   i=find(ip==0); ip(i)=w(i);
   ip=LocalRepmat(ip(:),[ceil(n/length(ip)),1]);
else, ip=-ones(n,1); end

%==========================================================================
% UserData
set(ax,'UserData',[get(ax,'UserData');...
   p1,p2,c(1:n,:),w(1:n),h(1:n),ip(1:n),a]);

%==========================================================================
% Arrowhead
if xys, whip=[w,h,ip]*sqrt(2)/72;
   w=whip(:,1); h=whip(:,2); ip=whip(:,3); p1=q1; p2=q2;
end
W=(p1-p2)./LocalRepmat(ar,[n,1]);
W=W./LocalRepmat(sqrt(sum(W.*W,2)),[1,3]);  % new z direction
U=[-W(:,2),W(:,1),Zeros];
N=sqrt(sum(U.*U,2)); i=find(N<eps); j=length(i);
U(i,:)=LocalRepmat([1,0,0],[j,1]); N(i)=ones(j,1);
U=U./LocalRepmat(N,[1,3]);                  % new x direction
V=[W(:,2).*U(:,3)-W(:,3).*U(:,2),...        % new y direction
   W(:,3).*U(:,1)-W(:,1).*U(:,3),...
   W(:,1).*U(:,2)-W(:,2).*U(:,1)];

m1=30; m2=10; num=200;               % max, min grid spacing, and threshold
if n<num, m=round((m2-m1)*n/num+m1); % adjust grid spacing automatically
else m=m2; end                       % for speed when matrix size > num

set(ax,'DefaultSurfaceTag','arrow3','DefaultSurfaceEdgeColor','none');
r=[0;1]; theta=(0:m)/m*2*pi; Ones=ones(1,m+1);
x=r*cos(theta); y=r*sin(theta); z=r*Ones;
G=surface(x/2,y/2,z); dar=diag(ar);
X=get(G,'XData'); Y=get(G,'YData'); Z=get(G,'ZData');
H2=Zeros; [j,k]=size(X);
for i=1:n                            % translate, rotate, and scale
   H2(i)=copyobj(G,ax);
   xyz=[w(i)*X(:),w(i)*Y(:),h(i)*Z(:)]*[U(i,:);V(i,:);W(i,:)]*dar;
   x=reshape(xyz(:,1),j,k)+p2(i,1);
   y=reshape(xyz(:,2),j,k)+p2(i,2);
   z=reshape(xyz(:,3),j,k)+p2(i,3);
   LocalSetSurface(xys,xs,ys,dx,dy,xr,yr,...
      x,y,z,a(i),c(i,:),H2(i),2,m+1,oldver);
end, delete(G);

%==========================================================================
% Initial Point Marker
if any(ip>0)
   theta=(-m:2:m)/m*pi; phi=(-m:2:m)'/m*pi/2; cosphi=cos(phi);
   x=cosphi*cos(theta); y=cosphi*sin(theta); z=sin(phi)*Ones;
   G=surface(x*ar(1)/2,y*ar(2)/2,z*ar(3)/2);
   X=get(G,'XData'); Y=get(G,'YData'); Z=get(G,'ZData');
   H3=zeros(n,1);
   for i=1:n                         % translate
      if ip(i)>0
         H3(i)=copyobj(G,ax);
         x=p1(i,1)+X*ip(i); y=p1(i,2)+Y*ip(i); z=p1(i,3)+Z*ip(i);
         LocalSetSurface(xys,xs,ys,dx,dy,xr,yr,...
            x,y,z,a(i),c(i,:),H3(i),m+1,m+1,oldver);
      end
   end, delete(G);
else, H3=[]; end

%==========================================================================
% Finish
if xys, xr=Xr; yr=Yr; zr=Zr;
   set(ax,'DataAspectRatioMode','auto');
else % "nearly" tight limits (much faster than "axis tight")
   w=max(w)*ar/2; ip=max([ip;0])*ar/2;
   xr=[min([p1(:,1)-ip(1);p2(:,1)-w(1)]),max([p1(:,1)+ip(1);p2(:,1)+w(1)])];
   yr=[min([p1(:,2)-ip(2);p2(:,2)-w(2)]),max([p1(:,2)+ip(2);p2(:,2)+w(2)])];
   zr=[min([p1(:,3)-ip(3);p2(:,3)-w(3)]),max([p1(:,3)+ip(3);p2(:,3)+w(3)])];
   if restore
      xr=[min(xr(1),Xr(1)),max(xr(2),Xr(2))];
      yr=[min(yr(1),Yr(1)),max(yr(2),Yr(2))];
      zr=[min(zr(1),Zr(1)),max(zr(2),Zr(2))];
   else, set(ax,'nextplot','replace'); end
end
azel=get(ax,'view');
if azel(2)==90, renderer='ZBuffer'; else, renderer='OpenGL'; end
set(fig,'Renderer',renderer);
set(ax,'LineStyleOrder',L,'ColorOrder',C,'DefaultLineTag',LT,...
   'DefaultSurfaceTag',ST,'DefaultSurfaceEdgeColor',EC,...
   'xlim',xr,'ylim',yr,'zlim',zr,'clim',get(ax,'CLim'));
if c1==3, set(ax,'CameraViewAngle',get(ax,'CameraViewAngle'),...
   'PlotBoxAspectRatio',get(ax,'PlotBoxAspectRatio'));
end
if nargout, hn=[H1(:);H2(:);H3(:)]; end

%==========================================================================
% Update
function H=LocalUpdate(fig,ax,ud,sf,flag)
p1=ud(:,1:3); p2=ud(:,4:6); c=ud(:,7:9); a=ud(:,13);
w=sf(1)*ud(:,10); h=sf(2)*ud(:,11); ip=sf(3)*ud(:,12);
H=get(ax,'children'); tag=get(H,'tag'); type=get(H,'type');
delete(H(strcmp(tag,'arrow3') & strcmp(type,'surface')));
set(fig,'nextplot','add'); set(ax,'nextplot','add');
if flag, map=colormap;               % update colors
   M=(p1-p2); M=sqrt(sum(M.*M,2)); minM=min(M); maxM=max(M);
   H1=H(strcmp(tag,'arrow3') & strcmp(type,'line'));
   for i=1:length(H1)
      x=get(H1(i),'xdata'); y=get(H1(i),'ydata'); z=get(H1(i),'zdata');
      if length(x)>2                 % multiple lines
         warning(sprintf(['Cannot perform magnitude coloring ',...
            'on lines that \nwere drawn with a single color, ',...
            'linestyle, and linewidth'])), break
      end
      M=sqrt((x(1)-x(2))^2+(y(1)-y(2))^2+(z(1)-z(2))^2);
      c=interp1(linspace(minM,maxM,size(map,1)),map,M);
      c(c<0)=0; c(c>1)=1; set(H1(i),'color',c);
   end
   H=arrow3(p1,p2,'|0',w,h,ip,a); H=[H1(:);H(:)];
else                                 % update surfaces
   set(ax,'ColorOrder',c); global ColorOrder, ColorOrder=[];
   H=arrow3(p1,p2,'o0',w,h,ip,a);
end
set(ax,'nextplot','replace');

%==========================================================================
% SetSurface
function LocalSetSurface(xys,xs,ys,dx,dy,xr,yr,x,y,z,a,c,H,n,m,oldver)
if xys
   x=x*dx+xr(1); y=y*dy+yr(1);
   if xs, x=10.^x; end
   if ys, y=10.^y; end
end
cd=zeros(n,m,3); cd(:,:,1)=c(1); cd(:,:,2)=c(2); cd(:,:,3)=c(3);
if oldver
   set(H,'XData',x,'YData',y,'ZData',z,'CData',cd);
else
   set(H,'XData',x,'YData',y,'ZData',z,'CData',cd,'FaceAlpha',a);
end

%==========================================================================
% ColorTable
function [vc,cn]=LocalColorTable(n,beta)
vc='kymcrgbadefhijlnpqstuvzw';       % valid color codes
%   k               y               m               c
cn=[0.00,0.00,0.00; 1.00,1.00,0.00; 1.00,0.00,1.00; 0.00,1.00,1.00;
%   r               g               b               a
    1.00,0.00,0.00; 0.00,1.00,0.00; 0.00,0.00,1.00; 0.00,0.70,0.00;
%   d               e               f               h
    0.40,0.40,0.40; 0.00,0.40,0.00; 0.90,0.00,0.40; 1.00,0.80,0.00;
%   i               j               l               n
    0.00,0.00,0.70; 0.20,0.80,0.50; 0.80,0.40,0.80; 0.50,0.20,0.00;
%   p               q               s               t
    1.00,0.40,0.60; 1.00,0.40,0.00; 0.00,0.80,1.00; 0.80,0.40,0.00;
%   u               v               z               w
    0.70,0.00,0.00; 0.60,0.00,1.00; 0.60,0.60,0.60; 1.00,1.00,1.00;];
if n, clf reset                      % plot color table
   name={'black','yellow','magenta','cyan',...
      'red','green','blue','apple green',...
      'dark gray','evergreen','fuchsia','honey',...
      'indigo','jade','lilac','nutbrown',...
      'pink','kumquat','sky blue','tan',...
      'umber','violet','zinc','white'};
   c=['yhtn';'gjae';'csbi';'plmv';'frqu';'wzdk'];
   set(gcf,'DefaultAxesXTick',[],'DefaultAxesYTick',[],...
      'DefaultAxesXTickLabel',[],'DefaultAxesYTickLabel',[],...
      'DefaultAxesXLim',[0,0.75],'DefaultAxesYLim',[0,0.75],...
      'DefaultRectangleEdgeColor','none');
   for i=1:24
      subplot(4,6,i); j=find(vc==c(i)); title(name{j});
      dark=LocalBrighten(cn(j,:),-beta);
      light=LocalBrighten(cn(j,:),beta);
      rectangle('Position',[0,0.00,0.75,0.25],'FaceColor',dark);
      rectangle('Position',[0,0.25,0.75,0.25],'FaceColor',cn(j,:));
      rectangle('Position',[0,0.50,0.75,0.25],'FaceColor',light);
      rectangle('Position',[0,0.00,0.75,0.75],'EdgeColor','k');
      if rem(i,6)==1
         set(gca,'YTickLabel',{'dark','normal','light'},...
            'YTick',[0.125,0.375,0.625]);
         if i==19
            text(0,-0.25,['{\bf\itARROW3}  Named Color Table  ',...
                  '( \beta = ',num2str(beta),' )']);
         end
      end
   end
end

%==========================================================================
% ColorMap
function [C,failed]=LocalColorMap(c,vc,cn,beta)
n=length(c); failed=0; C=zeros(n,3); i=1; j=1;
while 1
   if isempty(find([vc,'_^']==c(i))), failed=1; break, end
   if sum('_^'==c(i))
      if i+1>n, failed=1; break, end
      if ~sum(vc==c(i+1)), failed=1; break, end
      cc=cn(find(vc==c(i+1)),:); gamma=beta;
      if c(i)=='_', gamma=-beta; end
      C(j,:)=LocalBrighten(cc,gamma); i=i+2;
   else, C(j,:)=cn(find(vc==c(i)),:); i=i+1; end
   if i>n, break, end, j=j+1;
end, if n>j, C(j+1:n,:)=[]; end

%==========================================================================
% Brighten
function C=LocalBrighten(c,beta)
C=c.^((1-min(1-sqrt(eps),abs(beta)))^sign(beta));

%==========================================================================
% Repmat
function B = LocalRepmat(A,siz)
if length(A)==1, B(prod(siz))=A; B(:)=A; B=reshape(B,siz);
else, [m,n]=size(A); mind=(1:m)'; nind=(1:n)';
   mind=mind(:,ones(1,siz(1))); nind=nind(:,ones(1,siz(2)));
   B=A(mind,nind);
end

%==========================================================================
% Generate valid value for color, linestyle and linewidth
function [c,ls,lw]=LocalValidateCLSW(s)
if nargin<1, c='k'; ls='-'; lw=0.5;
else
   % identify linestyle
   if findstr(s,'--'), ls='--'; s=strrep(s,'--','');
   elseif findstr(s,'-.'), ls='-.'; s=strrep(s,'-.','');
   elseif findstr(s,'-'), ls='-'; s=strrep(s,'-','');
   elseif findstr(s,':'), ls=':'; s=strrep(s,':','');
   elseif findstr(s,'*'), ls='*'; s=strrep(s,'*','');
   else, ls='-'; end

   % identify linewidth
   tmp=double(s);
   tmp=find(tmp>45 & tmp<58);
   if length(tmp)
      if any(s(tmp)=='/'), lw='/'; else, lw=str2num(s(tmp)); end
      s(tmp)='';
   else, lw=0.5; end

   % identify color
   if length(s), s=lower(s);
      if length(s)>1, c=s(1:2);
      else, c=s(1); end
   else, c='k'; end
end