% function w=logrot(R)
%LOGROT is the inverse of ROT, i.e. W is a 3x1 vector s.t. R=rot(W)


% Roberto Tron (tron@cis.jhu.edu)


% Ver: 07-Feb-2014 14:54:03


function [w,theta]=logrot(R,method)
if nargin<2
    %method='mex';
    method='quaternion';
    %method='svd';
end
flagOutputTheta=false;

if nargout>1
    flagOutputTheta=true;
end
method=lower(method);
N=size(R,3);
w=zeros(3,N);
if flagOutputTheta
    theta=zeros(1,N);
end

if ~strcmp(method,'mex')
    for r=1:N
        switch (method)
            case 'trace'
                R1=R(:,:,r);
                %theta1=acos(max(min((trace(R(:,:,r))-1)/2,1),-1));
                theta1=acos(max(min((R1(1)+R1(5)+R1(9)-1)/2,1),-1));
                if(theta1<1e-15)
                    w(:,r)=[0;0;0];
                else
                    %w(:,r)=0.5/sin(theta1)*[R(3,2,r)-R(2,3,r);R(1,3,r)-R(3,1,r);R(2,1,r)-R(1,2,r)];
                    w(:,r)=0.5/sin(theta1)*[R1(6)-R1(8);R1(7)-R1(3);R1(2)-R1(4)];
                end
                if flagOutputTheta
                    theta(r)=theta1;
                else
                    w(:,r)=theta1*w(:,r);
                end
            case 'quaternion'
                %Following the discussion from
                %http://www.mathworks.com/matlabcentral/newsreader/view_thread/160945
                %using quaternions seems to be (numerically) the most accurate way to
                %compute angle and axis of rotation
                [theta1 w(:,r)] = angleaxis(dc2quat(R(:,:,r)));
                if flagOutputTheta
                    theta(r)=theta1;
                    w(:,r)=-w(:,r);
                else
                    w(:,r)=theta1*-w(:,r);
                end

    %         case 'svd'
    %             w(:,r)=vee(rot_log(eye(3),R(:,:,r)));
        end
    end
else
    [w theta]=mexLogrot(R);
    if ~flagOutputTheta
        w=w.*(ones(3,1)*theta);
    end
end
