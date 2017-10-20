function R = quat2rot(h)

% Uso: R = quat2rot(h)
%
% costruisce la matrice di rotazione R
% a partire dal quaternione h corrispondente

s(4)=h(1);
s(1)=h(2);
s(2)=h(3);
s(3)=h(4);

R(1,1)=s(1)^2-s(2)^2-s(3)^2+s(4)^2;
R(1,2)=2*(s(1)*s(2)-s(3)*s(4));
R(1,3)=2*(s(1)*s(3)+s(2)*s(4));

R(2,1)=2*(s(1)*s(2)+s(3)*s(4));
R(2,2)=-s(1)^2+s(2)^2-s(3)^2+s(4)^2;
R(2,3)=2*(s(2)*s(3)-s(1)*s(4));

R(3,1)=2*(s(1)*s(3)-s(2)*s(4));
R(3,2)=2*(s(2)*s(3)+s(1)*s(4));
R(3,3)=-s(1)^2-s(2)^2+s(3)^2+s(4)^2;
