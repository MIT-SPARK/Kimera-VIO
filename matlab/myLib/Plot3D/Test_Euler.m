%
R_0=eye(3)*1;
R_z1=rotz(30);
R_x=rotx(-30);
R_z2=rotz(30);

Ref=R_0;

colorrgb=['r','g','b'];
coloryellow=['y','y','y'];
stemRatio=0.8;
rhoRatio=0.025;
axscale=0.1;

ref_frame(Ref,[0,0,0],colorrgb,stemRatio,rhoRatio,axscale)
lighting phong; 
camlight left;

% ref_frame(Ref,[1.5,0,0],colorrgb,stemRatio,rhoRatio,axscale)
% Ref=Ref*R_z1;
% ref_frame(Ref,[1.5,0,0],colorrgb,stemRatio,rhoRatio,axscale)
% 
% ref_frame(Ref,[3,0,0],colorrgb,stemRatio,rhoRatio,axscale)
% Ref=Ref*R_x;
% ref_frame(Ref,[3,0,0],colorrgb,stemRatio,rhoRatio,axscale)
% 
% ref_frame(Ref,[4.5,0,0],colorrgb,stemRatio,rhoRatio,axscale)
% Ref=Ref*R_z2;
% ref_frame(Ref,[4.5,0,0],colorrgb,stemRatio,rhoRatio,axscale)
% 
% ref_frame(Ref,[6,0,0],colorrgb,stemRatio,rhoRatio,axscale)

