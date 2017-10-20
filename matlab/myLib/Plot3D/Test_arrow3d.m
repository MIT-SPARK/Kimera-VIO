
hold off;
arrow3D([0,0,0]', [1,0,0]', 'r', 0.7);
arrow3D([0,0,0]', [0,1,0]', 'g', 0.7);
arrow3D([0,0,0]', [0,0,1]', 'b', 0.7);
axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
lighting phong; camlight right;

