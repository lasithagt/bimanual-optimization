init_a = [1.4269, 0, 4.1412, 3.9466]';
x      = init_a;

% define the dual arm robot
alpha  = [pi/2 -pi/2 -pi/2 0 pi/2 -pi/2 0];
offset = [0,0,0,-pi/2,0,-pi/2,0];
d      = [x(1) x(2) x(3) 0 0 0 0];
a      = [0 0 0 x(4) 0 0 0];
% tool   = input.tool;

L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
L7 = Link('d', 0, 'a',    0, 'alpha', alpha(7),'offset', offset(7));

optim_arm = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'manipulator');

optim_arm.plot([pi/3,pi/4,0,0,pi/4,pi/5,0],'workspace', 10*[-0.3 1.5 -1 1 -1 1.5], 'noshadow','noarrow', 'view',[-136 24], 'tile1color',[0.9 0.9 0.9],'delay',0.01);

% generate joint positions
theta = [pi/3,pi/4,pi/3,-pi/4,pi/4,pi/5,0;...
         pi/6,0,pi/6,-pi/4,pi/4,pi/5,0]';

% desired poses
desired_poses = zeros(6, size(theta,2));
for i = 1:size(theta, 2)
    desired_poses(:,i) = FK_analytical([theta(:,i);x]);
end

desired_poses(2,1) = desired_poses(2,1) + 0.1;

% joints limits
joint_limits = [-pi, -pi, -pi, -pi, -pi, -pi, -pi;
                 pi, pi, pi, pi, pi, pi, pi];
             
% design vector limits
design_limits = [0, 0, 0, 0;
                 4, 4, 4, 4];


% concatenate limits
n_waypoints = size(theta,2);
limits = [repmat(joint_limits, 1, n_waypoints), design_limits]';

% optimization on the manifols
manifold_optimization(x, theta, desired_poses, limits)

