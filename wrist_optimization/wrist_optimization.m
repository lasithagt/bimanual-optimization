T = @(R,d)[R d(1:3);0 0 0 1];

global d_max;
d_max = 0.5;

lowerBounds = [0.5, 0.25, 2];
upperBounds = [3, 1.2, 2];

A = [];
b = [];
Aeq = [];
beq = [];
options = optimoptions('fmincon', 'algorithm', 'interior-point');
fmincon()

x = ones(1,5);
x = [x, pi/3];
fk_wrist_roll(x)

% cost function for the 
function ret = fk_wrist_roll(x)
    global d_max
    % variables
    l1 = x(1); l2 = x(2); l3 = x(3); l4 = x(4); l5 = x(5); th = x(6);
    T1 = [eye(3),   [0;0;l1];0 0 0 1];
    T2 = [eye(3),   [0;l2;0];0 0 0 1];
    T3 = @(d)[eye(3),   [0;0;l3+d];0 0 0 1];
    T4 = [rotx(th), [0;l4;0];0 0 0 1];
    T5 = [eye(3),   [0;l5;0];0 0 0 1];
    
    T_0 = T1 * T2 * T3(0) * T4 * T5;
    T_M = T1 * T2 * T3(d_max) * T4 * T5;
    
    theta_0 = rotm2eul(T_0(1:3,1:3),'XYZ');
    theta_M = rotm2eul(T_M(1:3,1:3),'XYZ');
    
    ret = theta_M - theta_0;
end

function ret = fk_wrist_pitch(x)

    l1 = x(1); l2 = x(2); l3 = x(3); l4 = x(4); l5 = x(5);
    T1 = [eye(3),   [0;0;l1];0 0 0 1];
    T2 = [eye(3),   [0;l2;0];0 0 0 1];
    T3 = @(d)[eye(3),   [0;0;l3+d];0 0 0 1];
    T4 = [rotx(th), [0;l4;0];0 0 0 1];
    T5 = [eye(3),   [0;l5;0];0 0 0 1];
    
    T_0 = T1 * T2 * T3(0) * T4 * T5;
    T_M = T1 * T2 * T3(d_max) * T4 * T5;
    
    theta_0 = rotm2eul(T_0(1:3,1:3),'XYZ');
    theta_M = rotm2eul(T_M(1:3,1:3),'XYZ');
    
    ret = theta_M - theta_0;
end

