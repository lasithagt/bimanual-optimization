function [x,P]=ekf(fstate,x,u,P,hmeas,z,Q,R)
% EKF   Extended Kalman Filter for nonlinear dynamic systems
% [x, P] = ekf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system:
%           x_k+1 = f(x_k) + + u_k + w_k
%           z_k   = h(x_k) + v_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   f: function handle for f(x)
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%           h: fanction handle for h(x)
%           z: current measurement
%           Q: process noise covariance 
%           R: measurement noise covariance
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance
%
% By Yi Cao at Cranfield University, 02/01/2008
%
global dt
[x1, F, u_tilda] = jaccsd(fstate, x, u);   % nonlinear update and linearization at current state

[z1, H] = jaccsd(hmeas, x1);    % nonlinear measurement and linearization

F   = expm(F*dt);
P   = F*P*F'+ Q;              % partial update
P12 = P*H';                   % cross covariance

K = P12/(H*P12+R);          % Kalman filter gain
x = x1' + K*(z-z1);           % state estimate
P = P-K*P12';               % state covariance matrix

function [z,F,u_tilda] = jaccsd(fun,x,u)
    global dt
    % JACCSD Jacobian through complex step differentiation
    % [z J] = jaccsd(f,x)
    % z = f(x)
    % J = f'(x)
    
    if (nargin == 2)
       z = fun(x);
    else
       z = fun(x,u)*dt + x'; 
    end
    
    n = numel(x);
    m = numel(z);
    F = zeros(m,n);
    
    h = n*eps;
    for k = 1:n
        x1 = x;
        x1(k) = x1(k) + h*i;
        
        if (nargin == 2)
            F(:,k) = imag(fun(x1))/h;
        else
            F(:,k) = imag(fun(x1,u))/h;
        end
    end
    
    % Linearized Input
    if (nargin > 2)

        u_tilda = z' - F*x;
    else
        u_tilda = 0;
    end
