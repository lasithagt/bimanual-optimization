close all
%% Dynamic Contact Parameter Estimation 
global dt;

dt = 0.01;
n = 4;                                        % number of state
q = [0.01;0.01;0.01;0.000001];                    % std of process 
r = 0.01;                                     % std of measurement
% Q = q^2*eye(n);                               
Q = diag(q.^2);                               % covariance of process
R = r^2;                                      % covariance of measurement  
% f = @(x)[x(2);x(3);0.05*x(1)*(x(2)+x(3))];  % nonlinear state equations
t  = 0:dt:1;
u  = 0.1*sin(t);
N  = numel(t);
f = @(x,u)nonLinearDynamics(x,u);
h_ = @(x)h(x);                                % measurement equation
s = [0;0;0;0];                               % initial state
x = s + q*randn; % initial state              % initial state with noise
P = eye(n);                                   % initial state covraiance
xV = zeros(n,N);          %estmate            % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(1,N);


for k=1:N
  z       = cos(t(k)) + r*randn;           % measurments
  sV(:,k) = s;                             % save actual state
  zV(k)   = z;                             % save measurment
  [x, P]  = ekf(f,x,u(k),P,h_,z,Q,R);      % ekf 
  xV(:,k) = x;                             % save estimate
  s       = s + f(s,u(k))'*dt + q*randn;   % update process 
end

for k=1:n                                  % plot results
  subplot(n,1,k)
  plot(1:N, sV(k,:), 'k-', 1:N, xV(k,:), 'r--')
end


%% Gives the non-linear function
function xx = nonLinearDynamics(x, u) 
    % Continuous time non-linear dynamics
    xx(1) =  (-x(4)*1*(-x(2)));
    xx(2) =  x(3);
    xx(3) =  u;
    xx(4) =  0; 
end

function yy = h(x)
    yy = x(1) ;
    
end