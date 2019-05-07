function SpringDamper

% Full information H-infinity control of a spring-damper system.

close all;

% Define the system matrices and the scaling parameters.
A = [0 0 1 0; 0 0 0 1; -1 1 -0.2 0.2; 0.5 -2.5 0.1 -0.15];
B2 = [0; 0; 1; 0];
alphau = 0.2; % magnitude requirement on u
B2prime = B2 * alphau;
B1 = [0; 0; 0; 0.5];
alphaw = 0.4; % magnitude limit on w
B1prime = B1 * alphaw;
D12 = [0; 0; 1];
D11 = [0; 0; 0];
alphax1 = 0.5; % magnitude requirement on x1
alphax2 = 0.5; % magnitude requirement on x2
C1 = [1/alphax1 0 0 0; 0 1/alphax2 0 0; 0 0 0 0];

% Compute the feedback matrix using Matlab's hinffi function.
Plant = ss(A, [B1prime B2prime], C1, [D11 D12]);
nu = 1;
gammamin = 1;
gammamax = 1;
tolerance = 0.001;
[K, N, gamma] = hinffi(Plant, nu, gammamin, gammamax, tolerance);
K = -K(:,1:4);
% Compute the feedback matrix using the Riccati equation.
R = B2prime*B2prime'-B1prime*B1prime'/gammamin^2;
R(1,1) = eps;
R(2,2) = eps;
X = care(A, eye(size(A)), C1'*C1, inv(R));
K = B2prime' * X;
% Compute the feedback matrix using the Hamiltonian.
H = [A, -R; -C1'*C1, -A'];
[v,d] = eig(H);
psi = [];
for i = 1 : 8
    if real(d(i,i)) < 0
        psi = [psi v(:,i)];
    end
end
psi11 = psi(1:4,:);
psi21 = psi(5:8,:);
X = psi21 * inv(psi11);
K = B2prime' * X;
% Compute the infinity norm of the normalized system.
Acl = A - B2prime * K;
Bcl = B1prime;
Ccl = C1 - D12 * K;
Dcl = [0; 0; 0];
sv = sigma(Acl, Bcl, Ccl, Dcl);
infnorm = max(max(sv));
% Here's another way to get the infinity norm.
SysCl = ss(Acl, Bcl, Ccl, Dcl);
infnorm = norm(SysCl, inf);
disp(['inf norm of normalized system = ', num2str(infnorm)]);
% Compute and plot the magnitude of the frequency responses
% of the unscaled system with feedback control.
K = alphau * K;
Acl = A - B2 * K;
Bcl = B1;
C = [1 0 0 0; 0 1 0 0; 0 0 0 0];
Ccl = C - D12 * K;
Dcl = 0;
SysCl = ss(Acl, Bcl, Ccl, Dcl);
omega = logspace(-1, 1);
[mag, pha] = bode(SysCl, omega);
magt1=mag(1,1,:); mag1=magt1(:);
magt2=mag(2,1,:); mag2=magt2(:);
magt3=mag(3,1,:); mag3=magt3(:);
figure;
loglog(omega,mag1,'r-', omega,mag2,'b--', omega,mag3,'k-.')
disp(['max freq response from w to d1 = ', num2str(max(mag1))]);
disp(['max freq response from w to d2 = ', num2str(max(mag2))]);
disp(['max freq response from w to u = ', num2str(max(mag3))]);
xlabel('Frequency (rad/sec)')
ylabel('Magnitude')
legend('d1/w', 'd2/w', 'u/w')