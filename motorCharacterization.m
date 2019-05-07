close all;

%% Motor Parameters
Km    = 1;
Ka    = 1;
phi   = 1;
Kphi  = 1;
tau_m = 1;
tau_a = 1;

tf1_w_va = tf(Ka*Kphi*phi*Km,[tau_a*tau_m tau_m+tau_a 1+Ka*Km*(Kphi).^2]);
t = 0:0.1:pi;
u = sin(5*t);
lsim(tf1_w_va,u,t)