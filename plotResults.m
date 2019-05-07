function plotResults(phi, alpha, q_ref, x_d)
% PLOTRESULTS Summary of this function goes here
% Use Peter Corke's Toolbox
%   Detailed explanation goes here

d = phi([1 3 5]);
a = phi([2 4 6]);

L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1));        
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2));
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));

% Display the current manipulator.
manip_res = SerialLink([L1 L2 L3], 'name', 'optimized robot');

% Plot the after and previous
manip_res.plot(q_ref, 'workspace', [-2.5 2.5 -2.5 2.5 -2.5 2.5], 'noshadow','noarrow', 'view',[-130 50],'tile1color',[10 1 1], 'trail', '-','delay',0.1)
manip_res.vellipse(q, options)
end

