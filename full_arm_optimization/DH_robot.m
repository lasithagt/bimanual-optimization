%% defines the robot using Dh parameters
function [robots] = DH_robot(DH_par, name, base)    
   
    robots = cell(1, 2);
    d      = DH_par();
    a      = DH_par();
    offset = DH_par();
    alpha  = DH_par();
    
    L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
    L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
    L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
    L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
    L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
    L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
    L7 = Link('d', d(7), 'a', a(7), 'alpha', alpha(7),'offset', offset_R(7));
    L8 = Link('d', 0, 'a', 0, 'alpha', 0,'offset', 0);

    robots{1} = SerialLink([L1 L2 L3 L4 L5 L6 L7 L8], 'name', name{1},'base', base{1});
    robots{2} = SerialLink([L1 L2 L3 L4 L5 L6 L7 L8], 'name', name{2},'base', base{2});

end
