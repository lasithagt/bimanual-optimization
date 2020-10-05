
function dexterity_analysis

    x      = [4.7875  -12.3765    1.0806    1.3030    1.6627    2.9388    3.8017    4.9553];

    alpha  = [pi/2 -pi/2 -pi/2 0 pi/2 -pi/2 0];
    offset = [0,0,0,-pi/2,0,-pi/2,0];
    d      = [0*x(5) 0*x(8) x(7) 0 0 0];
    a      = [0 0 0 x(8) 0 0 0];
    tool   = 0;

            
    L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
    L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
    L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
    L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
    L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
    L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
    L7 = Link('d', tool, 'a',    0, 'alpha', alpha(7),'offset', offset(7));
    
    mini_chain = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_mini','base', eye(4));

end