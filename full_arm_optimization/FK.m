function fwk = FK(lq)
    q1 = lq(1); q2 = lq(2); q3 = lq(3); q4 = lq(4); l1=lq(5); l2=lq(6); l3=lq(7); l4=lq(8);
    

    fwk = [(-(-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + (-sin(q1)*cos(q2) - sin(q2)*cos(q1))*cos(q3))*sin(q4) + ((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) + (-sin(q1)*cos(q2) - sin(q2)*cos(q1))*sin(q3))*cos(q4), (-(-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + (-sin(q1)*cos(q2) - sin(q2)*cos(q1))*cos(q3))*cos(q4) - ((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) + (-sin(q1)*cos(q2) - sin(q2)*cos(q1))*sin(q3))*sin(q4), 0, l1*cos(q1) - l2*sin(q1)*sin(q2) + l2*cos(q1)*cos(q2) + l3*(-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) + l3*(-sin(q1)*cos(q2) - sin(q2)*cos(q1))*sin(q3) + l4*(-(-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + (-sin(q1)*cos(q2) - sin(q2)*cos(q1))*cos(q3))*sin(q4) + l4*((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) + (-sin(q1)*cos(q2) - sin(q2)*cos(q1))*sin(q3))*cos(q4);...
        ((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + (sin(q1)*cos(q2) + sin(q2)*cos(q1))*cos(q3))*cos(q4) + ((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) - (sin(q1)*cos(q2) + sin(q2)*cos(q1))*sin(q3))*sin(q4), -((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + (sin(q1)*cos(q2) + sin(q2)*cos(q1))*cos(q3))*sin(q4) + ((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) - (sin(q1)*cos(q2) + sin(q2)*cos(q1))*sin(q3))*cos(q4), 0, l1*sin(q1) + l2*sin(q1)*cos(q2) + l2*sin(q2)*cos(q1) + l3*(-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + l3*(sin(q1)*cos(q2) + sin(q2)*cos(q1))*cos(q3) + l4*((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*sin(q3) + (sin(q1)*cos(q2) + sin(q2)*cos(q1))*cos(q3))*cos(q4) + l4*((-sin(q1)*sin(q2) + cos(q1)*cos(q2))*cos(q3) - (sin(q1)*cos(q2) + sin(q2)*cos(q1))*sin(q3))*sin(q4);...
        0, 0, 1, 0; 0, 0, 0, 1];

    
end
