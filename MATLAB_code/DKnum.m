function p_e = DKnum(q1,q2,q3,q4,q5,q6,q7)

    p_e = [0.316*cos(q1)*sin(q2) - 0.0825*sin(q1)*sin(q3) - 0.107*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - 1.0*cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + 0.088*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - 1.0*cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + 0.0825*cos(q4)*(sin(q1)*sin(q3) - 1.0*cos(q1)*cos(q2)*cos(q3)) - 0.088*cos(q6)*(sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - 1.0*cos(q1)*cos(q2)*cos(q3)) - 1.0*cos(q1)*sin(q2)*sin(q4))) + 0.384*sin(q4)*(sin(q1)*sin(q3) - 1.0*cos(q1)*cos(q2)*cos(q3)) - 0.107*sin(q6)*(sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) + cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - 1.0*cos(q1)*cos(q2)*cos(q3)) - 1.0*cos(q1)*sin(q2)*sin(q4))) + 0.0825*cos(q1)*cos(q2)*cos(q3) + 0.384*cos(q1)*cos(q4)*sin(q2) - 0.0825*cos(q1)*sin(q2)*sin(q4);
                0.0825*cos(q1)*sin(q3) + 0.316*sin(q1)*sin(q2) + 0.107*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 1.0*cos(q4)*sin(q1)*sin(q2)) - 0.088*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 1.0*cos(q4)*sin(q1)*sin(q2)) - 0.0825*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - 0.384*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + 0.088*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - 1.0*cos(q2)*sin(q1)*sin(q3))) + 0.107*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - 1.0*cos(q2)*sin(q1)*sin(q3))) - 0.0825*sin(q1)*sin(q2)*sin(q4) + 0.0825*cos(q2)*cos(q3)*sin(q1) + 0.384*cos(q4)*sin(q1)*sin(q2);
                                                                                                                                                                                                                                                                                                                                         0.316*cos(q2) + 0.384*cos(q2)*cos(q4) - 0.0825*cos(q3)*sin(q2) - 0.0825*cos(q2)*sin(q4) + 0.088*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - 1.0*cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + 0.107*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - 1.0*cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - 0.107*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)) + 0.088*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)) + 0.0825*cos(q3)*cos(q4)*sin(q2) + 0.384*cos(q3)*sin(q2)*sin(q4) + 0.333];
end

