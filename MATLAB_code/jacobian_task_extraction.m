clear
clc
syms q1 q2 q3 q4 q5 q6 q7 q8 real;
q = [q1; q2; q3; q4; q5; q6; q7; 0];
n=8;
d = [0.333 0 0.316 0 0.384 0 0 0.107]';
a = [0 0 0 0.0825 -0.0825 0 0.088 0]';
alfa = [0 -90 90 90 -90 90 90 0 ]';

A_dh(1:n) = {zeros(4)};
    for i = 1:n
        A_dh{i} = [[cos(q(i, 1)) -sin(q(i, 1)) 0 a(i)];
                  [cosd(alfa(i))*sin(q(i, 1)) cosd(alfa(i))*cos(q(i, 1)) -sind(alfa(i)) -d(i)*sind(alfa(i))];
                  [sind(alfa(i))*sin(q(i, 1)) sind(alfa(i))*cos(q(i, 1)) cosd(alfa(i)) cosd(alfa(i))*d(i)];
                  [0 0 0 1]];
    end

A_tot = A_dh{1};

for i = 2:n
    A_tot = A_tot*A_dh{i};
    if i==4
        A_0_4 = A_tot;
    end
end

simplify(A_tot)
% simplify(A_0_4)
R = simplify(A_tot(1:3,1:3));

p_e = [A_tot(1,4); A_tot(2,4); A_tot(3,4)];
p_4 = [A_0_4(1,4); A_0_4(2,4); A_0_4(3,4)]
z_4 = vpa(p_4(3))

phi = atan2(R(2,1),R(1,1));
theta = atan2(-R(3,1),sqrt((R(3,2)^2+R(3,3)^2)));
psy = atan2(R(3,2),R(3,3));
r = vpa([ ...
    p_e; ...
    phi; ...
    theta; ...
    psy; ...
    z_4]);
% r = vpa([p_e;
%      atan2((abs(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) + cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))^2 + abs(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))^2)^(1/2)/(abs(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) + cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))^2 + abs(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))^2 + abs(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))^2)^(1/2), (sin(conj(q6))*(cos(conj(q5))*(cos(conj(q2))*sin(conj(q4)) - cos(conj(q3))*cos(conj(q4))*sin(conj(q2))) + sin(conj(q2))*sin(conj(q3))*sin(conj(q5))) - cos(conj(q6))*(cos(conj(q2))*cos(conj(q4)) + cos(conj(q3))*sin(conj(q2))*sin(conj(q4))))/(abs(sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) + cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))^2 + abs(cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))^2 + abs(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))^2)^(1/2));
%      z_4])
% 
phi
theta
psy

Analytic_Jacobian = simplify(jacobian(r,[q1,q2,q3,q4,q5,q6,q7]))


