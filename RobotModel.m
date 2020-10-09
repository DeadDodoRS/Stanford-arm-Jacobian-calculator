clc; clear;

% set angles as symbolical
syms q1 real
syms q2 real
syms q3 real

nq1 = 0;
nq2 = pi / 2;

% revolute joint
nq3 = 5;

%Link lengths
L1=10;
L2=20;

L=[L1 L2];

FK=simplify(Rz(q1)*Tz(L1)*Ry(-q2)*Tx(L2)*Tx(q3))

startCords = [0; 0; 0; 1];
a=simplify(FK * startCords);

%Just FK checking
a1 = subs(FK, [q1, q2], [nq1, nq2]);

%Inverse kinematic
x = a1(1,4);
y = a1(2,4);
z = a1(3,4);

ik_q11 = atan2(y, x);
ik_q21 = atan2(z - L(1), sqrt(x^2 + y^2));

ik_q12 = atan2(y, x) + pi;
ik_q22 = pi - atan2(z - L(1), sqrt(x^2 + y^2));

ik_q3 = sqrt((x^2 + y^2) + (z - L(1))^2) - L(2);

%Jacobian. Classical approach

O = [
    cos(q1)*(q3 + L(2)) * cos(q2);
    sin(q1)*(q3 + L(2)) * cos(q2);
    L(1) + sin(q2)*(q3 + L(2));
    cos(q1);
    sin(q1);
    q1];

J_classical = simplify([diff(O, q1), -diff(O, q2), diff(O, q3)])

% Jacobian. Scew teory
% Origin
T00= eye(4);
T01= Rz(q1)*Tz(L(1));
T02= Rz(q1)*Tz(L(1))*Ry(-q2)*Tx(L(2)+q3);

O0 = T00(1:3,4);
O1 = T01(1:3,4);
O2 = T02(1:3,4);

Z0 = T00(1:3,3); % 3rd coloumn corresponds to Rz
Z1 = T01(1:3,2); % 1st coloumn corresponds to Ry
Z2 = T02(1:3,1); % 3rd coloumn corresponds to Tx

J1 = [cross(Z0,(O2-O0));Z0];
J2 = [cross(Z1,(O2-O1));Z1];
J3 = [Z2;0; 0; 0];

J_scew = [simplify(J1), simplify(J2), simplify(J3)]

%Numerical approach

%Extract rotation matrix from FK
R = simplify(FK(1:3,1:3));

% diff by q1
q1_diff=simplify(Rzd(q1)*Tz(L1)*Ry(-q2)*Tx(L2)*Tx(q3)* [R^-1 zeros(3,1);0 0 0 1]);
J1= [q1_diff(1,4), q1_diff(2,4), q1_diff(3,4), q1_diff(3,2), q1_diff(1,3), q1_diff(2,1)]';

% diff by q2
q2_diff=simplify(Rz(q1)*Tz(L1)*Ryd(-q2)*Tx(L2)*Tx(q3)* [R^-1 zeros(3,1);0 0 0 1]);
J2= [q2_diff(1,4), q2_diff(2,4), q2_diff(3,4), q2_diff(3,2), q2_diff(1,3), q2_diff(2,1)]';

% diff by q3
q3_diff=simplify(Rz(q1)*Tz(L1)*Ry(-q2)*Tx(L2)*Txd(q3)* [R^-1 zeros(3,1);0 0 0 1]);
J3= [q3_diff(1,4), q3_diff(2,4), q3_diff(3,4), q3_diff(3,2), q3_diff(1,3), q3_diff(2,1)]';

J_numerical= [simplify(J1), simplify(J2), simplify(J3)]

%We can find angles by = 0
det_Jv_rot = simplify(det(J_numerical(1:3,1:3)))
det_Jv_trans = simplify(det(J_numerical(4:6,1:3)))


%Velocity
syms t
q1Changes = sin(t);
q2Changes = cos(2*t);
q3Changes = sin(3*t);

J_velocity = simplify(subs(J_numerical, {L(1), L(2), q1, q2, q3}, {L(1), L(2), q1Changes, q2Changes, q3Changes}));
velocity = diff([q1Changes q2Changes q3Changes]');

time = 0:0.1:5;
velocity_data = subs(J_velocity * velocity, {t}, {time});

plot(time, velocity_data)
title('Velocity of the tool frame')
legend('Tx', 'Ty', 'Tz','Rx', 'Ry', 'Rz')
