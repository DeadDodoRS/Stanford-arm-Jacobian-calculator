function draw_robot(q,L)

figure;
hold on
view(200,25)
grid on
axis equal
T1=Rz(q(1))*Tz(L(1));
plot3(0,0,0,'ro','MarkerSize',10,'LineWidth', 10);
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-b','LineWidth', 5);
T2=Rz(q(1))*Tz(L(1))*Tx(-L(2));
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-b','LineWidth', 5);
plot3(T2(1,4),T2(2,4),T2(3,4),'ro','MarkerSize',10,'LineWidth', 10);
T3=Rz(q(1))*Tz(L(1))*Tx(-L(2))*Rx(q(2))*Tz(L(3));
plot3([T2(1,4) T3(1,4)],[T2(2,4) T3(2,4)],[T2(3,4) T3(3,4)],'-b','LineWidth', 5);
T4=Rz(q(1))*Tz(L(1))*Tx(-L(2))*Rx(q(2))*Tz(L(3))*Tz(q(3));
plot3([T3(1,4) T4(1,4)],[T3(2,4) T4(2,4)],[T3(3,4) T4(3,4)],'-r','LineWidth', 5);

T4x=Rz(q(1))*Tz(L(1))*Tx(-L(2))*Rx(q(2))*Tz(L(3))*Tz(q(3))*Rz(q(4))*Rx(q(5))*Rz(q(6))*Tx(5);
quiver3(T4(1,4),T4(2,4),T4(3,4),T4x(1,4)-T4(1,4),T4x(2,4)-T4(2,4),T4x(3,4)-T4(3,4),0,'-p')
T4y=Rz(q(1))*Tz(L(1))*Tx(-L(2))*Rx(q(2))*Tz(L(3))*Tz(q(3))*Rz(q(4))*Rx(q(5))*Rz(q(6))*Ty(5);
quiver3(T4(1,4),T4(2,4),T4(3,4),T4y(1,4)-T4(1,4),T4y(2,4)-T4(2,4),T4y(3,4)-T4(3,4),0,'-c')
T4z=Rz(q(1))*Tz(L(1))*Tx(-L(2))*Rx(q(2))*Tz(L(3))*Tz(q(3))*Rz(q(4))*Rx(q(5))*Rz(q(6))*Tz(5);
quiver3(T4(1,4),T4(2,4),T4(3,4),T4z(1,4)-T4(1,4),T4z(2,4)-T4(2,4),T4z(3,4)-T4(3,4),0,'-g')


