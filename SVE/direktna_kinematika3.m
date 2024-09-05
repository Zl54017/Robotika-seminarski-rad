theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);
theta3 = deg2rad(theta3_deg);

% rotacijska matrica po z osi zgloba 1
T01r=[cos(theta1) -sin(theta1) 0 0;
      sin(theta1)  cos(theta1) 0 0;
      0            0           1 0;
      0            0           0  1;];


T12r=[cos(theta2) -sin(theta2) 0 0;
      sin(theta2)  cos(theta2) 0 0;
      0            0           1 0;
      0            0           0  1;];


T23r=[cos(theta3) -sin(theta3) 0 0;
      sin(theta3)  cos(theta3) 0 0;
      0            0           1 0;
      0            0           0  1;];

T01=T01t*T01r;
T12=T12t*T12r;
T23=T23t*T23r;

T0ee=T01*T12*T23*T3ee;

posx=T0ee(1,4);
posy=T0ee(2,4);
posz=T0ee(3,4);

% disp(theta1_deg)

%disp(T0ee)

