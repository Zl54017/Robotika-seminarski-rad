% pozicija zglobova (0,0,0) odgovara potpuno uspravnom polozaju
% u simulinku trenutno: (188, 78, -7)

% u theta unesi vrijednosti zakrenutosti zgloba od pocetnog
theta1_deg=0;%zakret zgloba 1 oko vlastite z osi (pravilo desne ruke)
theta2_deg=0;%zakret zgloba 2 oko vlastite z osi
theta3_deg=0;% zakret zgloba 3 oko vlastite z osi

targetPos = [-25,85,403.5];

% deg -> rad
theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);
theta3 = deg2rad(theta3_deg);

% pravilo desne ruke -> +z os zgloba je u smjeru njegove vrtnje
% pozicija zgloba 1 s obzirom na ishodiste
x1=0;
y1=85;
z1=39;

% zglob 2 u odnosu na 1, gledano iz kooridnatnog sustava zgloba 1
x2=19;
y2=0;
z2=41.5;

% zglob 3 u odnosu na 2, gledano iz kooridnatnog sustava zgloba 2
x3=0;
y3=170;
z3=0;

% end effector u odnosu na 3, gledano iz 3
xe=0;
ye=-153;
ze=44;

%translacijska matrica zgloba 1
T01t=[1 0 0 x1;
      0 1 0 y1;
      0 0 1 z1;
      0 0 0  1];


T12t=[0 0 1 x2;
      1 0 0 y2;
      0 1 0 z2;
      0 0 0  1];


T23t=[1 0 0 x3;
      0 -1 0 y3;
      0 0 -1 z3;
      0 0 0  1];


T3ee=[1 0 0 xe;
      0 1 0 ye;
      0 0 1 ze;
      0 0 0  1];



%inverzna param
robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 4);

% prvi zglob
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
setFixedTransform(jnt1, T01t);
jnt1.JointAxis = [0 0 1]; % Rotacija oko z osi
jnt1.PositionLimits = deg2rad([-150,150]); % Ograničenja zgloba 1 u radijanima
body1.Joint = jnt1;
addBody(robot, body1, 'base');


% Drugi zglob
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
setFixedTransform(jnt2, T12t);
jnt2.JointAxis = [0 0 1]; % Rotacija oko z osi
jnt2.PositionLimits = deg2rad([-90, 0]); % Ograničenja zgloba 2 u radijanima
body2.Joint = jnt2;
addBody(robot, body2, 'body1');


% Treći zglob
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
setFixedTransform(jnt3, T23t);
jnt3.JointAxis = [0 0 1]; % Rotacija oko z osi
jnt3.PositionLimits = deg2rad([0 150]); % Ograničenja zgloba 3 u radijanima
body3.Joint = jnt3;
addBody(robot, body3, 'body2');


% End effector
endEffector = rigidBody('endeffector');
setFixedTransform(endEffector.Joint, T3ee);
addBody(robot, endEffector, 'body3');

% IK
ik = inverseKinematics('RigidBodyTree',robot);

initialguess = robot.homeConfiguration;
weights = [0 0 0 1 1 1]; % zanemari orijentaciju endef
