clear; clc; close all;

% DH-parametre
syms theta_1 theta_2 theta_3 theta_4 theta_5

[alpha_1, alpha_4] = deal(-sym(pi)/2, sym(pi)/2);
[alpha_2, alpha_3, alpha_5] = deal(0);
[a1, a2, a3, a4, a5] = deal(0, 0.35, 0.30, 0, 0);
[d1, d2, d3, d4, d5] = deal(0.2, 0, 0, 0.2, 0.05);

%      [theta d a alpha sigma]
L1 = Link([0 d1 a1 alpha_1 0]);
L2 = Link([0 d2 a2 alpha_2 0]);
L3 = Link([0 d3 a3 alpha_3 0]);
L4 = Link([0 d4 a4 alpha_4 0]);
L5 = Link([0 d5 a5 alpha_5 0]);
L2.offset = -pi/2;
L4.offset = pi/2;

% Sett sammen leddene til et SerialLink-objekt
% Kun fire første joints
robot4DOF = SerialLink([L1 L2 L3 L4]);

% Develop the transformation mapping End-effector to base (first 4 joints only) 
T01 = [cos(theta_1) -sin(theta_1)*cos(alpha_1) sin(theta_1)*sin(alpha_1) a1*cos(theta_1);
    sin(theta_1) cos(theta_1)*cos(alpha_1) -cos(theta_1)*sin(alpha_1) a1*sin(theta_1);
    0 sin(alpha_1) cos(alpha_1) d1;
    0 0 0 1];

th2 = theta_2 - pi/2; % ta med samme offset som i L2
T12 = [cos(th2) -sin(th2)*cos(alpha_2) sin(th2)*sin(alpha_2) a2*cos(th2);
    sin(th2) cos(th2)*cos(alpha_2) -cos(th2)*sin(alpha_2) a2*sin(th2);
    0 sin(alpha_2) cos(alpha_2) d2;
    0 0 0 1];

T23 = [cos(theta_3) -sin(theta_3)*cos(alpha_3) sin(theta_3)*sin(alpha_3) a3*cos(theta_3);
    sin(theta_3) cos(theta_3)*cos(alpha_3) -cos(theta_3)*sin(alpha_3) a3*sin(theta_3);
    0 sin(alpha_3) cos(alpha_3) d3;
    0 0 0 1];

th4 = theta_4 + pi/2;  % ta med samme offset som i L4
T34 = [cos(th4) -sin(th4)*cos(alpha_4) sin(th4)*sin(alpha_4) a4*cos(th4);
    sin(th4) cos(th4)*cos(alpha_4) -cos(th4)*sin(alpha_4) a4*sin(th4);
    0 sin(alpha_4) cos(alpha_4) d4;
    0 0 0 1];

% Symbolsk utledning
OT4 = T01*T12*T23*T34;

% Forenkle symbolsk
OT4 = simplify(OT4, 'Steps', 300)

% Test-konfigurasjon
q = [0.2, -0.6, 0.9, -0.3];  % Eksempel i radianer

% Legger vinklene inn i transformasjonsmatrisen
T_num = double(subs(OT4, ...
    {theta_1, theta_2, theta_3, theta_4}, num2cell(q)));

% Forward kinematics
T_fk = robot4DOF.fkine(q).T;

fprintf('\nForskjell (OT4 - fkine):\n');
vpa(T_num - T_fk, 6)

% Posisjonsvektor
disp('Posisjonsligning p(q)');
p = OT4(1:3,4)

% Beregning av Jacobian
q = [theta_1 theta_2 theta_3 theta_4];
disp('Jacobian Jv(q)');
Jv = simplify(jacobian(p, q)) % Jacobian for lineær hastighet

% Symbolsk uttrykk for leddhastighet
syms qdot1 qdot2 qdot3 qdot4 real
qdot = [qdot1; qdot2; qdot3; qdot4];

% Uttrykk for kartesisk hastighet
disp('Hastighetsligning pdot = Jv * qdot');
pdot = simplify(Jv * qdot)

% Setter inn faktiske tallverdier settes inn for theta og qdot
subs_vals = {theta_1, theta_2, theta_3, theta_4, qdot1, qdot2, qdot3, qdot4};
num_vals = {0.2, -0.6, 0.9, -0.3, 0.1, -0.2, 0.15, 0.05};

% Numerisk Jacobian
Jv_num = double(subs(Jv, subs_vals(1:4), num_vals(1:4)));

% Numerisk kartestisk hastighet
pdot_num = double(subs(pdot, subs_vals, num_vals));

disp('Numerisk Jacobian:');
disp(Jv_num);
disp('Numerisk hastighet pdot:');
disp(pdot_num);