function robot = boat_arm_model()
% boat_arm_model
% Returnerer SerialLink-modell av 5DOF-armen på båten.

    %% ---------- ROBOTDEFINISJON ----------
    [alpha_1, alpha_4] = deal(-pi/2, pi/2);
    [alpha_2, alpha_3, alpha_5] = deal(0);
    [a1, a2, a3, a4, a5] = deal(0, 0.35, 0.30, 0, 0);
    [d1, d2, d3, d4, d5] = deal(0.2, 0, 0, 0.2, 0.05);

    %      [theta d   a   alpha sigma]
    L1 = Link([0     d1  a1  alpha_1 0]);  % base-joint
    L2 = Link([0     d2  a2  alpha_2 0]);
    L3 = Link([0     d3  a3  alpha_3 0]);
    L4 = Link([0     d4  a4  alpha_4 0]);
    L5 = Link([0     d5  a5  alpha_5 0]);

    % Offsets slik du hadde
    L2.offset = -pi/2;
    L4.offset =  pi/2;

    % Bygg robot
    robot = SerialLink([L1 L2 L3 L4 L5], 'name', '5DOF');

    %% ---------- TOOL / HÅV ----------
    robot.tool = transl(0, 0, 0.20);  % håven/håndleddet stikker 20 cm ut
end
