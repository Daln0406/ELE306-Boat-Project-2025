function [T_BA, T_BC] = frames_boat_arm_cam()
% Transformasjoner på båten:
%   - A = arm-base
%   - B = båt-base
%   - C = kamera

    %% ----------- ARM-BASE I BÅT-FRAME (A -> B) -----------
    % Arm er plassert 0.25 m FORAN båtens origo
    xA = 0.25;    % 25 cm fram
    yA = 0.00;    % midt
    zA = 0.00;    % dekkhøyde

    R_BA = eye(3);
    p_BA = [xA; yA; zA];

    T_BA = [R_BA, p_BA;
            0 0 0 1];

    %% ----------- KAMERA I BÅT-FRAME (C -> B) -----------
    % Kamera 45 cm foran båtens origo (altså 20 cm foran armen)
    xC = 0.45;
    yC = 0.00;
    zC = 0.10;      % 10 cm over dekk

    % Tilt kamera ned mot vannet
    alpha_deg = -15;
    alpha = deg2rad(alpha_deg);

    R_y = [ cos(alpha)  0  -sin(alpha);
            0           1   0;
            sin(alpha)  0   cos(alpha)];

    p_BC = [xC; yC; zC];

    T_BC = [R_y, p_BC;
            0 0 0 1];
end
