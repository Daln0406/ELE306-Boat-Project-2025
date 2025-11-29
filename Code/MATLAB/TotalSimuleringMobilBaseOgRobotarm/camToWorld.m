function pW = camToWorld(pC, boatPose, T_BC)
% camToWorld
% Konverterer et punkt målt i kamera-frame (C) til world-frame (W).
%
% INPUT:
%   pC       = [xC; yC; zC]  (plast-posisjon i kamera-koordinater)
%   boatPose = [xB, yB, psiB]  (båtens posisjon/orientering i world)
%   T_BC     = transformasjon C -> B (fra frames_boat_arm_cam)
%
% OUTPUT:
%   pW       = [xW; yW; zW]  (plast-posisjon i world-koordinater)

    xB   = boatPose(1);
    yB   = boatPose(2);
    psiB = boatPose(3);  % yaw

    % Rotasjonsmatrise fra B til W (z-akse opp)
    R_WB = [ cos(psiB), -sin(psiB), 0;
             sin(psiB),  cos(psiB), 0;
             0,          0,         1 ];

    p_WB = [xB; yB; 0];  % vi antar z=0 i world (vannflate)

    T_WB = [R_WB, p_WB;
            0 0 0 1];

    % Gjør pC om til homogen vektor
    pC_h = [pC; 1];

    % Først C -> B, så B -> W
    pW_h = T_WB * T_BC * pC_h;

    % Plukk ut xyz
    pW = pW_h(1:3);
end
