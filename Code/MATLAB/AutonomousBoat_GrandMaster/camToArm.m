function pA = camToArm(pC, boatPose, T_BC, T_BA)
% camToArm
% Konverterer et punkt i kamera-frame (C) til arm-base-frame (A).
%
% INPUT:
%   pC       = [xC; yC; zC]  (plast-posisjon i kamera-koordinater)
%   boatPose = [xB, yB, psiB]  (båtens posisjon/orientering i world)
%              (N.B.: Vi trenger psiB for rotasjon B, ellers kan denne
%                     forenkles hvis A og C er definert i B-frame uten yaw)
%   T_BC     = transformasjon C -> B
%   T_BA     = transformasjon A -> B
%
% OUTPUT:
%   pA       = [xA; yA; zA]  (plast-posisjon i arm-base-koordinater)

    % === 1. C -> B ===
    pC_h = [pC; 1];
    pB_h = T_BC * pC_h;   % punkt i B-frame

    % === 2. B -> A ===
    % Vi trenger T_AB = (T_BA)^(-1)
    R_BA = T_BA(1:3, 1:3);
    p_BA = T_BA(1:3, 4);

    R_AB = R_BA.';              % inverse av rotasjon
    p_AB = -R_AB * p_BA;        % inverse av translasjon

    T_AB = [R_AB, p_AB;
            0 0 0 1];

    pA_h = T_AB * pB_h;
    pA   = pA_h(1:3);
end
