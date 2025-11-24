%% test_camToWorld_realPose.m
clear; clc; close all;

fprintf("=== TEST: Kamera to World med realistisk båtholdning ===\n");

% 1) Hent transformasjoner på båten
[T_BA, T_BC] = frames_boat_arm_cam();

% 2) Sett en REALISTISK båtPose i world (pool-frame)
% La oss si båten står på pos (0.3, 0.2) og peker 45 grader (pi/4 rad)
xB   = 0.30;       % 30 cm fra venstre i pool
yB   = 0.20;       % 20 cm opp i pool
psiB = pi/4;       % 45 graders heading

boatPose = [xB, yB, psiB];

% 3) Sett et punkt i kamera-frame
% F.eks. kamera oppdager plast 1 meter frem og 0.1 meter ned
% (zC tilpasses kamerahøyden)
pC = [1.0; 0.0; 0.0];

fprintf("pC (Camera-frame):   [%.2f, %.2f, %.2f]\n", pC);

% 4) Konverter til world
pW = camToWorld(pC, boatPose, T_BC);

fprintf("pW (World-frame):    [%.2f, %.2f, %.2f]\n\n", pW);

% 5) Plot world-scenen
figure('Color','w');
hold on; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Kamera til World transformasjon');

% Plot båten som en trekant
boatLen = 0.5;  % lengde frem
boatWid = 0.20; % halv bredde

R = [cos(psiB) -sin(psiB);
     sin(psiB)  cos(psiB)];

% båtkoordinater i lokal ramme
boat_local = [ boatLen  0;
               -boatLen  boatWid;
               -boatLen -boatWid ]';

boat_world = R * boat_local + [xB; yB];

patch(boat_world(1,:), boat_world(2,:), [0.2 0.4 0.7], ...
      'FaceAlpha',0.3,'EdgeColor','b','LineWidth',2);

% Kamera-posisjon i world
pC_world = ( [cos(psiB) -sin(psiB) 0;
              sin(psiB)  cos(psiB) 0;
              0          0         1] * T_BC(1:3,4) ) ...
            + [xB; yB; 0];

plot(pC_world(1), pC_world(2), 'ko','MarkerFaceColor','y','MarkerSize',8);
text(pC_world(1), pC_world(2), '  Camera');

% Plast-posisjon i world
plot(pW(1), pW(2), 'ro', 'MarkerSize',10, 'LineWidth',2);
text(pW(1), pW(2), '  Plastic');

axis equal
