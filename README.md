# ELE306-Boat-Project-2025
Autonomous plastic-collecting boat with manipulator - ELE306 HVL.

Utviklet av Bendik Bergquist Pedersen, Lars Folgerø Dale, Lars Helge Aamodt og Sebastian Alveberg

> ELE306 Robotics semester project – Robot design challenge 4:  
> *Small boat with manipulator for picking plastic from the sea*   

Denne mappen inneholder vår **fullstendige MATLAB-simulering** av den autonome båten med robotarm
for plastplukking, slik den er beskrevet i prosjektspesifikasjonen for ELE306 Robotikk.   

Simuleringen kobler sammen:

- Kinematikk og kontroll av **mobil base (båt med to thrustere)**
- Kinematikk og bevegelse for **robotarm** (plast-håv)
- **Navigasjon og kontroll** (f.eks. LOS/PID) for å følge bane og/eller søkemønster
- **Sensordata og estimat** (IMU, strøm, EKF – der det er aktivert i koden)
- Visualisering av båt- og armbevegelse over tid
