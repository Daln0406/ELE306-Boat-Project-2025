# MATLAB-kode – ELE306 Båtprosjekt 2025

Denne mappen inneholder all MATLAB-kode som brukes i ELE306 Robotikk-semesterprosjektet:
**Small boat with manipulator for picking plastic from the sea**.

Koden er organisert i flere undermapper.

Under følger en oversikt over hva hver mappe inneholder.

---

## 1. [`TotalSimuleringMobilBaseOgRobotarm/`](https://github.com/Daln0406/ELE306-Boat-Project-2025/tree/main/Code/MATLAB/TotalSimuleringMobilBaseOgRobotarm)
**Full simulering av båt + robotarm**

Hovedmappen for komplett simulering av hele systemet.  
Inneholder:
- Båtens kinematikk og styring (PID/LOS)
- Armbevegelser for plukking og dropp
- Navigasjon/søk
- Live-plotting og visualisering  

Dette er mappen som har blitt brukt for å generere resultater og figurer til rapporten.

For å kjøre simulering:

---

## 2. [`Robotarm/`](https://github.com/Daln0406/ELE306-Boat-Project-2025/tree/main/Code/MATLAB/Robotarm)
**Kinematikk for robotarmen**

Inneholder:
- Differensiell kinematikk
- Fremoverkinematikk
- Invers kinematikk
- Motion planning example

Brukes til å verifisere armens bevegelse.

For å kjøre simulering:
- Copy-paste inn i MATLAB

---

## 3. [`Simulink/`](https://github.com/Daln0406/ELE306-Boat-Project-2025/tree/main/Code/MATLAB/Simulink)
Simulink er __IKKE__ brukt i sluttresultatet av prosjektet. I en tidlig fase av prosjektet forsøkte vi oss på simulink men det var noe utfordrende og vi kom ikke helt i mål med dette. Vi valgte likevell i legge den ved om det er intressant men prosjektet er fullverdig uten simulink.
Kjøring av simulink:
  1. kjør init_boat
  2. kjør boat_pp_diff


