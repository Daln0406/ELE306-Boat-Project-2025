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
- Kjør script test_boat_master_test.m
- Viktig: For simuleringen i MATLAB har vi lagt til at plast dukker opp etter en viss tid t. Plasten dukker opp på er hardkodet posisjon (feks 1m fremfor båt). Dette kan skape trøbbel om båten er på en uheldig posisjon (plast kan dukke opp utenfor map). Om dette skjer kan "pc" variabel modifiseres litt, eventuelt t kan endres. Vi har også hatt litt trøbbel avhengig av hvilken MATLAB versjon som kjøres (24a vs 25b). Koden er sist testet og optimalisert for MATLAB24a og vi anbefaler å bruke denne versjonen.
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

## 3. [`Lattice Script/`](https://github.com/Daln0406/ELE306-Boat-Project-2025/tree/main/Code/MATLAB/Lattice-Script)
Enkle lattice program som er brukt i startfasen og implementert for ruteplanleggingen senere ved fullverdig simulering. 

