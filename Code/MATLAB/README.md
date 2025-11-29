# MATLAB-kode – ELE306 Båtprosjekt 2025

Denne mappen inneholder all MATLAB-kode som brukes i ELE306 Robotikk-semesterprosjektet:
**Autonom båt med robotarm for innsamling av plast**.

Koden er organisert i flere undermapper som hver representerer en del av systemet:
mobilbase, robotarm, navigasjon, plastsimulering og full-systemtester.

Under følger en oversikt over hva hver mappe inneholder.

---

## 📁 1. `TotalSimuleringMobilBaseOgRobotarm/`
**Full simulering av båt + robotarm**

Hovedmappen for komplett simulering av hele systemet.  
Inneholder:
- Båtens kinematikk og styring (PID/LOS)
- Armbevegelser for plukking og dropp
- Navigasjon/søk
- Live-plotting og visualisering  

Dette er mappen som brukes for å generere resultater og figurer til rapporten.

---

## 📁 2. `ArmKinematikk/`
**Kinematikk for robotarmen**

Inneholder:
- Fremoverkinematikk (FK)
- DH-parametere
- Test av armbevegelser og enkle trajektorier  

Brukes til å verifisere armens bevegelse før integrasjon med hovedsimulatoren.

---







# Her ligger koder for Robotarm, TotalSimuleringMobilBaseOgRobotarm og simulink.
1. ### MATLAB - TotalSimuleringMobilBaseOgRobotarm  
Dette er mappe for MATLAB simulering med navigasjon, lokalisering og robotarm. 
2. ARM
3. ### Simulink
Simulink er __IKKE__ brukt i sluttresultatet av prosjektet. I en tidlig fase av prosjektet forsøkte vi oss på simulink men det var noe utfordrende og vi kom ikke helt i mål med dette. Vi valgte likevell i legge den ved om det er intressant men prosjektet er fullverdig uten simulink.
Kjøring av simulink:
  1. kjør init_boat
  2. kjør boat_pp_diff


