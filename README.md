# Sumo-Omnetpp-traffic-simulation
Control adaptiv distribuit al semafoarelor pe Bd. Liviu Rebreanu (Timișoara)

Platformă de co-simulare pentru evaluarea a trei strategii de control al undei verzi
(NoSync, GreenWave, DynamicControl) pe 11 intersecții semaforizate de pe
Bd. Liviu Rebreanu, folosind SUMO (trafic microscopic) cuplat cu OMNeT++ / Veins
(comunicație V2I 802.11p) prin TraCI.

Adresa repositpry-ului:
https://github.com/biancaiuliabrindusoiu18/Sumo-Omnetpp-traffic-simulation

Cod sursă complet (fără binare compilate, fără fișiere de rezultate)
Fișierele de rezultate (.sca/.vec, ~30 GB) nu sunt incluse în repository.

# Harta SUMO
Instalare SUMO 1.8
https://sourceforge.net/projects/sumo/files/sumo/version%201.8.0/

Descarcare harta
https://extract.bbbike.org/
in format OSM xml 7z

Instalare JOSM pentru edit harta
https://josm.openstreetmap.de/

Restul edit urilor din pachetul sumo cu netedit, rulare in SUMO-GUI

Pentru simulare in sumo direct, fisierul .sumocfg


# Proiectul OMNeT++
Instalare OMNeT++ 5.6.2
https://omnetpp.org/download/old.html

Instalare Veins 5.2
https://veins.car2x.org/download/

Se dezarhiveaza OMNeT++. 
De deschide terminalul livrat cu OMNeT++, mingenv.md. Toate comenzile de build se pot rula in acest shell
La prima deschidere ./configure
IDE-ul se lanseaza cu omnetpp din acelasi shell.

Se dezarhciveaza Veins in workspace-ul din OMNeT++. Se importa Veins in IDE-ul OMNeT++ (file-import-general-existing peojects into workspace-select veins). ACum veins e disponibil in workspace.

Pentru a fi disponibil in proiectul proproiu: project-properties-project refernces, bifeaza veins. 

Pentru compilare project-build all

# Pentru rularea unei simulari
Rulare presupune 2 procese

1. pornire legatura sumo-omnetpp

Intr-un terminal mingenv, si il las sa ruleze pe tot parcurul simularii
/d/omn/veins-5.2/veins-veins-5.2/bin/veins_launchd -vv -c "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui"

practic scriptul asculta pe portul 9999 si porneste sumo-gui automat cand omnetpp cere conexiune

2. lansare simularea

Din IDE, click dreapta pe simulations/omentpp.ini, run as - OMNET++ Simulation, sau configure simulation de unde pot alege: qtenv sau cmdenv.
Se alege scenariul dorit si se da start. Cand pornesc simularea din oment, va rula simulatn si sumo


Rezultatele vor fi disponile la terminarea rularii, in fisierul simulations/results/

De acolo in continuarea au fost prelucrate cu un script de python









