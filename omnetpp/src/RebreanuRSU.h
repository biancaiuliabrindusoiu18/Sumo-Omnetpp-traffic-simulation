/*
 * RebreanuRSU.h
 *
 * Road Side Unit fixat la fiecare intersectie cu semafor.
 *
 * Monitorizeaza instantaneu (vector + scalar):
 *   - queueLength          : vehicule oprite in raza (speed < 0.1 m/s)
 *   - totalVehicles        : toate vehiculele in raza
 *   - avgSpeedAtRSU        : viteza medie a vehiculelor din raza
 *   - mainGreenDuration    : durata fazei verzi principale (DynamicControl)
 *
 * Monitorizeaza cumulativ (scalar final):
 *   - vehiclesThroughput        : cate vehicule distincte au trecut prin raza
 *   - numberOfStoppedVehicles   : cate vehicule distincte s-au oprit in raza
 *
 * Control semafor via TraCI:
 *   - dynamicControlEnabled     : activeaza controlul adaptiv
 *   - isStable                  : daca true, RSU nu modifica semaforul
 *
 * Control adaptiv LOCAL (fara comunicare RSU-RSU), tip SCATS/SCOOT-lite.
 * Fiecare RSU = senzor (coada din BSM) + actuator (setProgramDefinition pe
 * propriul semafor). Ancora fixa = RSU cu isStable=true (masoara, nu actioneaza).
 *
 * Coerenta undei verzi: offset + ciclu (120s) raman FIXE in net.xml.
 * Se modifica DOAR durata verde (main +X / secondary -X, suma constanta).
 *
 * Detectie coada: window-based pe fereastra de ROSU, vehicule DISTINCTE oprite,
 * clasificate bulevard vs transversala prin HEADING. Snapshot la GREEN-ON
 * (citit din starea reala a semaforului, nu timer intern).
 */

#ifndef REBREANURSU_H_
#define REBREANURSU_H_

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <map>
#include <set>
#include <deque>
#include <vector>
#include <string>

class RebreanuRSU : public veins::DemoBaseApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;

protected:
    void onBSM(veins::DemoSafetyMessage* bsm) override;
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void handleSelfMsg(cMessage* msg) override;

private:

    //  Vehicule active in raza (pentru statistici instantanee + cleanup)
    std::map<int, simtime_t>    activeVehicles;
    std::map<int, double>       vehicleSpeeds;
    std::set<int>               seenVehicleIds;
    std::set<int>               stoppedVehicleIds;
    double vehicleTimeout = 10.0;


    //  FAZA 1 - Detectie coada pe fereastra de rosu (window-based)

    std::set<int> blvdRedSet;   // oprite cand BULEVARDUL e rosu  -> queue_blvd
    std::set<int> secRedSet;    // oprite cand TRANSVERSALA e rosu -> queue_secondary

    // Istoric pe cicluri
    std::deque<int> queueBlvdHistory;
    std::deque<int> queueSecHistory;

    // Directia bulevardului
    veins::Coord blvdDir;
    bool         hasBlvdDir = false;
    veins::Coord myPos;)


    //  Detectie GREEN-ON din starea semaforului

    int  prevPhaseIndex   = -1;
    int  cachedPhaseIndex = -1;
    bool tlInitialized    = false;


    //  FAZA 2 - Decizie locala

    int  greenOnCount = 0;       // cate green-on de bulevard de la ultimul interval
    int  currentStage = 0;       // 0 = baseline, max = maxStage
    int  downCounter  = 0;       // intervale consecutive cu cerere mica

    // Praguri / parametri
    int    highThreshold;
    int    lowThreshold;
    int    intervalCycles;
    int    sampleCycles;
    int    maxStage;
    int    stageStep;
    int    downIntervalsNeeded;
    double queueRange;
    double stopSpeedThreshold;

    bool debugHeading;
    int   dbgCount = 0;


    //  FAZA 3 - Aplicare

    double baselineMainGreen = -1;
    double baselineSecGreen  = -1;
    int    pendingStage      = -1;
    int    adjustmentsApplied = 0;

    cMessage* cleanupTimer = nullptr;
    cMessage* applyTimer   = nullptr;


    //  Parametri TraCI / rol
    std::string trafficLightId;
    int  mainGreenPhase;
    int  secondaryGreenPhase;
    bool isStable;
    bool dynamicControlEnabled;

    const int minGreen = 15;
    const int maxGreen = 75;

    //coada din sumo
    bool useTraciQueue;
    std::vector<std::string> blvdLanes;
    bool blvdLanesReady = false;
    int  getRealBoulevardQueue();
    std::vector<std::string> secLanes;
    bool secLanesReady = false;
    int  getRealSecondaryQueue();


    //  Statistici

    cOutVector queueLengthVector;
    cOutVector totalVehiclesVector;
    cOutVector avgSpeedVector;
    simsignal_t queueLengthSignal;
    simsignal_t totalVehiclesSignal;
    simsignal_t avgSpeedAtRSUSignal;

    cOutVector queueBlvdVector;
    cOutVector queueSecVector;
    cOutVector mainGreenDurationVector;
    cOutVector secGreenDurationVector;
    cOutVector currentStageVector;
    simsignal_t queueBlvdSignal;
    simsignal_t queueSecSignal;
    simsignal_t mainGreenDurationSignal;
    simsignal_t secGreenDurationSignal;
    simsignal_t currentStageSignal;
    cOutVector  queueBlvdRealVector;
    simsignal_t queueBlvdRealSignal;
    cOutVector  queueSecBsmVector;
    simsignal_t queueSecBsmSignal;

    simsignal_t vehiclesThroughputSignal;
    simsignal_t numberOfStoppedVehiclesSignal;
    simsignal_t adjustmentsAppliedSignal;


    //  TraCI lazy init
    veins::TraCICommandInterface* traci = nullptr;
    veins::TraCICommandInterface* getTraCI();


    //  Metode private
    void computeBoulevardDirection();
    void updateStats();
    void cleanupInactiveVehicles();
    void pollTrafficLight();
    void runIntervalDecision();
    void applyStage(int stage);
    bool isBoulevardHeading(const veins::Coord& headingVec) const;
    double meanLastN(const std::deque<int>& dq, int n) const;
    void detectBoulevardLanes();
    void detectSecondaryLanes();
    void detectLanesForPhase(int phase, std::vector<std::string>& outLanes,
                             bool& ready, const char* label);
};

#endif /* REBREANURSU_H_ */
