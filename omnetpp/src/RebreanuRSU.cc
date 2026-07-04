#include "RebreanuRSU.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include <cmath>
#include <algorithm>

using namespace veins;

Define_Module(RebreanuRSU);


//  initialize

void RebreanuRSU::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage != 0) return;

    //  Semnale instantanee
    queueLengthSignal   = registerSignal("queueLength");
    totalVehiclesSignal = registerSignal("totalVehicles");
    avgSpeedAtRSUSignal = registerSignal("avgSpeedAtRSU");
    queueLengthVector.setName("queueLength");
    totalVehiclesVector.setName("totalVehicles");
    avgSpeedVector.setName("avgSpeedAtRSU");

    //  Semnale
    queueBlvdSignal         = registerSignal("queueBlvd");
    queueSecSignal          = registerSignal("queueSecondary");
    mainGreenDurationSignal = registerSignal("mainGreenDuration");
    secGreenDurationSignal  = registerSignal("secondaryGreenDuration");
    currentStageSignal      = registerSignal("currentStage");
    queueBlvdVector.setName("queueBlvd");
    queueSecVector.setName("queueSecondary");
    mainGreenDurationVector.setName("mainGreenDuration");
    secGreenDurationVector.setName("secondaryGreenDuration");
    currentStageVector.setName("currentStage");

    //  Cumulative
    vehiclesThroughputSignal      = registerSignal("vehiclesThroughput");
    numberOfStoppedVehiclesSignal = registerSignal("numberOfStoppedVehicles");
    adjustmentsAppliedSignal      = registerSignal("adjustmentsApplied");

    queueBlvdRealVector.setName("queueBlvdReal");
    queueBlvdRealSignal = registerSignal("queueBlvdReal");
    queueSecBsmVector.setName("queueSecBsm");
    queueSecBsmSignal = registerSignal("queueSecBsm");


    trafficLightId        = par("trafficLightId").stdstringValue();
    mainGreenPhase        = par("mainGreenPhase").intValue();
    secondaryGreenPhase   = par("secondaryGreenPhase").intValue();
    isStable              = par("isStable").boolValue();
    dynamicControlEnabled = par("dynamicControlEnabled").boolValue();


    highThreshold       = par("highThreshold").intValue();
    lowThreshold        = par("lowThreshold").intValue();
    intervalCycles      = par("intervalCycles").intValue();
    sampleCycles        = par("sampleCycles").intValue();
    maxStage            = par("maxStage").intValue();
    stageStep           = par("stageStep").intValue();
    downIntervalsNeeded = par("downIntervalsNeeded").intValue();
    queueRange          = par("queueRange").doubleValue();
    stopSpeedThreshold  = par("stopSpeedThreshold").doubleValue();
    debugHeading        = par("debugHeading").boolValue();

    computeBoulevardDirection();

    //coada din sumo
    useTraciQueue = par("useTraciQueue").boolValue();
    {
        std::string manual = par("boulevardLanes").stdstringValue();
        std::stringstream ss(manual); std::string ln;
        while (ss >> ln) blvdLanes.push_back(ln);
        blvdLanesReady = !blvdLanes.empty();
    }
    {
        std::string manual = par("secondaryLanes").stdstringValue();
        std::stringstream ss(manual); std::string ln;
        while (ss >> ln) secLanes.push_back(ln);
        secLanesReady = !secLanes.empty();
    }


    cleanupTimer = new cMessage("cleanupTimer");
    scheduleAt(simTime() + 1.0, cleanupTimer);

    applyTimer = nullptr;

    EV_INFO << "[RSU " << getParentModule()->getFullName() << "] init"
            << " tl=" << trafficLightId
            << " mainPh=" << mainGreenPhase << " secPh=" << secondaryGreenPhase
            << " stable=" << isStable << " dyn=" << dynamicControlEnabled
            << " hasBlvdDir=" << hasBlvdDir << "\n";
}



//  computeBoulevardDirection

void RebreanuRSU::computeBoulevardDirection()
{
    hasBlvdDir = false;

    cModule* parent  = getParentModule();
    cModule* network = parent->getParentModule();
    int i      = parent->getIndex();
    int nTotal = parent->getVectorSize();

    auto rsuPos = [&](int idx, Coord& out) -> bool {
        if (idx < 0 || idx >= nTotal) return false;
        cModule* sib = network->getSubmodule("rsu", idx);
        if (!sib) return false;
        cModule* mob = sib->getSubmodule("mobility");
        if (!mob) return false;
        out = Coord(mob->par("x").doubleValue(), mob->par("y").doubleValue(), 0);
        return true;
    };


    if (!rsuPos(i, myPos)) {
        EV_WARN << "[RSU " << getParentModule()->getFullName()
                << "] nu pot citi pozitia proprie -> clasificare doar pe fereastra\n";
        return;
    }

    Coord pPrev, pNext;
    bool hasPrev = rsuPos(i - 1, pPrev);
    bool hasNext = rsuPos(i + 1, pNext);

    Coord from, to;
    if (hasPrev && hasNext) { from = pPrev; to = pNext; }
    else if (hasNext)       { from = myPos; to = pNext; }
    else if (hasPrev)       { from = pPrev; to = myPos; }
    else return;

    Coord d = to - from;
    double len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 1e-6) return;

    blvdDir = Coord(d.x / len, d.y / len, 0);
    hasBlvdDir = true;
}



//  getTraCI (lazy)

TraCICommandInterface* RebreanuRSU::getTraCI()
{
    if (traci != nullptr) return traci;
    TraCIScenarioManager* manager = TraCIScenarioManagerAccess().get();
    if (manager && manager->isConnected())
        traci = manager->getCommandInterface();
    return traci;
}


//  getRealBoulevardQueue() - Coada Reala de pe Bulevard (E2)

int RebreanuRSU::getRealBoulevardQueue()
{
    if (blvdLanes.empty()) return 0;
    TraCICommandInterface* tc = getTraCI();
    if (tc == nullptr) return 0;
    int q = 0;
    for (auto& ln : blvdLanes) {
        try {
            q += tc->laneAreaDetector("det_" + ln).getLastStepVehicleNumber();
        } catch (...) {}
    }
    return q;
}


//  getRealSecondaryQueue() - Coada Reala pe transversala (E2)

int RebreanuRSU::getRealSecondaryQueue()
{
    if (secLanes.empty()) return 0;
    TraCICommandInterface* tc = getTraCI();
    if (tc == nullptr) return 0;
    int q = 0;
    for (auto& ln : secLanes) {
        try {
            q += tc->laneAreaDetector("det_" + ln).getLastStepVehicleNumber();
        } catch (...) {}
    }
    return q;
}


//  onBSM - acumulare coada pe fereastra de rosu

void RebreanuRSU::onBSM(DemoSafetyMessage* bsm)
{
    int          senderId = bsm->getKind();
    Coord        ss       = bsm->getSenderSpeed();   // x=speed, y=headingX, z=headingY
    double       speed    = ss.x;
    Coord        headingVec(ss.y, ss.z, 0);
    Coord        pos      = bsm->getSenderPos();
    simtime_t    now      = simTime();

    activeVehicles[senderId] = now;
    vehicleSpeeds[senderId]  = speed;
    seenVehicleIds.insert(senderId);
    if (speed < stopSpeedThreshold) stoppedVehicleIds.insert(senderId);

    updateStats();

    if (trafficLightId.empty()) return;

    double dx = pos.x - myPos.x, dy = pos.y - myPos.y;
    if (hasBlvdDir && (dx * dx + dy * dy) > (queueRange * queueRange)) return;

    if (speed >= stopSpeedThreshold) return;

    bool onBlvd = isBoulevardHeading(headingVec);

    if (debugHeading && dbgCount < 50) {
        double len = std::sqrt(headingVec.x * headingVec.x + headingVec.y * headingVec.y);
        double dot = (len > 1e-6 && hasBlvdDir)
                   ? (headingVec.x * blvdDir.x + headingVec.y * blvdDir.y) / len : 0.0;
        EV_WARN << "[HDG " << getParentModule()->getFullName() << "]"
                << " veh=" << senderId << " spd=" << speed
                << " hdg=(" << headingVec.x << "," << headingVec.y << ")"
                << " blvdDir=(" << blvdDir.x << "," << blvdDir.y << ")"
                << " |dot|=" << std::fabs(dot)
                << " -> " << (onBlvd ? "BLVD" : "CROSS")
                << " | phase=" << cachedPhaseIndex
                << " (main=" << mainGreenPhase << ", sec=" << secondaryGreenPhase << ")\n";
        dbgCount++;
    }

    if (onBlvd && cachedPhaseIndex != mainGreenPhase)
        blvdRedSet.insert(senderId);

    if (!onBlvd && cachedPhaseIndex != secondaryGreenPhase)
        secRedSet.insert(senderId);
}

void RebreanuRSU::onWSM(BaseFrame1609_4* wsm) { }



//  isBoulevardHeading

bool RebreanuRSU::isBoulevardHeading(const Coord& h) const
{
    if (!hasBlvdDir) return true;
    double len = std::sqrt(h.x * h.x + h.y * h.y);
    if (len < 1e-6) return true;
    double dot = (h.x * blvdDir.x + h.y * blvdDir.y) / len;
    return std::fabs(dot) > 0.707;
}

void RebreanuRSU::detectLanesForPhase(int phase, std::vector<std::string>& outLanes,
                                      bool& ready, const char* label)
{
    if (ready) return;
    TraCICommandInterface* tc = getTraCI();
    if (tc == nullptr) return;
    auto tl = tc->trafficlight(trafficLightId);
    std::string progId = tl.getCurrentProgramID();
    auto logic = tl.getProgramDefinition().getLogic(progId);
    if (phase >= (int)logic.phases.size()) return;

    std::string st = logic.phases[phase].state;
    std::list<std::string> lanes = tl.getControlledLanes();
    int i = 0; std::set<std::string> seen;
    for (auto& ln : lanes) {
        if (i < (int)st.size() && (st[i]=='G' || st[i]=='g') && !seen.count(ln)) {
            outLanes.push_back(ln); seen.insert(ln);
        }
        i++;
    }
    ready = true;

    std::list<std::string> knownDets = tc->getLaneAreaDetectorIds();
    std::set<std::string> knownSet(knownDets.begin(), knownDets.end());

    std::vector<std::string> filtered;
    for (auto& ln : outLanes)
        if (knownSet.count("det_" + ln)) filtered.push_back(ln);
    outLanes = filtered;

    std::cout << "[RSU " << getParentModule()->getFullName()
              << "] benzi " << label << " cu detector: " << outLanes.size() << "\n";
    for (auto& ln : outLanes) std::cout << "  -> " << ln << "\n";
}

void RebreanuRSU::detectBoulevardLanes()
{
    detectLanesForPhase(mainGreenPhase, blvdLanes, blvdLanesReady, "blvd");
}

void RebreanuRSU::detectSecondaryLanes()
{
    detectLanesForPhase(secondaryGreenPhase, secLanes, secLanesReady, "sec");
}



//  handleSelfMsg

void RebreanuRSU::handleSelfMsg(cMessage* msg)
{
    if (msg == cleanupTimer) {
        cleanupInactiveVehicles();
        updateStats();
        pollTrafficLight();
        scheduleAt(simTime() + 1.0, cleanupTimer);
        return;
    }
    if (msg == applyTimer) {
        if (pendingStage >= 0) {
            applyStage(pendingStage);
            pendingStage = -1;
        }
        return;
    }
    DemoBaseApplLayer::handleSelfMsg(msg);
}



//  pollTrafficLight

void RebreanuRSU::pollTrafficLight()
{
    if (trafficLightId.empty()) return;

    TraCICommandInterface* tc = getTraCI();
    if (tc == nullptr) return;

    auto tl = tc->trafficlight(trafficLightId);

    if (!tlInitialized) {
        std::string progId = tl.getCurrentProgramID();
        auto logic = tl.getProgramDefinition().getLogic(progId);
        if ((int)logic.phases.size() > std::max(mainGreenPhase, secondaryGreenPhase)) {
            baselineMainGreen = logic.phases[mainGreenPhase].duration.dbl();
            baselineSecGreen  = logic.phases[secondaryGreenPhase].duration.dbl();
        }
        prevPhaseIndex = cachedPhaseIndex = tl.getCurrentPhaseIndex();
        tlInitialized = true;
        if (!blvdLanesReady) detectBoulevardLanes();
        if (!secLanesReady)  detectSecondaryLanes();

        if (!secLanes.empty() && !blvdLanes.empty()) {
            std::set<std::string> bset(blvdLanes.begin(), blvdLanes.end());
            std::vector<std::string> kept;
            for (auto& ln : secLanes)
                if (!bset.count(ln)) kept.push_back(ln);
            if (kept.size() != secLanes.size()) {
                std::cout << "[RSU " << getParentModule()->getFullName()
                          << "] reconciliere: " << (secLanes.size()-kept.size())
                          << " banda(e) blvd scoasa(e) din sec\n";
            }
            secLanes = kept;
        }
        return;
    }

    int curPhase = tl.getCurrentPhaseIndex();
    cachedPhaseIndex = curPhase;

    if (prevPhaseIndex != mainGreenPhase && curPhase == mainGreenPhase) {
        int qBsm  = (int)blvdRedSet.size();
        int qReal = useTraciQueue ? getRealBoulevardQueue() : qBsm;
        blvdRedSet.clear();

        queueBlvdHistory.push_back(qReal);
        if ((int)queueBlvdHistory.size() > intervalCycles) queueBlvdHistory.pop_front();

        queueBlvdRealVector.record(qReal);
        emit(queueBlvdRealSignal, (double)qReal);
        queueBlvdVector.record(qBsm);
        emit(queueBlvdSignal, (double)qBsm);

        greenOnCount++;
        std::cout << "[RSU " << getParentModule()->getFullName()
                  << "] GREEN-ON Blvd t=" << simTime()
                  << " REAL_BLVD=" << qReal << " BSM_BLVD=" << qBsm << "\n";

        if (greenOnCount >= intervalCycles) {
            runIntervalDecision();
            greenOnCount = 0;
        }
    }

    if (prevPhaseIndex != secondaryGreenPhase && curPhase == secondaryGreenPhase) {
        int qBsm = (int)secRedSet.size();
        int qReal = useTraciQueue ? getRealSecondaryQueue() : qBsm;
        secRedSet.clear();

        queueSecHistory.push_back(qReal);
        if ((int)queueSecHistory.size() > intervalCycles) queueSecHistory.pop_front();

        queueSecVector.record(qReal);
        emit(queueSecSignal, (double)qReal);
        queueSecBsmVector.record(qBsm);
        emit(queueSecBsmSignal, (double)qBsm);

        std::cout << "[RSU " << getParentModule()->getFullName()
                  << "] GREEN-ON Sec t=" << simTime()
                  << " REAL_SEC=" << qReal << " BSM_SEC=" << qBsm << "\n";
    }

    prevPhaseIndex = curPhase;
}



//  meanLastN
double RebreanuRSU::meanLastN(const std::deque<int>& dq, int n) const
{
    if (dq.empty()) return 0.0;
    int take = std::min((int)dq.size(), n);
    double s = 0.0;
    for (int k = 0; k < take; ++k) s += dq[dq.size() - 1 - k];
    return s / take;
}



//  runIntervalDecision

void RebreanuRSU::runIntervalDecision()
{
    double avgBlvd = meanLastN(queueBlvdHistory, sampleCycles);
    double avgSec  = meanLastN(queueSecHistory,  sampleCycles);

    int oldStage = currentStage;

    if (avgBlvd > highThreshold && avgSec < highThreshold) {
        currentStage = std::min(currentStage + 1, maxStage);
        downCounter = 0;
    }
    else if (avgBlvd > highThreshold && avgSec > highThreshold) {
        downCounter = 0;
    }
    else if (avgBlvd < lowThreshold && currentStage > 0) {
        downCounter++;
        if (downCounter >= downIntervalsNeeded) {
            currentStage = std::max(currentStage - 1, 0);
            downCounter = 0;
        }
    }
    else {
        downCounter = 0;
    }

    std::cout << "[RSU " << getParentModule()->getFullName() << "] DECIZIE t=" << simTime()
            << " avgBlvd=" << avgBlvd << " avgSec=" << avgSec
            << " stage " << oldStage << "->" << currentStage
            << " down=" << downCounter << "\n";

    if (currentStage != oldStage && dynamicControlEnabled && !isStable)
        applyStage(currentStage);

    currentStageVector.record(currentStage);
    emit(currentStageSignal, (double)currentStage);
}



//  applyStage - setarea noilor durate pe semafor

void RebreanuRSU::applyStage(int stage)
{
    if (isStable || trafficLightId.empty() || baselineMainGreen < 0) return;
    TraCICommandInterface* tc = getTraCI();
    if (tc == nullptr) return;

    auto tl = tc->trafficlight(trafficLightId);

    simtime_t nextSwitch = tl.getAssumedNextSwitchTime();
    if ((nextSwitch - simTime()).dbl() <= 5.0) {
        pendingStage = stage;
        if (applyTimer == nullptr) applyTimer = new cMessage("applyTimer");
        if (!applyTimer->isScheduled())
            scheduleAt(nextSwitch + 0.5, applyTimer);
        EV_INFO << "[RSU " << getParentModule()->getFullName()
                << "] aplicare amanata (next switch in "
                << (nextSwitch - simTime()) << "s)\n";
        return;
    }

    double newMain = baselineMainGreen + (double)stage * stageStep;
    double newSec  = baselineSecGreen  - (double)stage * stageStep;

    newMain = std::max((double)minGreen, std::min((double)maxGreen, newMain));
    newSec  = std::max((double)minGreen, std::min((double)maxGreen, newSec));

    std::string progId = tl.getCurrentProgramID();
    auto logic = tl.getProgramDefinition().getLogic(progId);

    double before = 0.0;
    for (auto& ph : logic.phases) before += ph.duration.dbl();

    logic.phases[mainGreenPhase].duration      = newMain;
    logic.phases[secondaryGreenPhase].duration = newSec;

    double after = 0.0;
    for (auto& ph : logic.phases) after += ph.duration.dbl();

    if (std::fabs(after - before) > 0.5) {
        EV_WARN << "[RSU " << getParentModule()->getFullName()
                << "] APLICARE ANULATA: ciclul s-ar schimba (" << before
                << " -> " << after << ").\n";
        return;
    }

    tl.setProgramDefinition(logic, 0);
    adjustmentsApplied++;

    mainGreenDurationVector.record(newMain);
    secGreenDurationVector.record(newSec);
    emit(mainGreenDurationSignal, newMain);
    emit(secGreenDurationSignal, newSec);

    std::cout << "[RSU " << getParentModule()->getFullName() << "] APLICAT stage=" << stage
            << " main=" << newMain << " sec=" << newSec
            << " (ciclu=" << after << ", intra de la ciclul urmator)\n";
}



//  updateStats - statistici instantanee

void RebreanuRSU::updateStats()
{
    simtime_t now = simTime();
    int total = 0, queued = 0;
    double speedSum = 0.0;

    for (auto& kv : activeVehicles) {
        if ((now - kv.second).dbl() > vehicleTimeout) continue;
        total++;
        double spd = vehicleSpeeds[kv.first];
        speedSum += spd;
        if (spd < stopSpeedThreshold) queued++;
    }

    queueLengthVector.record(queued);
    emit(queueLengthSignal, (double)queued);
    totalVehiclesVector.record(total);
    emit(totalVehiclesSignal, (double)total);
    if (total > 0) {
        double avg = (speedSum / total) * 3.6;
        avgSpeedVector.record(avg);
        emit(avgSpeedAtRSUSignal, avg);
    }
}



//  cleanupInactiveVehicles
void RebreanuRSU::cleanupInactiveVehicles()
{
    simtime_t now = simTime();
    std::vector<int> toRemove;
    for (auto& kv : activeVehicles)
        if ((now - kv.second).dbl() > vehicleTimeout) toRemove.push_back(kv.first);
    for (int id : toRemove) {
        activeVehicles.erase(id);
        vehicleSpeeds.erase(id);
    }
}



//  finish
void RebreanuRSU::finish()
{
    emit(vehiclesThroughputSignal,      (double)seenVehicleIds.size());
    emit(numberOfStoppedVehiclesSignal, (double)stoppedVehicleIds.size());
    emit(adjustmentsAppliedSignal,      (double)adjustmentsApplied);

    if (cleanupTimer) { cancelAndDelete(cleanupTimer); cleanupTimer = nullptr; }
    if (applyTimer)   { cancelAndDelete(applyTimer);   applyTimer   = nullptr; }

    EV_INFO << "[RSU " << getParentModule()->getFullName() << "] finish:"
            << " throughput=" << seenVehicleIds.size()
            << " stopped=" << stoppedVehicleIds.size()
            << " adjustments=" << adjustmentsApplied
            << " finalStage=" << currentStage << "\n";

    DemoBaseApplLayer::finish();
}
