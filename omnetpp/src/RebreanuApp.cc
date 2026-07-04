/*
 * RebreanuApp.cc
 */

#include "RebreanuApp.h"

#include <cmath>
#include <sstream>

using namespace veins;

Define_Module(RebreanuApp);


//  initialize

void RebreanuApp::initialize(int stage)
{
    if (stage == 0) {
        speedVector.setName("speed");
        waitingTimeVector.setName("cumulativeWaitingTime");
    }

    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {
        travelTimeSignal    = registerSignal("travelTime");
        waitingTimeSignal   = registerSignal("waitingTime");
        avgSpeedSignal      = registerSignal("avgSpeed");
        numberOfStopsSignal = registerSignal("numberOfStops");
        routeCompletedSignal = registerSignal("routeCompleted");

        gwTravelTimeSignal     = registerSignal("gwTravelTime");
        gwWaitingTimeSignal    = registerSignal("gwWaitingTime");
        gwAvgSpeedSignal       = registerSignal("gwAvgSpeed");
        gwStopsSignal          = registerSignal("gwStops");
        gwRouteCompletedSignal = registerSignal("gwRouteCompleted");

        gwWETravelTimeSignal     = registerSignal("gwWETravelTime");
        gwWEWaitingTimeSignal    = registerSignal("gwWEWaitingTime");
        gwWEAvgSpeedSignal       = registerSignal("gwWEAvgSpeed");
        gwWEStopsSignal          = registerSignal("gwWEStops");
        gwWERouteCompletedSignal = registerSignal("gwWERouteCompleted");

        isOnGreenWaveWE = false;


        greenWavePrefixes.clear();
        std::string raw = par("greenWavePrefixes").stdstringValue();
        std::stringstream ss(raw);
        std::string item;
        while (std::getline(ss, item, ',')) {
            size_t a = item.find_first_not_of(" \t");
            size_t b = item.find_last_not_of(" \t");
            if (a != std::string::npos) {
                greenWavePrefixes.push_back(item.substr(a, b - a + 1));
            }
        }


        departureTime    = simTime();
        totalWaitingTime = 0;
        lastStopTime     = -1;
        lastTxTime       = 0;
        totalSpeed       = 0;
        speedSamples     = 0;
        numberOfStops    = 0;
        isStopped        = false;
        isFinished       = false;

        lastHeadingVec   = Coord(1, 0, 0);   // default pana la prima miscare
        hasLastPos       = false;
        isOnGreenWave    = false;
        gwChecked        = false;
    }
}


//  handlePositionUpdate  (apelat la fiecare 0.1s de TraCI)
void RebreanuApp::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
    if (isFinished) return;

    if (!gwChecked) {
        std::string sumoId = mobility->getExternalId();

        // WE+EW
        for (auto& pfx : greenWavePrefixes) {
            if (!pfx.empty() && sumoId.rfind(pfx, 0) == 0) { isOnGreenWave = true; break; }
        }

        // WE
        std::string rawWE = par("greenWavePrefixesWE").stdstringValue();
        std::stringstream ssWE(rawWE);
        std::string itemWE;
        while (std::getline(ssWE, itemWE, ',')) {
            size_t a = itemWE.find_first_not_of(" \t");
            size_t b = itemWE.find_last_not_of(" \t");
            if (a != std::string::npos) {
                std::string pfx = itemWE.substr(a, b - a + 1);
                if (!pfx.empty() && sumoId.rfind(pfx, 0) == 0) {
                    isOnGreenWaveWE = true; break;
                }
            }
        }

        gwChecked = true;
    }

    double currentSpeed = mobility->getSpeed(); // m/s
    Coord  curPos       = mobility->getPositionAt(simTime());

    speedVector.record(currentSpeed);
    totalSpeed += currentSpeed;
    speedSamples++;


    // Heading din delta de pozitie
    if (hasLastPos && currentSpeed > 1.0) {
        Coord d = curPos - lastPos;
        double len = std::sqrt(d.x * d.x + d.y * d.y);
        if (len > 1e-3)
            lastHeadingVec = Coord(d.x / len, d.y / len, 0);
    }
    lastPos = curPos;
    hasLastPos = true;


    if (currentSpeed < 0.1) {
        if (!isStopped) {
            isStopped = true;
            lastStopTime = simTime();
            numberOfStops++;
        }
    } else {
        if (isStopped) {
            isStopped = false;
            totalWaitingTime += simTime() - lastStopTime;
            waitingTimeVector.record(totalWaitingTime.dbl());
        }
    }

    // Trimite BSM la fiecare 1s
    if (simTime() - lastTxTime >= 1.0) {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        bsm->setKind(myId);
        sendDown(bsm);
        lastTxTime = simTime();
    }
}


//  finish

void RebreanuApp::finish()
{
    if (!isFinished) {
        isFinished = true;

        simtime_t travelTime = simTime() - departureTime;
        double avgSpeed = (speedSamples > 0) ? (totalSpeed / speedSamples) : 0.0;

        if (isStopped && lastStopTime >= 0)
            totalWaitingTime += simTime() - lastStopTime;

        bool finishedRoute = (getSimulation()->getSimulationStage() != omnetpp::CTX_FINISH);

        emit(travelTimeSignal,    travelTime.dbl());
        emit(waitingTimeSignal,   totalWaitingTime.dbl());
        emit(avgSpeedSignal,      avgSpeed);
        emit(numberOfStopsSignal, (double)numberOfStops);
        emit(routeCompletedSignal, finishedRoute ? 1.0 : 0.0);

        if (isOnGreenWave) {
            emit(gwTravelTimeSignal,     travelTime.dbl());
            emit(gwWaitingTimeSignal,    totalWaitingTime.dbl());
            emit(gwAvgSpeedSignal,       avgSpeed);
            emit(gwStopsSignal,          (double)numberOfStops);
            emit(gwRouteCompletedSignal, finishedRoute ? 1.0 : 0.0);
        }
        if (isOnGreenWaveWE) {
            emit(gwWETravelTimeSignal,     travelTime.dbl());
            emit(gwWEWaitingTimeSignal,    totalWaitingTime.dbl());
            emit(gwWEAvgSpeedSignal,       avgSpeed);
            emit(gwWEStopsSignal,          (double)numberOfStops);
            emit(gwWERouteCompletedSignal, finishedRoute ? 1.0 : 0.0);
        }

        EV_INFO << "[RebreanuApp] " << getParentModule()->getFullName()
                << (isOnGreenWave ? " [UNDA VERDE]" : "")
                << " | TravelTime="   << travelTime
                << "s | WaitingTime=" << totalWaitingTime
                << "s | AvgSpeed="    << avgSpeed * 3.6 << " km/h"
                << " | Stops="        << numberOfStops
                << " | Terminat="     << (finishedRoute ? "DA" : "NU") << "\n";
    }
    DemoBaseApplLayer::finish();
}


// populateWSM - pozitie + (viteza, headingX, headingY) in senderSpeed
void RebreanuApp::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    DemoBaseApplLayer::populateWSM(wsm, rcvId, serial);
    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        double spd = mobility->getSpeed();
        bsm->setSenderPos(mobility->getPositionAt(simTime()));
        bsm->setSenderSpeed(Coord(spd, lastHeadingVec.x, lastHeadingVec.y));
    }
}


//  Callbacks goale - nu avem nevoie de V2V

void RebreanuApp::onBSM(DemoSafetyMessage* bsm) { }

void RebreanuApp::onWSM(BaseFrame1609_4* wsm) { }

