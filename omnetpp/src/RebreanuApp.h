/*
 * RebreanuApp.h
 *
 * Aplicatia vehiculelor
 *
 * Colecteaza per vehicul:
 *   - travelTime    : durata totala a cursei
 *   - waitingTime   : timp total stat oprit la semafoare
 *   - avgSpeed      : viteza medie pe cursa (m/s)
 *   - numberOfStops : de cate ori s-a oprit vehiculul
 *
 * trimite BSM la fiecare 1.0s cu
 *   - pozitie
 *   - viteza + vector de heading impachetate in senderSpeed:
 *         senderSpeed.x  viteza scalara (m/s)
 *         senderSpeed.y  heading.x
 *         senderSpeed.z  heading.y
 *
 */


#ifndef REBREANUAPP_H_
#define REBREANUAPP_H_

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

class RebreanuApp : public veins::DemoBaseApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;

protected:
    void onBSM(veins::DemoSafetyMessage* bsm) override;
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void populateWSM(veins::BaseFrame1609_4* wsm,
                     veins::LAddress::L2Type rcvId = veins::LAddress::L2BROADCAST(),
                     int serial = 0) override;
    void handlePositionUpdate(cObject* obj) override;

private:
    //  Timpi
    simtime_t departureTime;      // cand a intrat vehiculul in simulare
    simtime_t totalWaitingTime;   // total timp stat oprit
    simtime_t lastStopTime;       // momentul ultimei opriri
    simtime_t lastTxTime;         // momentul ultimului beacon trimis

    //  Viteza
    double totalSpeed;
    int    speedSamples;

    //  Opriri
    int numberOfStops;

    //  Stare curenta
    bool isStopped;
    bool isFinished;

    //  Heading
    veins::Coord lastHeadingVec;
    veins::Coord lastPos;
    bool         hasLastPos;


    bool isOnGreenWave = false;
    bool gwChecked     = false;
    std::vector<std::string> greenWavePrefixes;

    bool isOnGreenWaveWE = false;

    simsignal_t gwWETravelTimeSignal;
    simsignal_t gwWEWaitingTimeSignal;
    simsignal_t gwWEAvgSpeedSignal;
    simsignal_t gwWEStopsSignal;
    simsignal_t gwWERouteCompletedSignal;


    cOutVector  speedVector;
    cOutVector  waitingTimeVector;

    simsignal_t travelTimeSignal;
    simsignal_t waitingTimeSignal;
    simsignal_t avgSpeedSignal;
    simsignal_t numberOfStopsSignal;

    simsignal_t routeCompletedSignal;   // 1 = a ajuns la destinatie, 0 = blocat la final

    simsignal_t gwTravelTimeSignal;
    simsignal_t gwWaitingTimeSignal;
    simsignal_t gwAvgSpeedSignal;
    simsignal_t gwStopsSignal;
    simsignal_t gwRouteCompletedSignal;
};

#endif /* REBREANUAPP_H_ */
