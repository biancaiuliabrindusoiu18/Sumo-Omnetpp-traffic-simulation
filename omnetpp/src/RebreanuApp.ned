//
// RebreanuApp.ned
//
package src;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple RebreanuApp extends DemoBaseApplLayer
{
    parameters:
        // Prefixe de ID SUMO care definesc UNDA VERDE (bulevard).
        // flow_WE = Vest->Est (route_0), flow_EW = Est->Vest (route_2).
        // "flow_WE" prinde si autobuzele "flow_WE_bus".
        string greenWavePrefixes = default("flow_WE,flow_EW");
        string greenWavePrefixesWE = default("flow_WE");

        @class(RebreanuApp);
        @display("i=block/app2");

        //  Statistici per vehicul (TOATE vehiculele) 
        @signal[travelTime](type=double);
        @signal[waitingTime](type=double);
        @signal[avgSpeed](type=double);
        @signal[numberOfStops](type=double);
        @statistic[travelTime](source=travelTime; record=mean,count,vector);
        @statistic[waitingTime](source=waitingTime; record=mean,count,vector);
        @statistic[avgSpeed](source=avgSpeed; record=mean,vector);
        @statistic[numberOfStops](source=numberOfStops; record=mean,vector);

        //  Traseu terminat / neterminat 
        @signal[routeCompleted](type=double);
        @statistic[routeCompleted](source=routeCompleted; record=mean,count,sum,vector);

        //  DOAR vehiculele de pe unda verde (bulevard general: WE + EW) 
        @signal[gwTravelTime](type=double);
        @signal[gwWaitingTime](type=double);
        @signal[gwAvgSpeed](type=double);
        @signal[gwStops](type=double);
        @signal[gwRouteCompleted](type=double);
        @statistic[gwTravelTime](source=gwTravelTime; record=mean,count,vector);
        @statistic[gwWaitingTime](source=gwWaitingTime; record=mean,count,vector);
        @statistic[gwAvgSpeed](source=gwAvgSpeed; record=mean,vector);
        @statistic[gwStops](source=gwStops; record=mean,vector);
        @statistic[gwRouteCompleted](source=gwRouteCompleted; record=mean,count,sum,vector);

        //  DOAR vehiculele de pe unda verde WE (Vest -> Est) 
        @signal[gwWETravelTime](type=double);
        @signal[gwWEWaitingTime](type=double);
        @signal[gwWEAvgSpeed](type=double);
        @signal[gwWEStops](type=double);
        @signal[gwWERouteCompleted](type=double);
        @statistic[gwWETravelTime](source=gwWETravelTime; record=mean,count,vector);
        @statistic[gwWEWaitingTime](source=gwWEWaitingTime; record=mean,count,vector);
        @statistic[gwWEAvgSpeed](source=gwWEAvgSpeed; record=mean,vector);
        @statistic[gwWEStops](source=gwWEStops; record=mean,vector);
        @statistic[gwWERouteCompleted](source=gwWERouteCompleted; record=mean,count,sum,vector);

        // co2Emission este inregistrat automat de veinsmobility
}