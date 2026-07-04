//
// RebreanuRSU.ned 
//
package src;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple RebreanuRSU extends DemoBaseApplLayer
{
    parameters:
        //  rol / TraCI 
        bool   dynamicControlEnabled = default(false);
        string trafficLightId        = default("");
        int    mainGreenPhase        = default(0);
        int    secondaryGreenPhase   = default(3);
        bool   isStable              = default(false);


        int initialMainGreenDuration      = default(55);
        int initialSecondaryGreenDuration = default(55);

        //  parametri algoritm (CALIBRABILI din .ini) 
        int    highThreshold       = default(10);   
        int    lowThreshold        = default(2);    
        int    intervalCycles      = default(5);    
        int    sampleCycles        = default(3);    
        int    maxStage            = default(2);
        int    stageStep           = default(10);   
        int    downIntervalsNeeded = default(2);   
        double queueRange          = default(150);
        double stopSpeedThreshold  = default(1.0);  
        bool   debugHeading        = default(false);

		bool useTraciQueue   = default(true);
		string boulevardLanes = default("");   
		string secondaryLanes = default("");   
		
        @class(RebreanuRSU);
        @display("i=block/app2");

        //  instantanee (compatibilitate) 
        @signal[queueLength](type=double);
        @signal[totalVehicles](type=double);
        @signal[avgSpeedAtRSU](type=double);
        @statistic[queueLength](source=queueLength; record=mean,max,min,vector);
        @statistic[totalVehicles](source=totalVehicles; record=mean,max,min,vector);
        @statistic[avgSpeedAtRSU](source=avgSpeedAtRSU; record=mean,max,min,vector);

        //  coada pe fereastra de rosu (per ciclu) 
        @signal[queueBlvd](type=double);
        @signal[queueSecondary](type=double);
        @statistic[queueBlvd](source=queueBlvd; record=mean,max,min,vector);
        @statistic[queueSecondary](source=queueSecondary; record=mean,max,min,vector);

        //  durate verde + stadiu 
        @signal[mainGreenDuration](type=double);
        @signal[secondaryGreenDuration](type=double);
        @signal[currentStage](type=double);
        @statistic[mainGreenDuration](source=mainGreenDuration; record=mean,max,min,vector);
        @statistic[secondaryGreenDuration](source=secondaryGreenDuration; record=mean,max,min,vector);
        @statistic[currentStage](source=currentStage; record=mean,max,min,vector);

        //  cumulative 
        @signal[vehiclesThroughput](type=double);
        @signal[numberOfStoppedVehicles](type=double);
        @signal[adjustmentsApplied](type=double);
        @statistic[vehiclesThroughput](source=vehiclesThroughput; record=last);
        @statistic[numberOfStoppedVehicles](source=numberOfStoppedVehicles; record=last);
        @statistic[adjustmentsApplied](source=adjustmentsApplied; record=last);
        
        @signal[queueBlvdReal](type=double);
		@statistic[queueBlvdReal](title="Coada reala bulevard (SUMO)"; source=queueBlvdReal; record=vector,mean,max,min);
        @signal[queueSecBsm](type=double);
        @statistic[queueSecBsm](title="Coada BSM transversala (V2X)"; source=queueSecBsm; record=vector,mean,max,min);

}
