#include "mbed.h"
#include "INA3221.h"

INA3221 INA(PB_9 , PB_8, INA3221_ADDRESS,0.1f,0.1f,0.1f);
DigitalOut myled(LED1);

int main() {
    uint8_t i=0;
    
    if(INA.CheckConnection()){
        printf("Connection OK \n");
    }
    else{
        printf("Error no Connection");
        while(1);
    }
        
    INA.Rest();
    INA.SetMode(INA3221_MODE_SHUNT_BUS_CONTINUOUS);
    INA.SetPowerValidLowerLimitVoltage(4.0f);
    INA.SetPowerValidUpperLimitVoltage(5.0f); 
    INA.SetShuntConversionTime(INA3221_1_1_MS);                          
    INA.SetBusConversionTime(INA3221_1_1_MS);                            
    INA.SetAveragingMode(INA3221_AVERAGE_64);                                        
    
    for(i = 1;i<4;i++)
    {
        INA.EnableChannel(i); 
        INA.EnableChannelSummation(i);
        // The critical-alert feature monitors functions based on individual conversions of each shunt-voltage channel.
        INA.SetCurrentCriticalAlertLimit(i, 0.7 );  // Peak max. 0.7 A
        // The warning alert monitors the averaged value of each shunt-voltage channel.
        INA.SetCurrentWarningAlertLimit(i, 0.5 );  // Average max. 0.5 A
    }
    
    
    while(1) {
        while(INA.ConversionReady()==0);
        
        if(INA.GetPowerVaildAlertFlag()){ 
            printf("Bus Voltage OK\n");
        }
        else{
            printf("Bus Voltage under Lower Limit\n");
        }
        
        printf("Ch1: %f mA Ch2: %f mA Ch3: %f mA\n",INA.GetCurrent(1)*1000.0f,INA.GetCurrent(2)*1000.0f,INA.GetCurrent(3)*1000.0f);
        printf("Ch1: %f V Ch2: %f V Ch3: %f V\n\n",INA.GetBusVoltage(1),INA.GetBusVoltage(2),INA.GetBusVoltage(3));
    
        myled = !myled;
        wait(1);
    }
}


