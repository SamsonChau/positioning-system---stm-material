#include "mbed.h"
#include "encoder.h"

//Pin assignment 
//Encoder 1 measure in X plane
#define Encoder1_PinA PC_0 //PA_13
#define Encoder1_PinB PC_1 //PA_14
int Encoder1_PPR = 2048;

//Encoder 2 measure in Y plane
#define Encoder2_PinA PB_4 //PC_14 
#define Encoder2_PinB PB_5 //PC_15 
int Encoder2_PPR = 2048;

int Encoder1_Omni_Wheel_Radius = 24; //Radius of the Omni Wheel of the encoder 1 in mm
int Encoder2_Omni_Wheel_Radius = 24; //Radius of the Omni Wheel of the encoder 2 in mm

int X_positionToCentre = 1200; //Horizontal distance from robot centre point to the encoder in mm
int Y_positionToCentre = 1200; //Vertical distance from robot centre point to the encoder in mm
float X_Distance = 0;
float Y_Distance = 0;
float PI = 3.14159;

Encoder X_pos(Encoder1_PinA,Encoder1_PinB,Encoder1_PPR ,false);
Encoder Y_pos(Encoder2_PinA,Encoder2_PinB ,Encoder2_PPR,false);
Serial pc(USBTX,USBRX);

// mRotaryEncoder(PinName pinA, PinName pinB, PinName pinSW, PinMode pullMode=PullUp, int debounceTime_us=1000);
int main(void){
    pc.baud(115200);
    float wheel_1_circumference = 2*PI*Encoder1_Omni_Wheel_Radius;
    float wheel_2_circumference = 2*PI*Encoder2_Omni_Wheel_Radius;
    float wheel_1_arc_length = wheel_1_circumference/Encoder1_PPR;
    float wheel_2_arc_length = wheel_2_circumference/Encoder2_PPR;
    pc.printf("wheel_1_circumference: %f  wheel_2_circumference: %f\r\n", wheel_1_circumference , wheel_2_circumference);
    pc.printf("wheel 1 arc_lengthe: %f  wheel 2 arc_length: %f\r\n", wheel_1_arc_length , wheel_1_arc_length);
    while(1){
        X_Distance = wheel_1_arc_length*X_pos.getPosition()/1000;
        Y_Distance = wheel_2_arc_length*Y_pos.getPosition()/1000;
        //pc.printf("X_pos: %d  Y_pos: %d\r\n",X_pos.getPosition() , Y_pos.getPosition() );
       pc.printf("X_dis: %f  Y_dia: %f\r\n", X_Distance ,  Y_Distance );
    }
}
      
 
