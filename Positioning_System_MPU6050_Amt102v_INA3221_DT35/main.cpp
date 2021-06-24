/*
Encoder setup 
 Amt102v 
 pin A require to use a interrrupt pin 
 pin B can be a noraml digital pin 
 pin x is optional and not used in this programe 
 
 Hardware setup:
 MPU6050 Breakout --------- STM32F446RE
 VCC ---------------------- 3.3V
 A ------------------------ A4
 B ------------------------ A5
 GND ---------------------- GND
 
MPU9250 setup 
 SDA and SCL should have external pull-up resistors (to 3.3V)./
 10k resistors worked for me. They should be on the breakout
 board.

 Hardware setup:
 MPU6050 Breakout --------- STM32F446RE
 VCC ---------------------- 3.3V
 SDA ---------------------- SDA
 SCL ---------------------- SCL
 GND ---------------------- GND

Note: The MPU6050 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.

Referenece By  
MPU9250 Basic Example Code
by: Kris Winer
date: April 1, 2014
 */
 
#include "mbed.h"
#include "MPU9250.h"
#include "encoder.h"
#include "BNO055.h"

//Encoder Pin assignment 
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

int X_positionToCentre = 120; //Horizontal distance from robot centre point to the encoder in mm
int Y_positionToCentre = 120; //Vertical distance from robot centre point to the encoder in mm

//Encoder parameter setup 
float X_Distance = 0;
float Y_Distance = 0;
float Encoder_X_Distance = 0;
float Encoder_Y_Distance = 0;
float wheel_1_circumference = 2*PI*Encoder1_Omni_Wheel_Radius;
float wheel_2_circumference = 2*PI*Encoder2_Omni_Wheel_Radius;
float wheel_1_arc_length = wheel_1_circumference/Encoder1_PPR;
float wheel_2_arc_length = wheel_2_circumference/Encoder2_PPR;
float Encoder1_circumference = 2*PI*X_positionToCentre;
float Encoder2_circumference = 2*PI*Y_positionToCentre;
//Constructor
Encoder X_pos(Encoder1_PinA,Encoder1_PinB,Encoder1_PPR ,false);
Encoder Y_pos(Encoder2_PinA,Encoder2_PinB ,Encoder2_PPR,false);

//MPU 9250 setup 
float sum = 0;
uint32_t sumCount = 0;
char buffer[14];
Timer t;
float angle = 0.00;
float preangle = 0.00;
float angleChange = 0.00;
int L_rotation = 0;
int R_rotation = 0;
MPU9250 mpu9250;

//BNO055 Setup
BNO055 imu(I2C_SDA,I2C_SCL);

//General setup
DigitalIn userButton(USER_BUTTON);

DigitalOut led(LED1);//user led
Serial pc(USBTX, USBRX); // tx, rx

void mpu9250_init(){
// Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  pc.printf("I AM 0x%x\n\r", whoami); 
  pc.printf("I SHOULD BE 0x71\n\r");
  
  if (whoami == 0x71) // WHO_AM_I should always be 0x68
  {  
    pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
    pc.printf("MPU9250 is online...\n\r");
    wait(1);
    
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    pc.printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);  
    pc.printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);  
    pc.printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);  
    pc.printf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);  
    pc.printf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);  
    pc.printf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);  
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    pc.printf("x gyro bias = %f\n\r", gyroBias[0]);
    pc.printf("y gyro bias = %f\n\r", gyroBias[1]);
    pc.printf("z gyro bias = %f\n\r", gyroBias[2]);
    pc.printf("x accel bias = %f\n\r", accelBias[0]);
    pc.printf("y accel bias = %f\n\r", accelBias[1]);
    pc.printf("z accel bias = %f\n\r", accelBias[2]);
    wait(2);
    mpu9250.initMPU9250(); 
    pc.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    mpu9250.initAK8963(magCalibration);
    pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
    pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
    if(Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
    if(Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
    if(Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
    if(Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");
    wait(1);
   }
   else
   {
    pc.printf("Could not connect to MPU9250: \n\r");
    pc.printf("%#x \n",  whoami);

    while(1) ; // Loop forever if communication doesn't happen
    }

    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    //Encder_init
      L_rotation = 0;
      R_rotation = 0;
      angle = 0.00;
      preangle = 0.00;
      angleChange = 0.00;
      X_Distance = 0;
      Y_Distance = 0;
      pc.printf("Encoder Reset . . . ");
      X_pos.setPosition(0);
      Y_pos.setPosition(0);
   // Reset the BNO055
    imu.reset();
   // Check that the BNO055 is connected and flash LED if not
    if (!imu.check())
        while (true){
            led = !led;
            wait(0.1);
            }
// Display sensor information
    pc.printf("BNO055 found\r\n\r\n");
    pc.printf("Chip          ID: %0z\r\n",imu.ID.id);
    pc.printf("Accelerometer ID: %0z\r\n",imu.ID.accel);
    pc.printf("Gyroscope     ID: %0z\r\n",imu.ID.gyro);
    pc.printf("Magnetometer  ID: %0z\r\n\r\n",imu.ID.mag);
    pc.printf("Firmware version v%d.%0d\r\n",imu.ID.sw[0],imu.ID.sw[1]);
    pc.printf("Bootloader version v%d\r\n\r\n",imu.ID.bootload);
// Display chip serial number
    for (int i = 0; i<4; i++){
        pc.printf("%0z.%0z.%0z.%0z\r\n",imu.ID.serial[i*4],imu.ID.serial[i*4+1],imu.ID.serial[i*4+2],imu.ID.serial[i*4+3]);
    }
    pc.printf("\r\n");
    imu.setmode(OPERATION_MODE_NDOF);
    pc.printf("Done");

}
int main()
{
  pc.baud(115200);  
  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C  
  t.start();        
  mpu9250_init();
  
 while(1) {
   
   imu.get_mag();
  // If intPin goes high, all data registers have new data
  if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
    // obtain accerlometer data 
    // calculate the accleration value in actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // obtain gyroscope data 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  
    mpu9250.readMagData(magCount);  // Read the x/y/z adc values   
    // obtain the magnetormeter data
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
  }
   
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat;
    sumCount++;
    /*
    if(lastUpdate - firstUpdate > 10000000.0f) {
     beta = 0.04;  // decrease filter gain after stabilized
     zeta = 0.015; // increasey bias drift gain after stabilized
    }
    */
   //Pass gyro rate as rad/s to AHRS fiter 
   //mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
   //  mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
   mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, imu.mag.rawy, imu.mag.rawx, imu.mag.rawz);
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = t.read_ms() - Count;
    if (delt_t > 50) { // update once per half-second independent of read rate
/*
    pc.printf("ax = %f ,", 1000*ax); 
    pc.printf("ay = %f ,", 1000*ay); 
    pc.printf("az = %f  mg\n\r", 1000*az); 

    pc.printf("gx = %f, ", gx); 
    pc.printf("gy = %f, ", gy); 
    pc.printf("gz = %f  deg/s\n\r", gz); 

    pc.printf("mx = %f", mx); 
    pc.printf(" my = %f", my); 
    pc.printf(" mz = %f  mG\n\r", mz); 
    
  //temperaature sensor
  //tempCount = mpu9250.readTempData();  // Read the adc values
  //temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
  //pc.printf(" temperature = %f  C\n\r", temperature); 
  
  //   
  //pc.printf("q0 = %f\n\r", q[0]);
  //pc.printf("q1 = %f\n\r", q[1]);
  //pc.printf("q2 = %f\n\r", q[2]);
  //pc.printf("q3 = %f\n\r", q[3]);      
  */  

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 3.7f; // Declination at Hong Kong =-3.7 degree
    roll  *= 180.0f / PI;
    
            // normalize the angle 
            if(yaw<0){
                 angle = 360-abs(yaw);
            }
            else{
                 angle = abs(yaw);
            }
            // calculate the rotation of the robot 
            if (abs(preangle - angle)>355 && preangle < angle){
                R_rotation++;
                L_rotation--;
            }
            else if(abs(preangle - angle)>355 && preangle > angle){
                R_rotation--;
                L_rotation++;
            }
            if (L_rotation > 0){
                angleChange = (angle/360) + L_rotation;
            }
            else if (R_rotation > 1){
                angleChange = (angle/360) + R_rotation - 1;
            }
            else if(R_rotation==1 && L_rotation ==-1){
                angleChange = (360 - angle) / 360; 
            }
            else {
                angleChange = (angle/360); 
            }
            // Get Encoder data 
            Encoder_X_Distance = wheel_1_arc_length*X_pos.getPosition();
            Encoder_Y_Distance = wheel_2_arc_length*Y_pos.getPosition();
            // Correct the encoder x & y plane measured diatance by imu data 
            X_Distance = Encoder_X_Distance + (angleChange * Encoder1_circumference);
            Y_Distance = Encoder_Y_Distance + (angleChange * Encoder2_circumference);
           // X_Distance = (angleChange * Encoder1_circumference);
           // Y_Distance = (angleChange * Encoder2_circumference);
           //display the displacemnet in mm 
           // pc.printf("angle: %f deg, X_pos: %f, Y_pos: %f ,L: %d, R: %d \r\n",angle,X_Distance,Y_Distance,L_rotation,R_rotation);

           pc.printf("angle: %f deg\r\n",angle);
      //pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
    //myled= !myled;
    Count = t.read_ms(); 
    preangle=angle;
    if(Count > 1<<21) {
        t.start(); // start the timer over again if ~30 minutes has passed
        Count = 0;
        deltat= 0;
        lastUpdate = t.read_us();
    }
    sum = 0;
    sumCount = 0; 
  }
 }
 
}