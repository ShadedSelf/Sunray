// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef __linux__

#include "AmRobotDriver.h"
#include "../../config.h"
#include "../../helper.h"
#include "../../robot.h"
#include "../../pinman.h"
#include "../../cpu.h"
#include "../../ArduinoUniqueID.h"

#if defined(_SAM3XA_)
  #include "../due/DueTimer.h"
#else
  #include "../agcm4/Adafruit_ZeroTimer.h"    // __SAMD51__
#endif


volatile bool leftPressed = false;
volatile bool rightPressed = false;


volatile boolean tone_pin_state = false;


void toneHandler(){  
  digitalWrite(pinBuzzer, tone_pin_state= !tone_pin_state);  
}


#if defined(__SAMD51__)
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3);

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}
#endif 




void AmRobotDriver::begin(){
  CONSOLE.println("using robot driver: AmRobotDriver");
}



void AmRobotDriver::run(){
}


bool AmRobotDriver::getRobotID(String &id){
  id = "";
  for(size_t i = 0; i < UniqueIDsize; i++){
      byte v = UniqueID[i];
      if (v <= 0xF) id += F("0");
      id += String(v, HEX);  
      if (i + 1 < UniqueIDsize) id += ":";
  }
  return true;
}

bool AmRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = "XX";
  ver = "XX";
  return true;
}

float AmRobotDriver::getCpuTemperature(){
  return GetCPUTemperature();  
}


// ------------------------------------------------------------------------------------


// odometry signal change interrupt

#define SUPER_SPIKE_ELIMINATOR 1  // advanced spike elimination  (experimental, comment out to disable)

void AmMotorDriver::OdometryMowISR(){			  
  if (digitalRead(pinMotorMowRpm) == LOW) return;
  if (millis() < motor.mowDriver.motorTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motor.mowDriver.motorTransitionTime;
    if (duration > 5) duration = 0;
    motor.mowDriver.motorTransitionTime = millis();
    motor.mowDriver.motorDurationMax = 0.7 * max(motor.mowDriver.motorDurationMax, ((float)duration));
    motor.mowDriver.motorTicksTimeout = millis() + motor.mowDriver.motorDurationMax;
  #else
    motor.mowDriver.motorTicksTimeout = millis() + 1;
  #endif
  motor.mowDriver.odomTicks++;    
}

void AmMotorDriver::OdometryLeftISR(){			  
  if (digitalRead(pinOdometryLeft) == LOW) return;
  if (millis() < motor.leftDriver.motorTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motor.leftDriver.motorTransitionTime;
    if (duration > 5) duration = 0;
    motor.leftDriver.motorTransitionTime = millis();
    motor.leftDriver.motorDurationMax = 0.7 * max(motor.leftDriver.motorDurationMax, ((float)duration));
    motor.leftDriver.motorTicksTimeout = millis() + motor.leftDriver.motorDurationMax;
  #else
    motor.leftDriver.motorTicksTimeout = millis() + 1;
  #endif
  motor.leftDriver.odomTicks++;    
}

void AmMotorDriver::OdometryRightISR(){			
  if (digitalRead(pinOdometryRight) == LOW) return;  
  if (millis() < motor.rightDriver.motorTicksTimeout) return; // eliminate spikes
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motor.rightDriver.motorTransitionTime;
    if (duration > 5) duration = 0;  
    motor.rightDriver.motorTransitionTime = millis();
    motor.rightDriver.motorDurationMax = 0.7 * max(motor.rightDriver.motorDurationMax, ((float)duration));  
    motor.rightDriver.motorTicksTimeout = millis() + motor.rightDriver.motorDurationMax;
  #else
    motor.rightDriver.motorTicksTimeout = millis() + 1;
  #endif
  motor.rightDriver.odomTicks++;        
  
  #ifdef TEST_PIN_ODOMETRY
    testValue = !testValue;
    digitalWrite(pinKeyArea2, testValue);  
  #endif
}


AmMotorDriver::AmMotorDriver(){

  // default values for all motor drivers (for parameters description, see AmRobotDriver.h)
  
  // MC33926 (https://www.nxp.com/docs/en/data-sheet/MC33926.pdf) - PwmFreqMax=20 khz
  MC33926.driverName = "MC33926";
  MC33926.forwardPwmInvert = false;
  MC33926.forwardDirLevel = LOW;
  MC33926.reversePwmInvert = true;
  MC33926.reverseDirLevel = HIGH;
  MC33926.usePwmRamp = false;
  MC33926.faultActive = LOW;
  MC33926.resetFaultByToggleEnable = true;
  MC33926.enableActive = HIGH;
  MC33926.disableAtPwmZeroSpeed=false;  
  MC33926.keepPwmZeroSpeed = true;
  MC33926.minPwmSpeed = 0;
  MC33926.maxPwmSpeed = 255;
  MC33926.pwmFreq = PWM_FREQ_3900;
  MC33926.adcVoltToAmpOfs = 0;
  MC33926.adcVoltToAmpScale = 1.905 * 2; // ADC voltage to amp for 2 drivers connected in parallel
  MC33926.adcVoltToAmpPow = 1.0;

  // DRV8308 (https://www.ti.com/lit/ds/symlink/drv8308.pdf) - PwmFreqMax=30 khz
  DRV8308.driverName = "DRV8308";
  DRV8308.forwardPwmInvert = false;
  DRV8308.forwardDirLevel = LOW;
  DRV8308.reversePwmInvert = false;
  DRV8308.reverseDirLevel = HIGH;
  DRV8308.usePwmRamp = false;
  DRV8308.faultActive = LOW;
  DRV8308.resetFaultByToggleEnable = true;
  DRV8308.enableActive = LOW;
  DRV8308.disableAtPwmZeroSpeed=false;
  DRV8308.keepPwmZeroSpeed = false; // never go to zero PWM (driver requires a periodic signal)  
  DRV8308.minPwmSpeed = 2;
  DRV8308.maxPwmSpeed = 255;
  DRV8308.pwmFreq = PWM_FREQ_29300;
  DRV8308.adcVoltToAmpOfs = -1.65;   // brushless-adapter: 0A=1.65V, resolution 132mV/A 
  DRV8308.adcVoltToAmpScale = 7.57; 
  DRV8308.adcVoltToAmpPow = 1.0; 

  // A4931 (https://www.allegromicro.com/en/Products/Motor-Driver-And-Interface-ICs/Brushless-DC-Motor-Drivers/~/media/Files/Datasheets/A4931-Datasheet.ashx) - PwmFreqMax=30 kHz
  // alternatives: MS4931 (https://datasheet.lcsc.com/lcsc/1809131539_Hangzhou-Ruimeng-Tech-MS4931_C231944.pdf)
  A4931.driverName = "A4931";
  A4931.forwardPwmInvert = false;
  A4931.forwardDirLevel = HIGH;
  A4931.reversePwmInvert = false;
  A4931.reverseDirLevel = LOW;
  A4931.usePwmRamp = false;
  A4931.faultActive = LOW;
  A4931.resetFaultByToggleEnable = false;
  A4931.enableActive = LOW;  // 'enable' actually is driver brake
  A4931.disableAtPwmZeroSpeed=false;
  A4931.keepPwmZeroSpeed = true;  
  A4931.minPwmSpeed = 0;    
  A4931.maxPwmSpeed = 255;    
  A4931.pwmFreq = PWM_FREQ_29300;   
  A4931.adcVoltToAmpOfs = -1.65;    // brushless-adapter: 0A=1.65V, resolution 132mV/A
  A4931.adcVoltToAmpScale = 7.57;
  A4931.adcVoltToAmpPow = 1.0; 

  // ACT-8015A brushless driver 
  BLDC8015A.driverName = "BLDC8015A";    // just a name for your driver
  BLDC8015A.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  BLDC8015A.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  BLDC8015A.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  BLDC8015A.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  BLDC8015A.usePwmRamp = false;       // use a ramp to get to PWM value?    
  BLDC8015A.faultActive = LOW;        // fault active level (LOW or HIGH)
  BLDC8015A.resetFaultByToggleEnable = false; // reset a fault by toggling enable? 
  BLDC8015A.enableActive = HIGH;       // enable active level (LOW or HIGH)
  BLDC8015A.disableAtPwmZeroSpeed = false;  // disable driver at PWM zero speed? (brake function)
  BLDC8015A.keepPwmZeroSpeed = true;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  BLDC8015A.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  BLDC8015A.maxPwmSpeed = 255;            
  BLDC8015A.pwmFreq = PWM_FREQ_29300;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  BLDC8015A.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)    // brushless-adapter: 0A=1.65V, resolution 132mV/A  
  BLDC8015A.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  BLDC8015A.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)

  // JYQD brushless driver 
  JYQD.driverName = "JYQD";    // just a name for your driver
  JYQD.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  JYQD.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  JYQD.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  JYQD.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  JYQD.usePwmRamp = false;       // use a ramp to get to PWM value?    
  JYQD.faultActive = LOW;        // fault active level (LOW or HIGH)
  JYQD.resetFaultByToggleEnable = true; // reset a fault by toggling enable? 
  JYQD.enableActive = HIGH;       // enable active level (LOW or HIGH)
  JYQD.disableAtPwmZeroSpeed = false;  // disable driver at PWM zero speed? (brake function)
  JYQD.keepPwmZeroSpeed = false;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  JYQD.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  JYQD.maxPwmSpeed = 255;            
  JYQD.pwmFreq = PWM_FREQ_3900;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  JYQD.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)   // brushless-adapter: 0A=1.65V, resolution 132mV/A
  JYQD.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  JYQD.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)

  // your custom brushed/brushless driver (ACT-8015A, JYQD_V7.3E3, etc.)
  CUSTOM.driverName = "CUSTOM";    // just a name for your driver
  CUSTOM.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  CUSTOM.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  CUSTOM.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  CUSTOM.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  CUSTOM.usePwmRamp = false;       // use a ramp to get to PWM value?    
  CUSTOM.faultActive = LOW;        // fault active level (LOW or HIGH)
  CUSTOM.resetFaultByToggleEnable = false; // reset a fault by toggling enable? 
  CUSTOM.enableActive = LOW;       // enable active level (LOW or HIGH)
  CUSTOM.disableAtPwmZeroSpeed=false;  // disable driver at PWM zero speed? (brake function)
  CUSTOM.keepPwmZeroSpeed = false;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  CUSTOM.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  CUSTOM.maxPwmSpeed = 255;          
  CUSTOM.pwmFreq = PWM_FREQ_29300;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  CUSTOM.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)        // brushless-adapter: 0A=1.65V, resolution 132mV/A
  CUSTOM.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  CUSTOM.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)
}
    

void AmMotorDriver::begin(DriverChip chip, int pinEnable, int pinPWM, int pinDir, int pinSense, int pinFault, int pinRpm, bool bindRpm, int pinOdo, bool bindOdo, int pinInterrupt, void(*interrupt)(void)){      

  driverChip = chip;
  pinMode(pinEnable, OUTPUT);
  digitalWrite(pinMotorEnable, driverChip.enableActive);
  pinMode(pinPWM, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(pinSense, INPUT);
  pinMode(pinFault, INPUT);

  if (bindRpm) // mow motor only
  {
	pinMode(pinRpm, INPUT);
  	pinMode(pinRpm, INPUT_PULLUP);  
  }

  if (bindOdo){ // drive motor only
  	pinMode(pinOdo, INPUT_PULLUP);
	//pinMan.setDebounce(pinOdo, 100); // reject spikes shorter than usecs on pin
  }

  pins.enable = pinEnable;
  pins.pwm = pinPWM;
  pins.dir = pinDir;
  pins.sense = pinSense;
  pins.fault = pinFault;
  pins.rpm = pinRpm;
  pins.odometry = pinOdo;

  // enable interrupts
  attachInterrupt(pinInterrupt, interrupt, CHANGE);   

  speedSign = 1;
  lastPwm = 0;

  odomTicks = 0;
  motorTicksTimeout = 0;
  motorTransitionTime = 0;
  motorDurationMax = 0;
}


void AmMotorDriver::run(){
}


// brushed/brushless motor driver
//(8-bit PWM=255, 10-bit PWM=1023)
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   PWM                H     Reverse

void AmMotorDriver::setMotorDriver(int pinDir, int pinPWM, int speed, DriverChip &chip, int speedSign) {
  //DEBUGLN(speed);
  bool reverse = (speedSign < 0);    
  
  if ((speed == 0) && (chip.keepPwmZeroSpeed)) {
    // driver does not require periodic signal at zero speed, we can output 'silence' for zero speed    
  } else {
    // verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
    // Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
    if (abs(speed) < chip.minPwmSpeed) speed = chip.minPwmSpeed * speedSign;
    if (abs(speed) > chip.maxPwmSpeed) speed = chip.maxPwmSpeed * speedSign;  
  }

  if (reverse) {  
    //CONSOLE.print("reverse ");
    //CONSOLE.print(pinDir);
    //CONSOLE.print(",");
    //CONSOLE.print(pinPWM);
    //CONSOLE.print(",");
    //CONSOLE.println(speed);    
    // reverse
    digitalWrite(pinDir, chip.reverseDirLevel) ;
    if (chip.reversePwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM

  } else {
    //CONSOLE.print("forward ");
    //CONSOLE.print(pinDir);
    //CONSOLE.print(",");
    //CONSOLE.print(pinPWM);
    //CONSOLE.print(",");
    //CONSOLE.println(speed);    
    // forward
    digitalWrite(pinDir, chip.forwardDirLevel) ;
    if (chip.forwardPwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM
  }  
}
    
void AmMotorDriver::setMotorPwm(int pwm){  
  // remember speed sign during zero-transition
  if (pwm < 0) speedSign = -1;
  if (pwm > 0) speedSign = 1; 
  
  // limit pwm to ramp if required
  if (driverChip.usePwmRamp){
    int deltaPwm = pwm-lastPwm;
    pwm = pwm + min(1, max(-1, deltaPwm));      
  }

  // remember last PWM values
  lastPwm = pwm;  

  // apply motor PWMs
  setMotorDriver(pins.dir, pins.pwm, pwm, driverChip, speedSign);
  
  // disable driver at zero speed (brake function)    
  bool enabled = driverChip.enableActive;
  if (driverChip.disableAtPwmZeroSpeed){  
    if (pwm == 0){
      enabled = !driverChip.enableActive;                
    }
    digitalWrite(pins.enable, enabled);
  }
}


void AmMotorDriver::getMotorFaults(bool &fault){ 
  if (digitalRead(pins.fault) == driverChip.faultActive) {
    fault = true;
  }
}

void AmMotorDriver::resetMotorFaults(){  
  if (digitalRead(pins.fault) == driverChip.faultActive) {
    if (driverChip.resetFaultByToggleEnable){
      digitalWrite(pins.enable, !driverChip.enableActive);
      digitalWrite(pins.enable, driverChip.enableActive);
    }
  }
}

void AmMotorDriver::getMotorCurrent(float &current){
  // current (amps)= ((ADCvoltage + ofs)^pow) * scale
  float ValuePosCheck	= 0;
  ValuePosCheck = (((float)ADC2voltage(analogRead(pins.sense))) + driverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  current = pow(
      ValuePosCheck, driverChip.adcVoltToAmpPow
      )  * driverChip.adcVoltToAmpScale;
}

void AmMotorDriver::getMotorEncoderTicks(int &ticks){
  ticks = odomTicks;
  // reset counters
  odomTicks = 0;
}    




// ------------------------------------------------------------------------------------


// --- battery switch off circuit --------------------
// JP8 Dauer-ON : automatic switch off circuit disabled
// JP8 Autom.   : automatic switch off circuit enabled
// Note: to increase hardware switch-off time increase capacitor C12  (under DC/DC module)


void AmBatteryDriver::begin(){
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  
  batteryFactor = (100+10) / 10;    // ADC voltage to battery voltage

  //INA169:  A precision amplifier measures the voltage across the Rs=0.1 ohm, 1% sense resistor. 
  //The Rs is rated for 2W continuous so you can measure up to +5A continuous. 
  //The output is a current that is drawn through the on-board RL=10K+10K=20K resistors so that the 
  // output voltage is 2V per Amp. So for 1A draw, the output will be 2V. You can change the 
  // load resistor RL to be smaller by soldering the bridge If you solder the bridge (RL=10K resistor) 
  // you'll get 1V per Amp.   
  //
  // Is = Vout * 1k / (Rs * RL)

  // PCB1.3 (external INA module)
  //   a) bridged      RL=10K:    Is = 1V * 1k / (0.1*10K)  = 1A
  //   b) non-bridged  RL=20k:    Is = 1V * 1k / (0.1*20K)  = 0.5A
  // PCB1.4 (INA soldered on main PCB)
  //   a) bridged      RL=6.8K:   Is = 1V * 1k / (0.05*6.8K)  = 2.941A
  //   b) non-bridged  RL=10.1k:  Is = 1V * 1k / (0.05*10.1K)  = 1.98A
  
  currentFactor = CURRENT_FACTOR;         // ADC voltage to current ampere  (0.5 for non-bridged)

  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);  
  myHumidity.begin();      
}


void AmBatteryDriver::run(){
}

    
float AmBatteryDriver::getBatteryVoltage(){
  float voltage = ((float)ADC2voltage(analogRead(pinBatteryVoltage))) * batteryFactor;
  return voltage;  
}

float AmBatteryDriver::getChargeVoltage(){
  float voltage = ((float)ADC2voltage(analogRead(pinChargeVoltage))) * batteryFactor;
  return voltage;
}


float AmBatteryDriver::getChargeCurrent(){    
  float amps = ((float)ADC2voltage(analogRead(pinChargeCurrent))) * currentFactor;    
	return amps;
}

float AmBatteryDriver::getBatteryTemperature(){
  #ifdef USE_TEMP_SENSOR
    // https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide
    return(myHumidity.readTemperature());
    //float humidity = myHumidity.readHumidity();                  
  #else
    return 0;
  #endif
}

void AmBatteryDriver::enableCharging(bool flag){
  digitalWrite(pinChargeRelay, flag);      
}

void AmBatteryDriver::keepPowerOn(bool flag){
  digitalWrite(pinBatterySwitch, flag);
}


// ------------------------------------------------------------------------------------
void BumperLeftInterruptRoutine(){
  leftPressed = (digitalRead(pinBumperLeft) == LOW);  
}

void BumperRightInterruptRoutine(){
  rightPressed = (digitalRead(pinBumperRight) == LOW);  
}


void AmBumperDriver::begin(){	
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
  attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, CHANGE);
	attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, CHANGE);
}

void AmBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = leftPressed;
  rightBumper = rightPressed;
}

bool AmBumperDriver::obstacle(){
  return (leftPressed || rightPressed);
}
    
bool AmBumperDriver::getLeftBumper(){
  return (leftPressed);
}

bool AmBumperDriver::getRightBumper(){
  return (rightPressed);
}

void AmBumperDriver::run(){  
}


// ------------------------------------------------------------------------------------


void AmStopButtonDriver::begin(){
  nextControlTime = 0;
  pressed = false;  
  pinMode(pinButton, INPUT_PULLUP);  
}

void AmStopButtonDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  pressed = (digitalRead(pinButton)== LOW);
}

bool AmStopButtonDriver::triggered(){
  return pressed;
}


// ------------------------------------------------------------------------------------


void AmRainSensorDriver::begin(){
  nextControlTime = 0;
  isRaining = false;  
  pinMode(pinRain, INPUT);
}

void AmRainSensorDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  isRaining = (digitalRead(pinRain)== LOW);
}

bool AmRainSensorDriver::triggered(){
  return isRaining;
}

// ------------------------------------------------------------------------------------


void AmLiftSensorDriver::begin(){
  nextControlTime = 0;
  isLifted = false;  
  pinMode(pinLift, INPUT_PULLUP);
}

void AmLiftSensorDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  isLifted = (digitalRead(pinLift)== LOW);
}

bool AmLiftSensorDriver::triggered(){
  return isLifted;
}

// ------------------------------------------------------------------------------------

void AmBuzzerDriver::begin(){  
  pinMode(pinBuzzer, OUTPUT);                
  digitalWrite(pinBuzzer, LOW);
}

void AmBuzzerDriver::run(){  
}

void AmBuzzerDriver::noTone(){  
  #ifdef _SAM3XA_
    Timer1.stop();  
    digitalWrite(pinBuzzer, LOW);
  #elif __SAMD51__  // __SAMD51__
    //::noTone(pinBuzzer);     
    zerotimer.enable(false);
    digitalWrite(pinBuzzer, LOW);
  #endif     
}

void AmBuzzerDriver::tone(int freq){  
  #ifdef _SAM3XA_
    pinMode(pinBuzzer, OUTPUT);
    Timer1.attachInterrupt(toneHandler).setFrequency(freq).start();   
  #elif __SAMD51__      // __SAMD51__
    //::tone(pinBuzzer, freq);    

    // Set up the flexible divider/compare
    uint8_t divider  = 1;
    uint16_t compare = 0;
    tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;
    
    divider = 16;
    prescaler = TC_CLOCK_PRESCALER_DIV16;
    compare = (48000000/16)/freq;   
    
    zerotimer.enable(false);
    zerotimer.configure(prescaler,       // prescaler
            TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
            TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
            );

    zerotimer.setCompare(0, compare);
    zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, toneHandler);
    zerotimer.enable(true);
  #endif     
}

#endif  // #ifndef __linux__

