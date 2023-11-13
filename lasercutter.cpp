#include "lasercutter.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

LiquidCrystal_I2C lcd(0x27,16,2);
int counter = 0;
int angle = 0; 
int aState;
int aLastState;  
Lasercutter::Lasercutter(int stepPin, int dirPin, int outputA, int outputB, int times, int dmicro){
  
  _stepPin = stepPin;
  _dirPin = dirPin;
  _outputA = outputA;
  _outputB = outputB;
  _times = times;  
  _dmicro = dmicro;
  pinMode(_stepPin, OUTPUT); 
  pinMode(_dirPin, OUTPUT);
  pinMode (_outputA, INPUT);
  pinMode (_outputB, INPUT);

}

void Lasercutter::begin(double bdrate) {
  delay(30);
  Serial.begin(bdrate);      
  Serial.println("Started");
  Serial.print("stepPin = ");  
  Serial.println(_stepPin);
  Serial.print("dirPin = ");  
  Serial.println(_dirPin);
  Serial.print("outputA = ");  
  Serial.println(_outputA);
  Serial.print("outputB = ");  
  Serial.println(_outputB);
   
  lcd.init();  
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("Laser Cutter");
  lcd.setCursor(1, 1);
  lcd.print("Setup finished");
  lcd.setBacklight(0);
  
  aLastState = digitalRead(_outputA);

  Serial.println("Setup finished");
}


double Lasercutter::PIDcalc(double inp, int sp){
  //  Serial.println(steps);

   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   if(rateError > 0.3 || rateError < -0.3){cumError = 0;}             // reset the Integral commulator when Proportional is doing the work

   double out = kp*error + ki*cumError + kd*rateError; //PID output               

   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   if(out > 254){out = 254;}    //limit the function for smoother operation
   if(out < -254){out = -254;}
   if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
   return out;                                        //the function returns the PID output value 
  
}


void Lasercutter::lcdswitch(bool status){
  if(status){lcd.backlight();}
  else{lcd.setBacklight(0);}
}

void Lasercutter::ShowInfoLcd(int speed, int direction, int BTstatus){ 
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("speed| dir | bat");
  lcd.setCursor(1,1);
  lcd.print(speed);  
  lcd.setCursor(4,1);
  lcd.print(" |  "); 
  lcd.setCursor(7,1);
  lcd.print(direction); 
  lcd.setCursor(10,1);
  lcd.print(" | ");  
  lcd.setCursor(13,1);

}

void Lasercutter::lcdenshow(int clicks, int output, int tempsteps){ 
 // lcd.setBacklight(1);
  lcd.clear();
  lcd.setCursor(0, 0);
   //        0123456789012345 
  lcd.print(" SP | PID |steps");
  lcd.setCursor(0,1);
  lcd.print(clicks);  
  lcd.setCursor(4,1);
  lcd.print("|"); 
  lcd.setCursor(6,1);
  lcd.print(output); 
  lcd.setCursor(10,1);
  lcd.print("|");  
  lcd.setCursor(12,1);
}

void Lasercutter::run(){
  aState = digitalRead(_outputA);
  if (aState != aLastState){     
     if (digitalRead(_outputB) != aState) { 
       counter ++;
       angle ++;
       rotateCW();  
     }
     else {
       counter--;
       angle --;
       rotateCCW(); 
     }
     if (counter >= 30 or counter <= -30 ) {
      counter = 0;
     }
     /*
     lcd.clear();
     lcd.print("Position: ");
     lcd.print(int(angle*(-1.8)));
     lcd.print("deg"); 
     lcd.setCursor(0,0);
    */ 
    Serial.print("Position: ");
    Serial.println(counter);
   }
  aLastState = aState;
}

void Lasercutter::rotateCW() {
  digitalWrite(_dirPin,LOW);
  for(int i = 0; i < _times; i++){
    digitalWrite(_stepPin,HIGH);
    delayMicroseconds(_dmicro);
    digitalWrite(_stepPin,LOW);
    delayMicroseconds(_dmicro); 
  }
}
void Lasercutter::rotateCCW() {
  digitalWrite(_dirPin,HIGH);
  for(int i = 0; i < _times; i++){
    digitalWrite(_stepPin,HIGH);
    delayMicroseconds(_dmicro);
    digitalWrite(_stepPin,LOW);
    delayMicroseconds(_dmicro); 
 }  
}

