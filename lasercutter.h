/*                 ******************
              Class for managing my Laser Cutter 
              Featuring: Stepper control with A4988 Driver
                         Rotary encoder
                         I2C screen 
                         
                By: Gal Arbel 2023
                **********************                                   */
#ifndef LASERCUTTER_H
#define LASERCUTTER_H

class Lasercutter { 

  public:
    Lasercutter(int stepPin, int dirPin, int outputA, int outputB, int times, int dmicro); 
    void begin(double bdrate);    
    void ShowInfoLcd(int speed, int direction, int BTstatus);
    void lcdenshow(int clicks, int output, int tempsteps);
    void lcdswitch(bool status);
    void run();



  private:
    int _stepPin;
    int _dirPin;
    int _outputA;
    int _outputB; 
    int _times;
    int _dmicro;
    void rotateCW();  
    void rotateCCW();  


    double PIDcalc(double inp, int sp);
    unsigned long currentTime;
    unsigned long previousTime;
    double elapsedTime;
    double error;
    double lastError;
    double input, output;
    double cumError, rateError;
    double kp = 0;
    double ki = 0; 
    double kd = 0;
    
};


#endif 