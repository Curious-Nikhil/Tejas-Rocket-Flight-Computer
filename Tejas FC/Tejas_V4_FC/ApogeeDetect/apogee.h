#ifndef APOGEE
#define APOGEE

//Author - Nyameaama Gambrah

#include"../Adafruit_BMP280.h"

#define SAMPLE_SIZE 5

class APOGEE_DETECT {
    private:
        Adafruit_BMP280 get;
        double *speed_of_increase = (double*)malloc(SAMPLE_SIZE);
        
    private:
        //Function to compile altitude vals with time metadata
        double *getAltVals();

        //Function gets duration between two timestamps
        double timeBetween(double x,double y);

    public:
        //Function to analyse velocity for a decceleration pattern
        boolean analyseAltDecceleration();

        //Function to compute estimated time to apogee
        double time_to_Apogee();
};


#endif