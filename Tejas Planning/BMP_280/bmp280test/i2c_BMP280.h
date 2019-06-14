#ifndef i2c_bmp280_h
#define i2c_bmp280_h

#include "i2c.h"
#include "i2c_Sensor.h"


/** ######################################################################

Driver for the BMP280-Sensor

 CONSUMPTION: standby 0.5 µA, measure 4.2@1Hz, 260-1120µA

 ONE-TIME-MEASURE: disable sensor, [start measurement, wait, read ] ...
 AUTO-Measure: enable sensor, start measurement, [read, read ] ...


########################################################################  */

class BMP280 : public i2cSensor, public manualSensor
{

private:

    /** ######### Register-Map ################################################################# */
    static const uint8_t I2C_ADDRESS 	                =(0x76);

// CALIBRATION DATA, 25 Register. 0x88 - 0xA1
    static const uint8_t REG_DIG_T1                     =(0x88);    // watch out - switched MSB/LSB
    static const uint8_t REG_DIG_T2                     =(0x8A);
    static const uint8_t REG_DIG_T3                     =(0x8C);
    static const uint8_t REG_DIG_P1                     =(0x8E);
    static const uint8_t REG_DIG_P2                     =(0x90);
    static const uint8_t REG_DIG_P3                     =(0x92);
    static const uint8_t REG_DIG_P4                     =(0x94);
    static const uint8_t REG_DIG_P5                     =(0x96);
    static const uint8_t REG_DIG_P6                     =(0x98);
    static const uint8_t REG_DIG_P7                     =(0x9A);
    static const uint8_t REG_DIG_P8                     =(0x9C);
    static const uint8_t REG_DIG_P9                     =(0x9E);

    static const uint8_t REG_ID						    =(0xD0);
    static const uint8_t		VAL_ID					=(0x58);

    static const uint8_t REG_RESET					    =(0xE0);
    static const uint8_t 	VAL_RESET				    =(0xB6); 	// write it to trigger POR

    static const uint8_t REG_STATUS					    =(0xF3);
    static const uint8_t 	MSK_STATUS_MEASURING	    =(1<<3);	// 1 when conversion is running
    static const uint8_t 	MSK_STATUS_IMUPDATE		    =(1<<0);	// 1 when NVM data is copied to image registers

    static const uint8_t REG_CTRL_MEAS				    =(0xF4);
    static const uint8_t 	MSK_CTRL_OSRS_T			    =(B11100000);
    static const uint8_t 		VAL_CTRL_OSRS_T00	    =(B00000000); // skip measurement
    static const uint8_t 		VAL_CTRL_OSRS_T01	    =(B00100000); // 1x (no oversampling)
    static const uint8_t 		VAL_CTRL_OSRS_T02	    =(B01000000); // 2x --> 17bit, 2m°C
    static const uint8_t 		VAL_CTRL_OSRS_T04	    =(B01100000); // 4x  	(brings no improvement)
    static const uint8_t 		VAL_CTRL_OSRS_T08	    =(B10000000); // 8x		(brings no improvement)
    static const uint8_t 		VAL_CTRL_OSRS_T16	    =(B10100000); // 16x	(brings no improvement)
    static const uint8_t 	MSK_CTRL_OSRS_P			    =(B00011100);
    static const uint8_t 		VAL_CTRL_OSRS_P00	    =(B00000000); // skip measurement
    static const uint8_t 		VAL_CTRL_OSRS_P01	    =(B00000100); // 1x (no oversampling)
    static const uint8_t 		VAL_CTRL_OSRS_P02	    =(B00001000); // 2x
    static const uint8_t 		VAL_CTRL_OSRS_P04	    =(B00001100); // 4x
    static const uint8_t 		VAL_CTRL_OSRS_P08	    =(B00010000); // 8x
    static const uint8_t 		VAL_CTRL_OSRS_P16	    =(B00010100); // 16x --> 20 bit, 0.16 Pa
    static const uint8_t 	MSK_CTRL_MODE			    =(B00000011);
    static const uint8_t 		VAL_MODE_SLEEP		    =(B00000000);	// low power
    static const uint8_t 		VAL_MODE_FORCED		    =(B00000001);	// manual
    static const uint8_t 		VAL_MODE_NORMAL		    =(B00000011); 	// automatic

    static const uint8_t REG_CONFIG					    =(0xF5);
    static const uint8_t 	MSK_CONFIG_T_SB			    =(B11100000);
    static const uint8_t 		VAL_SB_0000			    =(B00000000);
    static const uint8_t 		VAL_SB_0062			    =(B00100000);
    static const uint8_t 		VAL_SB_0125			    =(B01000000);
    static const uint8_t 		VAL_SB_0250			    =(B01100000);
    static const uint8_t 		VAL_SB_0500			    =(B10000000);
    static const uint8_t 		VAL_SB_1000			    =(B10100000);
    static const uint8_t 		VAL_SB_2000			    =(B11000000);
    static const uint8_t 		VAL_SB_4000			    =(B11100000);
    static const uint8_t 	MSK_CONFIG_FILTER		    =(B00011100);
    static const uint8_t 		VAL_FILTER_00		    =(B00000000);	// full BW
    static const uint8_t 		VAL_FILTER_02 		    =(B00000100);	// 0.223 * ODR
    static const uint8_t 		VAL_FILTER_04 		    =(B00001000);	// 0.092 * ODR
    static const uint8_t 		VAL_FILTER_08 		    =(B00001100);	// 0.042 * ODR
    static const uint8_t 		VAL_FILTER_16 		    =(B00010000);	// 0.021 * ODR
    static const uint8_t 	MSK_CONFIG_SPI3W_EN		    =(B00000001);	// 1 = activate SPI-Mode

    static const uint8_t REG_PRESS_MSB				    =(0xF7);
    static const uint8_t REG_PRESS_LSB				    =(0xF8);
    static const uint8_t REG_PRESS_XLSB				    =(0xF9); 	// bit 4-7 usable

    static const uint8_t REG_TEMP_MSB				    =(0xFA);
    static const uint8_t REG_TEMP_LSB				    =(0xFB);
    static const uint8_t REG_TEMP_XLSB				    =(0xFC);	// bit 4-7 usable

    uint16_t    dig_T1, dig_P1;
    int16_t     dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t     tFine;

    /** ######### function definition ################################################################# */

public:

    BMP280(void) :  dig_T1(0), dig_P1(0), dig_T2(0), dig_T3(0), dig_P2(0), dig_P3(0), dig_P4(0), dig_P5(0), dig_P6(0), dig_P7(0), dig_P8(0), dig_P9(0), tFine(0)
    {
    };

    /**< Enable / Disable the Sensor */
    inline void    setEnabled(const uint8_t enable = 1)
    {
        uint8_t _value;
        if (enable) _value=VAL_MODE_NORMAL;
        else        _value=0;
        i2c.setRegister(I2C_ADDRESS,REG_CTRL_MEAS, MSK_CTRL_MODE, _value);
    };

    /**< read Enable / Disable - Status */
    inline uint8_t getEnabled(void)
    {
        return (3 & i2c.readByte(I2C_ADDRESS,REG_CTRL_MEAS));
    };

    /**< do a software reset */
    inline void    reset(void)
    {
        i2c.writeByte(I2C_ADDRESS,REG_RESET, VAL_RESET);
    };

    /**<  */
    inline void setPressureOversampleRatio(const uint8_t sampleRatio = 16)
    {
        uint8_t _value;
        if 		(sampleRatio > 15) 	_value = VAL_CTRL_OSRS_P16;
        else if (sampleRatio > 7)	_value = VAL_CTRL_OSRS_P08;
        else if (sampleRatio > 3)	_value = VAL_CTRL_OSRS_P04;
        else if (sampleRatio > 1)	_value = VAL_CTRL_OSRS_P02;
        else if (sampleRatio > 0)	_value = VAL_CTRL_OSRS_P01;
        else  						_value = VAL_CTRL_OSRS_P00; // disable!!!
        i2c.setRegister(I2C_ADDRESS,REG_CTRL_MEAS, MSK_CTRL_OSRS_P, _value);
    };

    inline void setTemperatureOversampleRatio(const uint8_t sampleRatio = 2)
    {
        uint8_t _value;
        if 		(sampleRatio > 15) 	_value = VAL_CTRL_OSRS_T16;
        else if (sampleRatio > 7)	_value = VAL_CTRL_OSRS_T08;
        else if (sampleRatio > 3)	_value = VAL_CTRL_OSRS_T04; // more isn't better
        else if (sampleRatio > 1)	_value = VAL_CTRL_OSRS_T02; // 2 should be maximum
        else if (sampleRatio > 0)	_value = VAL_CTRL_OSRS_T01;
        else  						_value = VAL_CTRL_OSRS_T00; // disable!!!
        i2c.setRegister(I2C_ADDRESS,REG_CTRL_MEAS, MSK_CTRL_OSRS_T, _value);
    };

    inline void setFilterRatio(const uint8_t filterRatio = 0)
    {
        uint8_t _value;
        if 		(filterRatio > 15) 	_value = VAL_FILTER_16;
        else if (filterRatio > 7)	_value = VAL_FILTER_08;
        else if (filterRatio > 3)	_value = VAL_FILTER_04;
        else if (filterRatio > 1)	_value = VAL_FILTER_02;
        else  						_value = VAL_FILTER_00; // disable!!!
        i2c.setRegister(I2C_ADDRESS,REG_CONFIG, MSK_CONFIG_FILTER, _value);
    };

    inline void setStandby(const uint16_t ms = 0)
    {
        uint8_t _value;
        if 		(ms > 3000) _value = VAL_SB_4000;
        else if (ms > 1500)	_value = VAL_SB_2000;
        else if (ms >  750)	_value = VAL_SB_1000;
        else if (ms >  350)	_value = VAL_SB_0500;
        else if (ms >  180)	_value = VAL_SB_0250;
        else if (ms >   90)	_value = VAL_SB_0125;
        else if (ms >   31)	_value = VAL_SB_0062;
        else  				_value = VAL_SB_0000; // disable!!!
        i2c.setRegister(I2C_ADDRESS,REG_CONFIG, MSK_CONFIG_T_SB, _value);
    }

    /**< initialize */
    inline uint8_t initialize(void)
    {
        if (i2c.probe(I2C_ADDRESS)==0) return 0;

        reset();
        delay(4);

        setPressureOversampleRatio(16);
        setTemperatureOversampleRatio(2);
        setFilterRatio();
        setStandby();
        i2c.setRegister(I2C_ADDRESS,REG_CONFIG, MSK_CONFIG_SPI3W_EN, 0);

        readTrimming();
        /*
                Serial.println("");
                Serial.println(dig_T1);
                Serial.println(dig_T2);
                Serial.println(dig_T3);
        */
        setEnabled(1);
        return 1;
    };

    void readTrimming()
    {
        uint8_t _value[2];
        i2c.read(I2C_ADDRESS, REG_DIG_T1, _value, 2);
        dig_T1 = uint16_t((uint16_t(_value[1]<<8)) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_T2, _value, 2);
        dig_T2 = int16_t((_value[1]<<8) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_T3, _value, 2);
        dig_T3 = int16_t((_value[1]<<8) | _value[0]);

        i2c.read(I2C_ADDRESS, REG_DIG_P1, _value, 2);
        dig_P1 = uint16_t((uint16_t(_value[1]<<8)) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_P2, _value, 2);
        dig_P2 = int16_t((_value[1]<<8) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_P3, _value, 2);
        dig_P3 = int16_t((_value[1]<<8) | _value[0]);

        i2c.read(I2C_ADDRESS, REG_DIG_P4, _value, 2);
        dig_P4 = int16_t((_value[1]<<8) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_P5, _value, 2);
        dig_P5 = int16_t((_value[1]<<8) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_P6, _value, 2);
        dig_P6 = int16_t((_value[1]<<8) | _value[0]);

        i2c.read(I2C_ADDRESS, REG_DIG_P7, _value, 2);
        dig_P7 = int16_t((_value[1]<<8) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_P8, _value, 2);
        dig_P8 = int16_t((_value[1]<<8) | _value[0]);
        i2c.read(I2C_ADDRESS, REG_DIG_P9, _value, 2);
        dig_P9 = int16_t((_value[1]<<8) | _value[0]);
    };


    /**< disables continious Mode! (enable(1)) */
    inline void triggerMeasurement(void)
    {
        i2c.setRegister(I2C_ADDRESS,REG_CTRL_MEAS, MSK_CTRL_MODE, VAL_MODE_FORCED);
    };

    inline uint8_t checkMeasurement(void)
    {
        return !(MSK_STATUS_MEASURING & i2c.readByte(I2C_ADDRESS, REG_STATUS));
    };

    /**<  if you started a measurement and want to actively wait for it to finish */
    inline uint8_t awaitMeasurement(void)
    {
        uint8_t _counter = 0;
        while(checkMeasurement()==0)
        {
            if(++_counter > 250) return 0; //Error out after max of 500ms for a read
            delay(2);
        }
        return 1; // Measurement finished
    };

    /**<  gives airpressure in Pascal */
    void getPressure(uint32_t& pascal)
    {
        uint8_t _value[3];
        i2c.read(I2C_ADDRESS, REG_PRESS_MSB, _value, 3);

        int32_t var1, var2, adc;
        adc     = (uint32_t( uint16_t(_value[0] << 8) | _value[1])<<4) | (_value[2]>>4);
        var1 = (((int32_t)tFine)>>1) - (int32_t)64000;
        var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
        var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
        var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
        var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
        var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
        if (var1 == 0) return; // avoid exception caused by division by zero

        pascal = (((uint32_t)(((int32_t)1048576)-adc)-(var2>>12)))*3125;
        if (pascal < 0x80000000)    pascal = (pascal << 1) / ((uint32_t)var1);
        else                        pascal = (pascal / (uint32_t)var1) * 2;

        var1 = (((int32_t)dig_P9) * ((int32_t)(((pascal>>3) * (pascal>>3))>>13)))>>12;
        var2 = (((int32_t)(pascal>>2)) * ((int32_t)dig_P8))>>13;
        pascal = (uint32_t)((int32_t)pascal + ((var1 + var2 + dig_P7) >> 4));
    };

    void getPressure(float& pascal)
    {
        uint32_t iPascal;
        getPressure(iPascal);
        pascal = float(iPascal);
    }

    /**<  gives pressure-values */
    void getMeasurement(float& pascal)
    {
        getPressure(pascal);
    };

    /**<  gives the number of meters above sea level */
    void getAltitude(float& meter)
    {
        uint32_t iPascal;
        getPressure(iPascal);

        meter = 44330.0*(1-pow(float(iPascal)/101325.0,1.0/5.255));

    };

    /**<  gives temperature in degree celsius */
    void getTemperature(int32_t& millicelsius)
    {
        uint8_t value[3];
        i2c.read(I2C_ADDRESS, REG_TEMP_MSB, value, 3);

        int32_t var1, var2, adc;
        adc     = (uint32_t( uint16_t(value[0] << 8) | value[1])<<4) | (value[2]>>4);
        var1    = ((((adc>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
        var2    = (((((adc>>4) - ((int32_t)dig_T1)) * ((adc>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3))>>14;
        //var2 = ((adc>>4) - ((int32_t)dig_T1));
        //var2 = (((var2 * var2) >> 12) * ((int32_t)dig_T3))>>14;
        tFine   = var1 + var2;
        millicelsius = (tFine * 50 + 1280) >> 8;

    };

    void getTemperature(float& celsius)
    {
        int32_t iTemperature;
        getTemperature(iTemperature);
        celsius = float(iTemperature) / 1000;
    };

};

/** ######### Preinstantiate Object ################################################################# */
/** < it's better when this is done by the user */
//BMP280 bmp280 = BMP280();

#endif



