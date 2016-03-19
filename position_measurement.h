#ifndef POSITION_MEASUREMENT_H 
#define POSITION_MEASUREMENT_H 

#include "psd_readout.h"

struct position_t {

    double x; 
    double y; 
    double x_sq; 
    double y_sq; 

    double x_err; 
    double y_err; 

    void reset () {
        x      =  0;
        y      =  0;
        x_sq   =  0;
        y_sq   =  0;
        x_err  =  0;
        y_err  =  0;
    }; 

    /* Sanitizes a position reading to account for various NAN cases */
    void sanitize() {

        /* Handle Cases of NAN*/
        bool data_err = (x!=x || y!=y || x_err!=x_err || y_err!=y_err );

        if (data_err)  {
            x    =999.99999;
            y    =999.99999;
            x_err=  9.99999;
            y_err=  9.99999;
        }
    }

}; 

struct psdMeasurement { 
    bool state; 

    double x1    = 0;
    double x2    = 0;
    double y1    = 0;
    double y2    = 0;

    double x1_sq = 0;
    double x2_sq = 0;
    double y1_sq = 0;
    double y2_sq = 0;

    double x()
    {
      double x = double((x2-x1)/double(x2+x1));
      return x;
    }

    double y()
    {
      double y = double((y2-y1)/double(y2+y1));
      return y;
    }

    void reset ()
    {
        x1 = 0;
        x2 = 0;
        y1 = 0;
        y2 = 0;

        state = 0; 
    }

    void read (int ipsd)
    {
        x1 = analogRead(PSD_PIN[ipsd][0]);
        x2 = analogRead(PSD_PIN[ipsd][1]);
        y1 = analogRead(PSD_PIN[ipsd][2]);
        y2 = analogRead(PSD_PIN[ipsd][3]);

        state = 0;
        state |= voltage(data.x1, ipsd) > analog_threshold[ipsd];
        state |= voltage(data.x2, ipsd) > analog_threshold[ipsd];
        state |= voltage(data.y1, ipsd) > analog_threshold[ipsd];
        state |= voltage(data.y2, ipsd) > analog_threshold[ipsd];
    }


    friend psdMeasurement operator+(const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 + c2.x1;
        pos.x2 = c1.x2 + c2.x2;
        pos.y1 = c1.y1 + c2.y1;
        pos.y2 = c1.y2 + c2.y2;

        pos.state = c1.state || c2.state; 

        return pos;
    }

    friend psdMeasurement operator-(const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 - c2.x1;
        pos.x2 = c1.x2 - c2.x2;
        pos.y1 = c1.y1 - c2.y1;
        pos.y2 = c1.y2 - c2.y2;

        return pos;
    }

    friend psdMeasurement operator/(const psdMeasurement &c1, const double &c2)
    {
        psdMeasurement pos;

        pos.x1 = c1.x1 / c2;
        pos.x2 = c1.x2 / c2;
        pos.y1 = c1.y1 / c2;
        pos.y2 = c1.y2 / c2;

        return pos;
    }

    friend psdMeasurement operator/ (const psdMeasurement &c1, const int &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 / c2;
        pos.x2 = c1.x2 / c2;
        pos.y1 = c1.y1 / c2;
        pos.y2 = c1.y2 / c2;

        return pos;
    }


    friend psdMeasurement operator* (const psdMeasurement &c1, const double &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 * c2;
        pos.x2 = c1.x2 * c2;
        pos.y1 = c1.y1 * c2;
        pos.y2 = c1.y2 * c2;

        return pos;
    }

    friend psdMeasurement operator* (const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 * c2.x1;
        pos.x2 = c1.x2 * c2.x2;
        pos.y1 = c1.y1 * c2.y1;
        pos.y2 = c1.y2 * c2.y2;

        return pos;
    }


    psdMeasurement & operator= (const psdMeasurement &c1)
    {
        //psdMeasurement pos;
        x1 = (c1.x1);
        x2 = (c1.x2);
        y1 = (c1.y1);
        y2 = (c1.y2);

        state = c1.state; 

        return *this;
    }
};

struct dualPSDMeasurement {
    psdMeasurement psd0; 
    psdMeasurement psd1; 

    double x1 (int ipsd) {double ret = (ipsd==0) ? psd0.x1 : psd1.x1; return ret; }
    double x2 (int ipsd) {double ret = (ipsd==0) ? psd0.x2 : psd1.x2; return ret; }
    double y1 (int ipsd) {double ret = (ipsd==0) ? psd0.y1 : psd1.y1; return ret; }
    double y2 (int ipsd) {double ret = (ipsd==0) ? psd0.y2 : psd1.y2; return ret; }

    double x (int ipsd) {double ret = (ipsd==0) ? psd0.x() : psd1.x(); return ret; }
    double y (int ipsd) {double ret = (ipsd==0) ? psd0.y() : psd1.y(); return ret; }

    void read () {
        psd0.read(); 
        psd1.read(); 
    }

    void reset () {
        psd0.reset(); 
        psd1.reset(); 
    }
}; 

struct dualPosition {
    struct position_t psd0; 
    struct position_t psd1; 

    double x (int ipsd) {double ret = (ipsd==0) ? psd0.x() : psd1.x(); return ret; }
    double y (int ipsd) {double ret = (ipsd==0) ? psd0.y() : psd1.y(); return ret; }

    double x_sq (int ipsd) {double ret = (ipsd==0) ? psd0.x_sq() : psd1.x_sq(); return ret; }
    double y_sq (int ipsd) {double ret = (ipsd==0) ? psd0.y_sq() : psd1.y_sq(); return ret; }

    double x_err (int ipsd) {double ret = (ipsd==0) ? psd0.x_err : psd1.x_err; return ret; }
    double y_err (int ipsd) {double ret = (ipsd==0) ? psd0.y_err : psd1.y_err; return ret; }

    void reset () {
        psd0.reset(); 
        psd1.reset(); 
    }

    /* prints a dual position measurement */
    void print() {

        if (print_stddev)   {
            sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 7.5f % 7.5f % 7.5f % 7.5f % 5.2f",
                    x(0),
                    y(0),
                    x(1),
                    y(1),
                    x_err(0),
                    y_err(0),
                    x_err(1),
                    y_err(1),
                    readTemperature());
        }
        else  {
            sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 5.2f",
                    x(0),
                    y(0),
                    x(1),
                    y(1),
                    readTemperature());
        }

        Serial.println(msg);
    }
}; 

#endif
