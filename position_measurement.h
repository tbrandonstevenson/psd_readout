#ifndef POSITION_MEASUREMENT_H 
#define POSITION_MEASUREMENT_H 

#include "psd_readout.h"
#include <Arduino.h>

struct psdPosition {

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

        x1_sq = 0;
        x2_sq = 0;
        y1_sq = 0;
        y2_sq = 0;

        state = 0; 
    }

    void read (int ipsd) {
        x1 = analogRead(PSD_PIN[ipsd][0]);
        x2 = analogRead(PSD_PIN[ipsd][1]);
        y1 = analogRead(PSD_PIN[ipsd][2]);
        y2 = analogRead(PSD_PIN[ipsd][3]);

        state  = 0;
        state |= voltageNoCal(x1) > analog_threshold[ipsd];
        state |= voltageNoCal(x2) > analog_threshold[ipsd];
        state |= voltageNoCal(y1) > analog_threshold[ipsd];
        state |= voltageNoCal(y2) > analog_threshold[ipsd];
    }


    friend psdMeasurement operator+(const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 + c2.x1;
        pos.x2 = c1.x2 + c2.x2;
        pos.y1 = c1.y1 + c2.y1;
        pos.y2 = c1.y2 + c2.y2;

        pos.x1_sq = c1.x1_sq  + c2.x1_sq ;
        pos.x2_sq = c1.x2_sq  + c2.x2_sq ;
        pos.y1_sq = c1.y1_sq  + c2.y1_sq ;
        pos.y2_sq = c1.y2_sq  + c2.y2_sq ;


        pos.state = c1.state || c2.state; 

        return pos;
    }

    friend psdMeasurement operator- (const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 - c2.x1;
        pos.x2 = c1.x2 - c2.x2;
        pos.y1 = c1.y1 - c2.y1;
        pos.y2 = c1.y2 - c2.y2;

        return pos;
    }

    friend psdMeasurement operator/ (const psdMeasurement &c1, const double &c2)
    {
        psdMeasurement pos;

        pos.x1 = c1.x1 / c2;
        pos.x2 = c1.x2 / c2;
        pos.y1 = c1.y1 / c2;
        pos.y2 = c1.y2 / c2;

        
        pos.x1_sq = c1.x1_sq / c2;
        pos.x2_sq = c1.x2_sq / c2;
        pos.y1_sq = c1.y1_sq / c2;
        pos.y2_sq = c1.y2_sq / c2;

        
        pos.state = c1.state; 

        return pos;
    }

    friend psdMeasurement operator/ (const psdMeasurement &c1, const int &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 / c2;
        pos.x2 = c1.x2 / c2;
        pos.y1 = c1.y1 / c2;
        pos.y2 = c1.y2 / c2;

        pos.x1_sq = c1.x1_sq / c2;
        pos.x2_sq = c1.x2_sq / c2;
        pos.y1_sq = c1.y1_sq / c2;
        pos.y2_sq = c1.y2_sq / c2;

        pos.state = c1.state; 

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
        x1 = (c1.x1);
        x2 = (c1.x2);
        y1 = (c1.y1);
        y2 = (c1.y2);

        x1_sq = (c1.x1_sq);
        x2_sq = (c1.x2_sq);
        y1_sq = (c1.y1_sq);
        y2_sq = (c1.y2_sq);


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

    double x1_sq (int ipsd) {double ret = (ipsd==0) ? psd0.x1_sq : psd1.x1_sq; return ret; }
    double x2_sq (int ipsd) {double ret = (ipsd==0) ? psd0.x2_sq : psd1.x2_sq; return ret; }
    double y1_sq (int ipsd) {double ret = (ipsd==0) ? psd0.y1_sq : psd1.y1_sq; return ret; }
    double y2_sq (int ipsd) {double ret = (ipsd==0) ? psd0.y2_sq : psd1.y2_sq; return ret; }

    double x (int ipsd) {double ret = (ipsd==0) ? psd0.x() : psd1.x(); return ret; }
    double y (int ipsd) {double ret = (ipsd==0) ? psd0.y() : psd1.y(); return ret; }

    bool state () {
        return (psd0.state || psd1.state); 
    } 

    void read () {
        psd0.read(0); 
        psd1.read(1); 
    }

    void reset () {
        psd0.reset(); 
        psd1.reset(); 
    }
}; 

struct dualPosition {
    struct psdPosition psd0; 
    struct psdPosition psd1; 

    double x (int ipsd) {double ret = (ipsd==0) ? psd0.x : psd1.x; return ret; }
    double y (int ipsd) {double ret = (ipsd==0) ? psd0.y : psd1.y; return ret; }

    double x_sq (int ipsd) {double ret = (ipsd==0) ? psd0.x_sq : psd1.x_sq; return ret; }
    double y_sq (int ipsd) {double ret = (ipsd==0) ? psd0.y_sq : psd1.y_sq; return ret; }

    double x_err (int ipsd) {double ret = (ipsd==0) ? psd0.x_err : psd1.x_err; return ret; }
    double y_err (int ipsd) {double ret = (ipsd==0) ? psd0.y_err : psd1.y_err; return ret; }

    void set_x (int ipsd, double value) {
        if (ipsd==0) psd0.x = value; 
        if (ipsd==1) psd1.x = value; 
    }

    void set_y (int ipsd, double value) {
        if (ipsd==0) psd0.y = value; 
        if (ipsd==1) psd1.y = value; 
    }

    void set_x_sq (int ipsd, double value) {
        if (ipsd==0) psd0.x_sq = value; 
        if (ipsd==1) psd1.x_sq = value; 
    }

    void set_y_sq (int ipsd, double value) {
        if (ipsd==0) psd0.y_sq = value; 
        if (ipsd==1) psd1.y_sq = value; 
    }

    void set_x_err (int ipsd, double value) {
        if (ipsd==0) psd0.x_err = value; 
        if (ipsd==1) psd1.x_err = value; 
    }

    void set_y_err (int ipsd, double value) {
        if (ipsd==0) psd0.y_err = value; 
        if (ipsd==1) psd1.y_err = value; 
    }

    void reset () {
        psd0.reset(); 
        psd1.reset(); 
    }

    void sanitize() {
        psd0.sanitize(); 
        psd1.sanitize(); 
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
