struct positionMeasurement {

    int x1 = 0;
    int x2 = 0;
    int y1 = 0;
    int y2 = 0;

    double x() {
      double x = double((x2+x1)/double(x2-x1));
      return x; 
    }

    double y() {
      double y = double((y2+y1)/double(y2-y1));
      return y; 
    }

    void reset () {
        x1 = 0;
        x2 = 0;
        y1 = 0;
        y2 = 0;
    }

    friend positionMeasurement operator+(const positionMeasurement &c1, const positionMeasurement &c2)
    {
        positionMeasurement pos;
        pos.x1 = c1.x1 + c2.x1;
        pos.x2 = c1.x2 + c2.x2;
        pos.y1 = c1.y1 + c2.y1;
        pos.y2 = c1.y2 + c2.y2;
        return pos;
    }

    friend positionMeasurement operator-(const positionMeasurement &c1, const positionMeasurement &c2)
    {
        positionMeasurement pos;
        pos.x1 = c1.x1 - c2.x1;
        pos.x2 = c1.x2 - c2.x2;
        pos.y1 = c1.y1 - c2.y1;
        pos.y2 = c1.y2 - c2.y2;
        return pos;
    }

    friend positionMeasurement operator/(const positionMeasurement &c1, const double &c2)
    {
        positionMeasurement pos;
        pos.x1 = c1.x1 / c2;
        pos.x2 = c1.x2 / c2;
        pos.y1 = c1.y1 / c2;
        pos.y2 = c1.y2 / c2;
        return pos;
    }

    friend positionMeasurement operator/ (const positionMeasurement &c1, const int &c2)
    {
        positionMeasurement pos;
        pos.x1 = c1.x1 / c2;
        pos.x2 = c1.x2 / c2;
        pos.y1 = c1.y1 / c2;
        pos.y2 = c1.y2 / c2;
        return pos;
    }


    friend positionMeasurement operator* (const positionMeasurement &c1, const double &c2)
    {
        positionMeasurement pos;
        pos.x1 = c1.x1 * c2;
        pos.x2 = c1.x2 * c2;
        pos.y1 = c1.y1 * c2;
        pos.y2 = c1.y2 * c2;
        return pos;
    }


    positionMeasurement & operator= (const positionMeasurement &c1)
    {
        //positionMeasurement pos;
        x1 = (c1.x1);
        x2 = (c1.x2);
        y1 = (c1.y1);
        y2 = (c1.y2);
        return *this;
    }
};
