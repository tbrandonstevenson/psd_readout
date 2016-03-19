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
    }

    friend psdMeasurement operator+(const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 + c2.x1;
        pos.x2 = c1.x2 + c2.x2;
        pos.y1 = c1.y1 + c2.y1;
        pos.y2 = c1.y2 + c2.y2;

        pos.x1_sq = c1.x1_sq + c2.x1_sq;
        pos.x2_sq = c1.x2_sq + c2.x2_sq;
        pos.y1_sq = c1.y1_sq + c2.y1_sq;
        pos.y2_sq = c1.y2_sq + c2.y2_sq;

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

        pos.x1_sq = c1.x1_sq - c2.x1_sq;
        pos.x2_sq = c1.x2_sq - c2.x2_sq;
        pos.y1_sq = c1.y1_sq - c2.y1_sq;
        pos.y2_sq = c1.y2_sq - c2.y2_sq;

        return pos;
    }

    friend psdMeasurement operator/(const psdMeasurement &c1, const double &c2)
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
        return pos;
    }


    friend psdMeasurement operator* (const psdMeasurement &c1, const double &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 * c2;
        pos.x2 = c1.x2 * c2;
        pos.y1 = c1.y1 * c2;
        pos.y2 = c1.y2 * c2;

        pos.x1_sq = c1.x1_sq * c2;
        pos.x2_sq = c1.x2_sq * c2;
        pos.y1_sq = c1.y1_sq * c2;
        pos.y2_sq = c1.y2_sq * c2;
        return pos;
    }

    friend psdMeasurement operator* (const psdMeasurement &c1, const psdMeasurement &c2)
    {
        psdMeasurement pos;
        pos.x1 = c1.x1 * c2.x1;
        pos.x2 = c1.x2 * c2.x2;
        pos.y1 = c1.y1 * c2.y1;
        pos.y2 = c1.y2 * c2.y2;

        pos.x1_sq = c1.x1_sq * c2.x1_sq;
        pos.x2_sq = c1.x2_sq * c2.x2_sq;
        pos.y1_sq = c1.y1_sq * c2.y1_sq;
        pos.y2_sq = c1.y2_sq * c2.y2_sq;

        return pos;
    }


    psdMeasurement & operator= (const psdMeasurement &c1)
    {
        //psdMeasurement pos;
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
