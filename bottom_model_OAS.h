//
// C++ Interface: bottom_model
//
// Description: 
//
//
// Author: Teledyne Gavia ehf. <info@gavia.is>, (C) 2014
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef BOTTOM_MODEL_OAS_H
#define BOTTOM_MODEL_OAS_H

#include <vector>

namespace TestOAS{

struct coordinate
{
  double x;
  double depth;
};

struct line
{
  double x1;
  double x2;
  double y1;
  double y2;
};

class BottomModel
{
  public:
    BottomModel() {}
    ~BottomModel();
    /**
     * Reads a csv file with a list of (x,z) values. x values must be strictly increasing
     */
    void readFile(const char* filename);
    /**
     * Returns vertical downward distance from given location x,z.
     * If below the bottom, negative values are returned.
     */
    double getAltitude(double x, double z);

    line getBottomLine(double x);
  private:
    std::vector<coordinate*> bottom;
};

}  	// End namespace TestOAS
#endif
