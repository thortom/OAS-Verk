//
// C++ Interface: navigationSonar_model
//
// Description:
//
//
// Author: Teledyne Gavia ehf. <info@gavia.is>, (C) 2014
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef NAVIGATIONSONAR_MODEL_H
#define NAVIGATIONSONAR_MODEL_H

#include <math.h>
#include <vector>
#include "bottom_model_OAS.h"

namespace TestOAS{

class SonarBeam
{
public:
    SonarBeam();
    SonarBeam(const int lineNumb, const int totalLineCount, const double maxRange, const double beamWidth);
    ~SonarBeam();

    void setSonarBeam(const int lineNumb, const int totalLineCount, const double maxRange, const double beamWidth);
    line getLine(const coordinate auvPos);

private:
    double slope;
    double intersection;
    line beamLine;
    double lineAngle;
};


class NavigationModel
{
public:
    NavigationModel();
    NavigationModel(const int numbBeams, const double maxRange);
    ~NavigationModel();

    double getRange(const coordinate auvPosition, BottomModel& bottom);
    void configureSonarBeam(const int numbBeams, const double maxRange);

private:
    double range;
    double maxRange;
    double beamWidth;              // Units in radians
    int numbSonarBeams;
    SonarBeam* sonarBeams;

    // Code from -> http://www.softwareandfinance.com/Visual_CPP/VCPP_Intersection_Two_line_Segments_EndPoints.html
    int IsPointInBoundingBox(double x1, double y1, double x2, double y2, double px, double py);
    int LineIntersection(double l1x1, double l1y1, double l1x2, double l1y2,
       double l2x1, double l2y1, double l2x2, double l2y2);
    int LineSegmentIntersection(double l1x1, double l1y1, double l1x2, double l1y2,
       double l2x1, double l2y1, double l2x2, double l2y2,
       double* intersection_X, double* intersection_Y);
};

}
#endif
