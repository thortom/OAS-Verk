//
// C++ Implementation: navigationSonar_model
//
// Description:
//
//
// Author: Teledyne Gavia ehf. <info@gavia.is>, (C) 2014
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <Numerics/Geometry/geometry.h>
#include <iostream>
#include <math.h>

#include "navigationSonar_model.h"

using namespace TestOAS;

namespace TestOAS{

SonarBeam::SonarBeam()
{
    // TODO
}

SonarBeam::SonarBeam(const int lineNumb, const int totalLineCount, const double maxRange, const double beamWidth)
{
    setSonarBeam(lineNumb, totalLineCount, maxRange, beamWidth);
}

SonarBeam::~SonarBeam()
{
    // TODO:
}

void SonarBeam::setSonarBeam(const int lineNumb, const int totalLineCount, const double maxRange, const double beamWidth)
{
    if (totalLineCount <= 1)
    {
        lineAngle = 0.0;
    }
    else
    {
        lineAngle = -beamWidth/(totalLineCount - 1.0)*(lineNumb - 1) + beamWidth/2.0;
    }
    beamLine.x1 = 0.0;
    beamLine.y1 = 0.0;
    beamLine.x2 = maxRange*cos(lineAngle);
    beamLine.y2 = -maxRange*sin(lineAngle);

    slope = (beamLine.y2 - beamLine.y1)/(beamLine.x2 - beamLine.x1);                        // Probably not needed
    intersection = beamLine.y2 - slope*beamLine.x2;
}

line SonarBeam::getLine(const coordinate auvPos)
{
    line auvBeam;
    auvBeam.x1 = auvPos.x + beamLine.x1;
    auvBeam.x2 = auvPos.x + beamLine.x2;
    auvBeam.y1 = auvPos.depth + beamLine.y1;
    auvBeam.y2 = auvPos.depth + beamLine.y2;
    return auvBeam;
}

NavigationModel::NavigationModel()
{
    int numbBeams = 4;
    maxRange = 50;
    configureSonarBeam(numbBeams, maxRange);
}

NavigationModel::NavigationModel(const int numbBeams, const double maxRange)
{
    configureSonarBeam(numbBeams, maxRange);
}

NavigationModel::~NavigationModel()
{
    // TODO:
}

double NavigationModel::getRange(const coordinate auvPosition, BottomModel& bottom)
{
    line sonarLine;
    line bottomLine;
    double minRange = maxRange;
    double currentRange = maxRange;
    double x = auvPosition.x;
    double intersectionX, intersectionY;

    for (int i = 0; i < numbSonarBeams; i++)
    {
        sonarLine = sonarBeams[i].getLine(auvPosition);
        while (x <= (auvPosition.x + maxRange))
        {

            bottomLine = bottom.getBottomLine(x);

            if (LineSegmentIntersection(sonarLine.x1, sonarLine.y1, sonarLine.x2, sonarLine.y2,
                    bottomLine.x1, bottomLine.y1, bottomLine.x2, bottomLine.y2, &intersectionX, &intersectionY))
            {
                currentRange = intersectionX - auvPosition.x;

                if (currentRange < minRange)
                {
                    minRange = currentRange;
                }
            }
            x++;
        }
        x = auvPosition.x;
    }
    return minRange;
}

void NavigationModel::configureSonarBeam(const int numbBeams, const double maxRange)
{
    beamWidth = 10.0 * Gavia::Numerics::Geometry::deg2rad;
    this->maxRange = maxRange;
    sonarBeams = new SonarBeam[numbBeams];
    numbSonarBeams = numbBeams;
    for (int i = 0; i < numbBeams; i++)
    {
        sonarBeams[i].setSonarBeam(i+1, numbBeams, maxRange, beamWidth);
    }
}

int NavigationModel::IsPointInBoundingBox(double x1, double y1, double x2, double y2, double px, double py)
{
    double left, top, right, bottom; // Bounding Box For Line Segment
    // For Bounding Box
    if(x1 < x2)
    {
        left = x1;
        right = x2;
    }
    else
    {
        left = x2;
        right = x1;
    }
    if(y1 < y2)
    {
        top = y2;
        bottom = y1;
    }
    else
    {
        top = y1;
        bottom = y2;
    }


    if( (px >= left) && (px <= right) && (py <= top) && (py >= bottom) )
    {
        return 1;
    }
    else
        return 0;
}

int NavigationModel::LineIntersection(double l1x1, double l1y1, double l1x2, double l1y2,
                                    double l2x1, double l2y1, double l2x2, double l2y2)
{

    double m1, c1, m2, c2;
    double intersection_X, intersection_Y;
    double dx, dy;

    dx = l1x2 - l1x1;
    dy = l1y2 - l1y1;

    m1 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c1 = l1y1 - m1 * l1x1; // which is same as y2 - slope * x2

    dx = l2x2 - l2x1;
    dy = l2y2 - l2y1;

    m2 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c2 = l2y1 - m2 * l2x1; // which is same as y2 - slope * x2

    if( (m1 - m2) == 0)
        return 0;
    else
    {
        intersection_X = (c2 - c1) / (m1 - m2);
        intersection_Y = m1 * intersection_X + c1;
        return 1;                                           // Added
    }
}

int NavigationModel::LineSegmentIntersection(double l1x1, double l1y1, double l1x2, double l1y2,
                                            double l2x1, double l2y1, double l2x2, double l2y2,
                                            double* intersection_X, double* intersection_Y)
{
    double m1, c1, m2, c2;
    double dx, dy;

    dx = l1x2 - l1x1;
    dy = l1y2 - l1y1;

    m1 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c1 = l1y1 - m1 * l1x1; // which is same as y2 - slope * x2

    dx = l2x2 - l2x1;
    dy = l2y2 - l2y1;

    m2 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c2 = l2y1 - m2 * l2x1; // which is same as y2 - slope * x2

    if( (m1 - m2) == 0)
    {
        //std::cout << "m1 - m2 == 0" << std::endl;
        return 0;
    }
    else
    {
        *intersection_X = (c2 - c1) / (m1 - m2);
        *intersection_Y = m1 * *intersection_X + c1;
    }
    if(IsPointInBoundingBox(l1x1, l1y1, l1x2, l1y2, *intersection_X, *intersection_Y) == 1 &&
        IsPointInBoundingBox(l2x1, l2y1, l2x2, l2y2, *intersection_X, *intersection_Y) == 1)
    {
        return 1;
    }
    else
        return 0;
}

} // End of namespace TestOAS
