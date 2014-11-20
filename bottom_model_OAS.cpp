//
// C++ Implementation: bottom_model
//
// Description: 
//
//
// Author: Teledyne Gavia ehf. <info@gavia.is>, (C) 2014
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "bottom_model_OAS.h"
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <cmath>
#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

using namespace TestOAS;

namespace TestOAS{

BottomModel::~BottomModel()
{
  while(!bottom.empty())
  {
    delete bottom.back(), bottom.pop_back();
  }
}

void
BottomModel::readFile(const char* filename)
{
  std::ifstream file;
  file.open (filename);
  if (file.fail())
  {
    std::cout << "can't open file " << filename << "\n";
    std::exit(1);
  }

  while (!file.eof())
  {
    coordinate* new_coordinate = new coordinate();
	file >> new_coordinate->x;
	file >> new_coordinate->depth;
	bottom.push_back( new_coordinate );
	//std::cout << "x: " << new_coordinate->x << "depth" << new_coordinate->depth << std::endl;
  }
  bottom.pop_back();
  file.close();
}

double
BottomModel::getAltitude(double x, double z)
{
  double ret = 0;
  double currentDepth;
  std::vector<coordinate*>::iterator it = bottom.begin();
  while (it != bottom.end() & ((*it)->x <= x))
  {																													// TODO: verify if correct
	if ((it+1) == bottom.end())
	{
		ret = (*it)->depth - z;
		break;
	}
	currentDepth = (*it)->depth + (x - (*it)->x)/((*(it+1))->x - (*it)->x)*((*(it+1))->depth - (*it)->depth); 		// Linear interpolation
    ret = currentDepth - z;
    it++;
  }
  return ret;
}

line
BottomModel::getBottomLine(double x)
{
  line ret; 	// {x, x+1, 0.0, 0.0};
  ret.x1 = x;
  ret.x2 = x+1;
  ret.y1 = 0.0;
  ret.y2 = 0.0;

  std::vector<coordinate*>::iterator it = bottom.begin();
  while (it != bottom.end() && (*it)->x < (bottom.size() - 1) && ((*it)->x <= x))
  {
    ret.y1 = (*it)->depth;
    it++;
  }
  ret.y2 = (*it)->depth;
  return ret;
}

} // End of namespace TestOAS
