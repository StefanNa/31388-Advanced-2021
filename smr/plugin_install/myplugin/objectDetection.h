
/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                             *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef OBJECT_DETECT_H
#define OBJECT_DETECT_H

#include <cstdlib>

//#include <ulms4/ufunclaserbase.h>
//#include <urob4/uresposehist.h>
#include <vector>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <math.h>

using namespace std;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class ObjectDetection {
public:
  
  vector<double> getLinebasic(double x1, double y1, double x2, double y2);

 
  double pointLineDis(double x, double y, vector<double> line);


  vector<double> lsqLine(vector<double> X, vector<double> Y);

  vector<vector<double>> ransac(vector<double> X, vector<double> Y,int maxlines ,int randcouples, float thresh, int minLineSup,int minNoPoints);

  vector<vector<double>> identification(vector<vector<double>> Lines);

};  

#endif