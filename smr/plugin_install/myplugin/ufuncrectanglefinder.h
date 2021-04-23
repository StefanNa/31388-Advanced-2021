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
#ifndef UFUNC_MRCOBST_H
#define UFUNC_MRCOBST_H

#include <cstdlib>

#include <ulms4/ufunclaserbase.h>
#include <urob4/uresposehist.h>
#include<vector>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

using namespace std;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class UFuncrectanglefinder : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFuncrectanglefinder()
  { // set the command (or commands) handled by this plugin
    setCommand("findrectangle", "findrectangleif", "rectangle detect for MRC (Compiled " __DATE__ " " __TIME__ ")");
    createBaseVar();
  }
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

  void polar2crt(ULaserData * data, double result[][2]);
  int boxFilter(double points[][2], int pointLength, double minX, double maxX, double minY, double maxY, double result[][2]);
  int conv(double f[], int f_len, double w[], int w_len, double result[]);
  void getX(double points[][2], int pointLength, double result[]);
  void getY(double points[][2], int pointLength, double result[]);

  int findMaxIndex(double values[], int length, int skip = 0);
  
  protected:
    void createBaseVar();
    UVariable *var_rect;
    UResPoseHist * poseHist;
};

class ObjectDetection {
public:
  
  vector<double> getLinebasic(double x1, double y1, double x2, double y2);

 
  double pointLineDis(double x, double y, vector<double> line);


  vector<double> lsqLine(vector<double> X, vector<double> Y);

  vector<vector<double>> ransac(vector<double> X, vector<double> Y,int maxlines ,int randcouples, float thresh, int minLineSup,int minNoPoints);


};




#endif

