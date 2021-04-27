/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
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
#include "boxDetect_final.h"
#include <vector>
using namespace std;

#ifdef LIBRARY_OPEN_NEEDED

vector<double> X;
vector<double> Y;

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunczoneobst' with your class name */
  return new UFuncboxFinder();
}
#endif


bool UFuncboxFinder::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
	bool result = true;

	if (resource->isA(UResPoseHist::getMapPoseID())) { // pointer to server the resource that this plugin can provide too
		// but as there might be more plugins that can provide the same resource
		// use the provided
		if (remove)
			// the resource is unloaded, so reference must be removed
			poseHist = NULL;
		else if (poseHist != (UResPoseHist *) resource)
			// resource is new or is moved, save the new reference
			poseHist = (UResPoseHist *) resource;
		else
			// reference is not used
			result = false;
	}
	// other resource types may be needed by base function.
	result = UFunctionBase::setResource(resource, remove);
	return result;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
// #define SMRWIDTH 0.4
bool UFuncboxFinder::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  const int MVL = 30;
  char value[MVL];
  ULaserData * data;
  
  // define variables
  double x_laser, y_laser, x_world, y_world, r, theta;
  int i;
  double offset = 0.26;
  
  // do some action and send a reply
  data = getScan(msg, (ULaserData*)extra);
  
  if (data->isValid()) {
    UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());
    // Gets the odometry pose at the time when the laserscan was taken, poseAtScan.x poseAtScan.y poseAtScan.a (x,y,angle) in reality poseAtScan.h
    // Convert from polar coordinates to cartesian coordinates

    for (i=0;i<data->getRangeCnt();i++){
      // Get the scan data (in polar coordinates)
      r=data->getRangeMetre(i);
      theta=data->getAngleRad(i);
      // Convert the scan data to cartesian coordinates (laser frame)
      x_laser=r*cos(theta);
      y_laser=r*sin(theta);
      // Convert the cartesian coordinates (laser frame) to cartesian coordinates world frame
      x_world = poseAtScan.x + offset*cos(poseAtScan.h) + (x_laser*cos(poseAtScan.h) - y_laser*sin(poseAtScan.h));
      y_world = poseAtScan.y + offset*sin(poseAtScan.h) + (x_laser*sin(poseAtScan.h) + y_laser*cos(poseAtScan.h));
      // keep measurement if it is inside the square
      if ((x_world >= 1.0 && x_world <= 3.0) && (y_world >= 1.0 && y_world <= 2.0)){
        X.push_back(x_world);
        Y.push_back(y_world);
      }
    }
    
    // send this string as the reply to the client
    sendMsg(msg, reply);

  }
  else
    sendWarning(msg, "No scandata available");

  // return true if the function is handled with a positive result
  // used if scanpush or push has a count of positive results
  return true;
}

void UFuncboxFinder::createBaseVar()
{ // add also a global variable (global on laser scanner server) with latest data
  var_zone = addVarA("zone", "0 0 0 0 0 0 0 0 0", "d", "Value of each laser zone. Updated by zoneobst.");
}