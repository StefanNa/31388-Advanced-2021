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
#include "ufuncrectanglefinder.h"

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunczoneobst' with your class name */
  return new UFuncrectanglefinder();
}
#endif

bool UFuncrectanglefinder::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
  bool result = true;

  if (resource->isA(UResPoseHist::getOdoPoseID())) { // pointer to server the resource that this plugin can provide too
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
bool UFuncrectanglefinder::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  const int MVL = 30;
  char value[MVL];
  ULaserData * data;
  //double robotwidth;
  //
  //double r,delta;
  //double minRange; // min range in meter
  // double minAngle = 0.0; // degrees
//   double d,robotwidth;
  //double zone[9];
  double pose[3];
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "rectanglefinder");
    sendText("--- available rectanglefinder options\n");
    sendText("help            This message\n");
    sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
    sendText("device=N        Laser device to use (see: SCANGET help)\n");
    sendText("see also: SCANGET and SCANSET\n");
    sendHelpDone();
  }
  else
  { // do some action and send a reply
    data = getScan(msg, (ULaserData*)extra);
    // 
    if (data->isValid())
    {
      int imax=data->getRangeCnt();
      //Convert laserscan data to cartesian coordinates
      double cart[MAX_RANGE_VALUES][2];
      double filtered[MAX_RANGE_VALUES][2];
      polar2crt(data, cart);
      //print("Polar 2 Cartesian coordinates\n");
      for(int i = 0; i < imax; i++){
        //printf("i: %i, x: %f, y:%f \n", i, cart[i][0], cart[i][1]);
      }
      imax = boxFilter(cart, MAX_RANGE_VALUES, 0.2, 1.2, -0.5, 0.5, filtered);
      //print("Filtered data\n");
      for(int i = 0; i < imax; i++){
        //printf("i: %i, x: %f, y:%f \n", i, filtered[i][0], filtered[i][1]);
      }

      double x[MAX_RANGE_VALUES];
      double y[MAX_RANGE_VALUES];

      getX(filtered, imax, x);
      getY(filtered, imax, y);

      double w[] = {0.25, 0.0, -0.5, 0.0, 0.25};
      double ddx[MAX_RANGE_VALUES];
      double ddy[MAX_RANGE_VALUES];

      conv(x, imax, w, 5, ddx);
      conv(y, imax, w, 5, ddy);

      double g[MAX_RANGE_VALUES];

      for (int i =0; i < imax; i++){
        g[i] = sqrt(ddx[i]*ddx[i] + ddy[i]*ddy[i]);
        //printf("i: %d, x:%.3f, y:%.3f, ddx:%.3f, ddy:%.3f, g:%.3f\n", i, x[i], y[i], ddx[i], ddy[i], g[i]);
      }
      
      int maxI = findMaxIndex(g, imax, 2);
      //printf("Corner located at: %d\n", maxI);
      double x1 = x[0];
      double y1 = y[0];
      double x2 = x[maxI];
      double y2 = y[maxI];
      double x3 = x[imax - 1];
      double y3 = y[imax - 1];

      double diag = sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1));
      if (diag > 0.32){ //Found corner
        //printf("Found corner of square\n");
        double size1 = sqrt(pow(x2-x1, 1) + pow(y2-y1, 2));
        double size2 = sqrt(pow(x3-x2, 2) + pow(y3-y2, 2));

        if (size1 > size2){
          //printf("Side 1 is longer than side 2\n");
          pose[2] = atan2(y1 - y2, x1 - x2);
        }else{
          //printf("Side 2 is longer than side 1\n");
          pose[2] = atan2(y3 - y2, x3 - x2);
        }
        pose[0] = x1 + 0.5*(x3-x1);
        pose[1] = y1 + 0.5*(y3-y1);
      }else { //Found one edge
        //printf("Found only one edge\n");
        double size1 = sqrt(pow(x1-x3,2) + pow(y1-y3,2));
        if (abs(size1 - 0.3) < abs(size1 - 0.2)){
          //printf("Found long edge\n");
          pose[2] = atan2(y1 - y3, x1 - x3);
          pose[0] = x1 + 0.5*(x3-x1) + cos(pose[2]+3.1415/2)*0.1;
          pose[1] = y1 + 0.5*(y3-y1) + sin(pose[2]+3.1415/2)*0.1;
        }else{
          //printf("Found short edge\n");
          pose[2] = atan2(y1 - y3,  x1 - x3) + 3.1415/2;
          pose[0] = x1 + 0.5*(x3-x1) + cos(pose[2])*0.15;
          pose[1] = y1 + 0.5*(y3-y1) + sin(pose[2])*0.15;
        }
      }
      /* SMRCL reply format */
      //snprintf(reply, MRL, "<laser l10=\"%g\" l11=\"%g\" l12=\"%g\" l13=\"%g\" l14=\"%g\" "
      //                            "l15=\"%g\" l16=\"%g\" l17=\"%g\" l18=\"%g\" />\n", 
      
      snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" />\n", 
                    pose[0],pose[1],pose[2]);
                           
      // send this string as the reply to the client
      sendMsg(msg, reply);
      // save also as gloabl variable
      
      for(int i = 0; i < 3; i++)
        var_rect->setValued(pose[i], i);
    }
    else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  // used if scanpush or push has a count of positive results
  return true;
}

void UFuncrectanglefinder::createBaseVar()
{ // add also a global variable (global on laser scanner server) with latest data
  var_rect = addVarA("rect", "0 0 0", "d", "Rectangle pose. Updated by findrectangle.");
}

void UFuncrectanglefinder::polar2crt(ULaserData * data, double result[][2]){
  int imax = data->getRangeCnt();
  for(int i =0; i < imax; i++){
    double r = data->getRangeMeter(i);
    double theta = data->getAngleRad(i);
    double x = r*cos(theta);
    double y = r*sin(theta);
    result[i][0] = x;
    result[i][1] = y;

    
  }
}

int UFuncrectanglefinder::boxFilter(double points[][2], int pointLength, double minX, double maxX, double minY, double maxY, double result[][2]){
  int count = 0;
  for (int i = 0; i < pointLength; i++){
    double x = points[i][0];
    double y = points[i][1];
    if(x <= maxX && x >= minX && y <=maxY && y >= minY){
      result[count][0] = x;
      result[count][1] = y;
      count++;
    }
  }
  return count;
}

int UFuncrectanglefinder::conv(double f[], int f_len, double w[], int w_len, double result[]){
  for(int i = 0; i < f_len; i++){
    double r = 0.0;
    for(int j = 0; j < w_len; j++){
      int m = j - (w_len-1)/2;
      if (i-m >=0 && i-m < f_len)
        r += f[i-m]*w[j];
    }
    result[i] = r;
  }
  return f_len;
}

void UFuncrectanglefinder::getX(double points[][2], int pointLength, double x[]){
  for(int i = 0; i < pointLength; i++){
    x[i] = points[i][0];
  }
}
void UFuncrectanglefinder::getY(double points[][2], int pointLength, double y[]){
  for(int i = 0; i < pointLength; i++){
    y[i] = points[i][1];
  }
}

int UFuncrectanglefinder::findMaxIndex(double values[], int length, int skip){
  double max = -10000.0;
  int maxIndex = -1;
  for (int i = skip; i < length - skip; i++)
  {
    if(values[i] > max){
      max = values[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}

// Ax+By+C=0
//line[0]x - line[1]y + line[2] = 0
vector<double> ObjectDetection::getLinebasic(double x1, double y1, double x2, double y2) {
  vector<double> line(3, 9999);

  if (x1 == x2 && y1 != y2) {
    line[0] = 1;
    line[1] = 0;
    line[2] = -x1;
    return line;
  }
  else if (x1 != x2 && y1 == y2) {
    line[0] = 0;
    line[1] = 1;
    line[2] = -y1;
    return line;
  }
  double A = (y1 - y2) / (x1 - x2);
  double C = y2 - A * x2;

  line[0] = A;
  line[1] = -1;
  line[2] = C;

  return line;
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
// Ax+By+C=0
// Distance=abs(A * x + B * y + C)/sqrt(A * A + B * B)
double ObjectDetection::pointLineDis(double x, double y, vector<double> line) {
  double distance = 0;
  double A = line[0];
  double B = line[1];
  double C = line[2];
  double numerator = fabs(A * x + B * y + C);
  double deniminator = sqrt(A * A + B * B);
  distance = numerator / deniminator;
  return distance;
}


// Ax+By+C=0
//line[0]x - line[1]y + line[2] = 0
vector<double> ObjectDetection::lsqLine(vector<double> X, vector<double> Y) {
  vector<double> lsqLine(3, 0);

  int n = X.size();
  double sumX = 0;
  double sumY = 0;
  double meanX = 0; 
  double meanY = 0;
  double covXY = 0; 
  double varX = 0; 
  double varY = 0;
  double A = 0; 
  double C = 0;

  for (int i = 0; i < n; i++) {
    sumX += X[i];
    sumY += Y[i];
  }
  meanX = sumX / n;
  meanY = sumY / n;
  for (int i = 0; i < n; i++) {
    covXY += (X[i] - meanX)*(Y[i] - meanY);
    varX += (X[i] - meanX) * (X[i] - meanX);
    varY += (Y[i] - meanY) * (Y[i] - meanY);
  }

  if (varX < 0.00001) { //the line is parallel to y axis
    lsqLine[0] = -1;
    lsqLine[1] = 0;
    lsqLine[2] = meanX;
  }
  else if (varY < 0.00001) {//the line is parallel to x axis
    lsqLine[0] = 0;
    lsqLine[1] = -1;
    lsqLine[2] = meanY;
  }
  else {
    A = covXY / varX;
    //C = - By_ - Ax_ --> Ax + By + C = 0 ; B =-1
    C = meanY - A * meanX;
    lsqLine[0] = A;
    lsqLine[1] = -1;
    lsqLine[2] = C;
  }
  return lsqLine;
}


/*
maxlines - The maximum number of line extraction iterations
randcouples - The number of random point couples to try before choosing the best candidate
thresh - The distance threshold determining whether a point is supporting a line
minLineSup - The minimum number of points to support a line for the line to be accepted
minNoPoints - The minimum number of points to continue line extraction iterations
*/
vector<vector<double>> ObjectDetection::ransac(vector<double> X, vector<double> Y,int maxlines ,int randcouples, float thresh, int minLineSup,int minNoPoints) {
  vector<double> inside_X = X;
  vector<double> inside_Y = Y;

  vector<vector<double>> Lines;

  int maxSupport=0;
  int nlines=0;
  int lineSupport = 0;
  // Continue while enough points are left and not too many lines are found
  while (inside_X.size() > minNoPoints && nlines < maxlines) {
    vector<double> candLine;
    vector<double> X_inliers;
  vector<double> Y_inliers;
    maxSupport=0;

    //try a certain amoiunt of random points
    for (int i = 0; i < randcouples; i++) {
      //get random points that are not identical
      int p1 ;
      int p2;
      p1 = rand() % inside_X.size();
      p2 = rand() % inside_X.size();
      while (p1 == p2) {
        p2 = rand() % inside_X.size();
      }
      double x1 = inside_X[p1];
      double y1 = inside_Y[p1];
      double x2 = inside_X[p2];
      double y2 = inside_Y[p2];
      //get simple line from 2 points
      candLine = getLinebasic(x1, y1, x2, y2);

      int lineSupport = 0;
      vector<double> X_pre;
      vector<double> Y_pre;
      // check distance of each point to line
      for (int i = 0; i < inside_X.size(); i++) {
        double dis = pointLineDis(inside_X[i], inside_Y[i], candLine);
        if (dis < thresh) {
          X_pre.push_back(inside_X[i]);
          Y_pre.push_back(inside_Y[i]);
          lineSupport++;
        }
      }
      // if more inliers than last round, update inliers
      if (lineSupport > maxSupport) {
        maxSupport = lineSupport;
        X_inliers = X_pre;
        Y_inliers = Y_pre;
      }

  }
  // after random repitition if enough inliers, get lsqline
  if (lineSupport > minLineSup) {
    candLine = lsqLine(X_inliers, Y_inliers);
    Lines.push_back(candLine);
    nlines=nlines+1;

    //remove used points from pool for next line
    for (int i = 0; i < X_inliers.size(); i++) {
      for (int j = 0; j < inside_X.size(); j++) {
        if (X_inliers[i] == inside_X[j] && Y_inliers[i] == inside_Y[j]) {
          inside_X.erase(inside_X.begin() + j);
          inside_Y.erase(inside_Y.begin() + j);
          break;
        }
      }
    }

  }
  
}
return Lines;
}