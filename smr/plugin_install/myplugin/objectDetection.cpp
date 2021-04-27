#include "objectDetection.h"

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