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
  return fabs(cos(line[0])*x+sin(line[0])*y -line[1]);
  
}


// Ax+By+C=0
//line[0]x - line[1]y + line[2] = 0
vector<double> ObjectDetection::lsqLine(vector<double> X, vector<double> Y) {
  vector<double> lsqLine(2, 0);

  int n = X.size();
  double sumX = 0;
  double sumY = 0;
  double sumX2 = 0;
  double sumY2 = 0;
  double sumXY = 0;
  double meanX = 0; 
  double meanY = 0;
  double covXY = 0; 
  double varX = 0; 
  double varY = 0;
  double A = 0; 
  double C = 0;
  double alpha;
  double r;

  for (int i = 0; i < n; i++) {
    sumX += X[i];
    sumY += Y[i];
    sumX2 += X[i]*X[i];
    sumY2 += Y[i]*Y[i];
    sumXY += X[i]*Y[i];

  }

alpha=0.5*atan2(2*sumX*sumY-2*n*sumXY,sumX*sumX-sumY*sumY-n*sumX2+n*sumY2);


  meanX = sumX / double(n);
  meanY = sumY / double(n);

  r=meanX*cos(alpha)+meanY*sin(alpha);

  if (r < 0){
    r=r*-1;
    if(alpha<0){
            alpha = alpha + 3.141593;}
    else{
            alpha = alpha - 3.141593;}
    }
lsqLine[0]=alpha;
lsqLine[1]=r;

  // for (int i = 0; i < n; i++) {
  //   covXY += (X[i] - meanX)*(Y[i] - meanY)/double(n);
  //   varX += (X[i] - meanX) * (X[i] - meanX)/double(n);
  //   varY += (Y[i] - meanY) * (Y[i] - meanY)/double(n);
  // }

  // if (varX ==0) { //the line is parallel to y axis
  //   lsqLine[0] = -1;
  //   lsqLine[1] = 0;
  //   lsqLine[2] = meanX;
  // }
  // else if (varY ==0) {//the line is parallel to x axis
  //   lsqLine[0] = 0;
  //   lsqLine[1] = -1;
  //   lsqLine[2] = meanY;
  // }
  // else {
  //   A = covXY / varX;
  //   //C = - By_ - Ax_ --> Ax + By + C = 0 ; B =-1
  //   C = meanY - A * meanX;
  //   lsqLine[0] = A;
  //   lsqLine[1] = -1;
  //   lsqLine[2] = C;
  // }
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
  
  cout << "X_points=";
  cout  << "[";
  for (int p = 0; p < inside_X.size(); p++) {
    cout <<inside_X[p] << ", ";
  }
  cout << "]"<< "\n";

  cout << "Y_points=";
  cout  << "[";
  for (int p = 0; p < inside_X.size(); p++) {
    cout <<inside_Y[p] << ", ";
  }
  cout << "]"<< "\n";

  // cout << "X points:\n" << inside_X << "\n";
  // cout << "Y points:\n" << inside_Y << "\n";

  cout << "all points in calculation: " << X.size() << "\n\n";
  // Continue while enough points are left and not too many lines are found
  // while (inside_X.size() > minNoPoints && nlines < maxlines) {
  for (int line_ = 0; line_ < maxlines; line_++) {
    if (inside_X.size() < minNoPoints){
      break;
    }
    
    // cout << "num points: " << inside_X.size() << "\n";
    
  // for (int nlin = 0; nlin < maxlines; nlin++) {
    // if (nlines <= maxlines) {
    vector<double> candLine;
    vector<double> X_inliers;
    vector<double> Y_inliers;
    
    maxSupport=0;

    //try a certain amoiunt of random points
    for (int a = 0; a < randcouples; a++) {
      //get random points that are not identical
      // cout << "in for randcouples " << a << "\n";
      int p1 ;
      int p2;
      p1 = rand() % inside_X.size();
      p2 = rand() % inside_X.size();
      while (p1 == p2) {
        p2 = rand() % inside_X.size();
        // cout << "in while p1p2 " << p1 << " "<< p2 << "\n"; 
      }
      double x1 = inside_X[p1];
      double y1 = inside_Y[p1];
      double x2 = inside_X[p2];
      double y2 = inside_Y[p2];
      //get simple line from 2 points
      vector<double> X_first(2, 0);
      vector<double> Y_first(2, 0);
      X_first[0]=x1;
      X_first[1]=x2;
      Y_first[0]=y1;
      Y_first[1]=y2;
    
      candLine = lsqLine(X_first, Y_first);

      int lineSupport = 0;
      vector<double> X_pre;
      vector<double> Y_pre;
      // check distance of each point to line
      for (int i = 0; i < inside_X.size(); i++) {
        double dis = pointLineDis(inside_X[i], inside_Y[i], candLine);
        if (dis < thresh) {
          // cout << "push";
          // cout << "dis < thresh " << dis << " "<< thresh << "\n";
          X_pre.push_back(inside_X[i]);
          Y_pre.push_back(inside_Y[i]);
          lineSupport++;
          //cout << lineSupport << "\n";
        }
      }
      // if more inliers than last round, update inliers
      if (lineSupport > maxSupport) {
        // cout << "support";
        maxSupport = lineSupport;
        X_inliers = X_pre;
        Y_inliers = Y_pre;
      }


  }

  // after random repitition if enough inliers, get lsqline
  

  if (maxSupport > minLineSup) {
    cout << "support: " << maxSupport <<" minLineSup: " << minLineSup << "\n";
    int oldSupport=0;
    do{
      oldSupport = lineSupport;
      candLine = lsqLine(X_inliers, Y_inliers);
      
      vector<double> X_pre;
      vector<double> Y_pre;
      int lineSupport=0;
      for (int i = 0; i < inside_X.size(); i++) {
        double dis = pointLineDis(inside_X[i], inside_Y[i], candLine);
        if (dis < thresh) {
          // cout << "push";
          // cout << "dis < thresh " << dis << " "<< thresh << "\n";
          X_pre.push_back(inside_X[i]);
          Y_pre.push_back(inside_Y[i]);
          lineSupport++;
          //cout << lineSupport << "\n";
        }
      }
      X_inliers = X_pre;
      Y_inliers = Y_pre;
      candLine = lsqLine(X_inliers, Y_inliers);
    }while(lineSupport != oldSupport);
    Lines.push_back(candLine);
    nlines=nlines+1;

    //remove used points from pool for next line
    for (int i_ = 0; i_ < X_inliers.size(); i_++) {
      for (int j = 0; j < inside_X.size(); j++) {
        //cout << "for for remove " << i_ << " " << j << "\n";
        if (X_inliers[i_] == inside_X[j] && Y_inliers[i_] == inside_Y[j]) {
          inside_X.erase(inside_X.begin() + j);
          inside_Y.erase(inside_Y.begin() + j);
          break;
        }
      }
    }

  }
  
}
cout <<"\n\n\n\n"<<"found " << nlines << "lines" << "\n\n\n\n";
return Lines;
}


vector<double> ObjectDetection::identification(vector<vector<double>> Lines) {
int numBoxLines = Lines.size(); //size(box_lines,2);
int objectType=0;
vector<vector<double>> Lines_in=Lines;
vector<vector<double>> intersectPoints;
double x0=0;
double y0=0;

// intersectPoints = zeros(2,numBoxLines);
int cornerCount = 1;
for (int idx = 0; idx < numBoxLines; idx++) {
    for (int jdx = idx+1; jdx < numBoxLines; jdx++) {
        double r1=Lines_in[idx][1];
        double a1=Lines_in[idx][0];

        double r2=Lines_in[jdx][1];
        double a2=Lines_in[jdx][0];
        y0=(r1*cos(a2)-r2*cos(a1))/(sin(a1)*cos(a2)-sin(a2)*cos(a1));
        x0=(r1-y0*sin(a1))/cos(a1);

        vector<double> x0y0;
        x0y0.push_back(x0);
        x0y0.push_back(y0);
        if(x0y0[0] > 0.8 && x0y0[0] < 3.2 && x0y0[1] > 0.8 && x0y0[1] < 2.2){
            intersectPoints.push_back(x0y0);
        }

    }
  }
cornerCount=intersectPoints.size();

// pose = mean(intersectPoints,2);
// c1 = intersectPoints(:,1);
// c2s = intersectPoints(:,2:end);
// dists = sqrt(sum((c2s-c1).^2));
// [sortedDists,distIndex] = sort(dists);
// c2 = c2s(:,distIndex(2));
// side = c2 - c1;
// pose(3) = atan2(side(2),side(1));

vector<double> pose;
vector<double> dists;
double dist=0;
double distmax=0;
double x=0;
double y=0;

vector<double> c1={intersectPoints[0][0],intersectPoints[0][1]};
vector<double> c2(2,0);
if(cornerCount == 4){
  //mean
  double minDist=1000;
  int minDistidx=0;
for (int id = 0; id < cornerCount; id++) {
    x=x+intersectPoints[id][0];
    y=y+intersectPoints[id][1];
    if (id !=0){
      dist=sqrt(pow(c1[0]-intersectPoints[id][0],2) +pow(c1[1]-intersectPoints[id][1],2));
      if (dist<minDist){
        minDist=dist;
        minDistidx=id;
      }
    }
}


pose.push_back(x/cornerCount);
pose.push_back(y/cornerCount);


c2=intersectPoints.at(minDistidx);
vector<double> side = {c2[0] - c1[0],c2[1] - c1[1]};
double pi=3.141593;
pose.push_back(atan2(side[1],side[0])-(pi/2));
if (pose[2] > pi/2)
        {pose[2] = pose[2] - pi;}
if (pose[2] < -pi/2)
        {pose[2] = pose[2] + pi;}
if (minDist > 0.175)
        {objectType = 2;}
    else
        {objectType = 1;}
// intersectPoints.push_back(pose);



}
else if (cornerCount == 3){
double minCorner=1000;
int minCornerid=1;
int sideIdx;
double angle;
double Xcoord;
double Ycoord;

for (int id = 0; id < cornerCount; id++) {
    vector<double> c1={intersectPoints[id][0],intersectPoints[id][1]};
    vector<vector<double>> sides;
    for (int id2 = 0; id2 < cornerCount; id2++) {

      if (id2 !=id){
        sides.push_back({intersectPoints[id2][0]-c1[0],intersectPoints[id2][1]-c1[1]});
    }

    }
    double corner=fabs(sides[0][0]*sides[1][0]+sides[0][1]*sides[1][1]);
    if (corner < minCorner){
      minCorner = corner;
      minCornerid = id;
      double dist1= sqrt(pow(sides[0][0],2)+pow(sides[0][1],2));
      double dist2= sqrt(pow(sides[1][0],2)+pow(sides[1][1],2));
      if (dist1>dist2){
        sideIdx=0;
        if (dist1>0.35){
          objectType = 3;
        }
        else{
          objectType = 4;
        }
      }
      else{
        sideIdx=1;
        if (dist2>0.35){
          objectType = 3;
        }
        else{
          objectType = 4;
        }
      }
      angle=atan2(sides[sideIdx][1],sides[sideIdx][0]);
      Xcoord=c1[0];
      Ycoord=c1[1];

    }

}
pose.push_back(Xcoord);
pose.push_back(Ycoord);
pose.push_back(angle);

}
pose.push_back(objectType);


return pose;
}

int ObjectDetection::getIndex(vector<double> v, int K){
  int index=0;
    auto it = find(v.begin(), v.end(), K);
 
    // If element was found
    if (it != v.end())
    {
     
        // calculating the index
        // of K
        int index = it - v.begin();
        
    }
    else {
        // If the element is not
        // present in the vector
        int index=-1;
    }
    return index;
}