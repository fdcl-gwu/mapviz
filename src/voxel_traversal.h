// C/C++ includes
#include <cfloat>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>

//Eigen includes
#include <eigen3/Eigen/Core>
/**
 * @brief returns all the voxels that are traversed by a ray going from start to end
 * @param start : continous world position where the ray starts
 * @param end   : continous world position where the ray end
 * @return vector of voxel ids hit by the ray in temporal order
 *
 * J. Amanatides, A. Woo. A Fast Voxel Traversal Algorithm for Ray Tracing. Eurographics '87
 */

template <class NumType>
NumType sgn(NumType x){
  return (x > 0) - (x < 0);
}

using namespace std;
 class Ray_base{
 private:
     double _bin_size;
     double mapMinX, mapMinY, mapMinZ;
     double mapMaxX, mapMaxY, mapMaxZ;
     int numCellsX, numCellsY, numCellsZ;
     int n;
     vector<double> mapEstRay, sensorRanges, sensorLocWFVector, unitVectorWFVector;
     vector<unsigned> cellIndices;
     double minRange = 0;
     double maxRange;
     // Temporary Variables used with Ray Casting
     vector<unsigned> xCellIndices, yCellIndices, zCellIndices;
     vector<double> xCellDepths, yCellDepths, zCellDepths;
     std::vector<Eigen::Vector3i> visited_voxels;
     Eigen::Vector3d ray;
     Eigen::Vector3i last_voxel, current_voxel;
     Eigen::Vector3i diff = {0,0,0};
     double stepX, stepY, stepZ;
     double next_voxel_boundary_x, next_voxel_boundary_y, next_voxel_boundary_z;
     double tMaxX,tMaxY,tMaxZ;
     double tDeltaX,tDeltaY,tDeltaZ;
     Eigen::Vector3d ray_start, ray_end;
 public:
     Ray_base(double x_min, double y_min, double z_min,
            double L_x, double L_y, double L_z, double voxel_size){
         this->mapMinX = x_min;
         this->mapMinY = y_min;
         this->mapMinZ = z_min;

         this->mapMaxX = x_min + L_x;
         this->mapMaxY = y_min + L_y;
         this->mapMaxZ = z_min + L_z;
         this->_bin_size = voxel_size;

         this->numCellsX = floor(10 / _bin_size)+1;
         this->numCellsY = floor(10 / _bin_size)+1;
         this->numCellsZ = floor(10 / _bin_size)+1;
         this->n = numCellsX*numCellsY*numCellsZ;


         current_voxel = Eigen::Vector3i::Zero();
         last_voxel = Eigen::Vector3i::Zero();
         sensorLocWFVector.reserve(3);
         unitVectorWFVector.reserve(3);
        //  sensorLocWFVector[0] = 0;
        //  sensorLocWFVector[1] = 0;
        //  sensorLocWFVector[2] = 0;
        //  unitVectorWFVector[0] = -1/sqrt(3);
        //  unitVectorWFVector[1] = 1/sqrt(3);
        //  unitVectorWFVector[2] = 1/sqrt(3);
         int maxNumCellsAlongRay = numCellsX+numCellsY+numCellsZ;
         sensorRanges.reserve(maxNumCellsAlongRay);
          cellIndices.resize(maxNumCellsAlongRay);
          xCellIndices.resize(numCellsX);
          yCellIndices.resize(numCellsY);
          zCellIndices.resize(numCellsZ);
          xCellDepths.resize(numCellsX);
          yCellDepths.resize(numCellsY);
          zCellDepths.resize(numCellsZ);
          visited_voxels.reserve(maxNumCellsAlongRay);

         cout<<"constructor starting: numcells: "<<numCellsX<<"x"<<numCellsY<<"x"<<numCellsZ<<" : "<< this->n <<
             " max cell ray:"<<maxNumCellsAlongRay<< endl;

     }
     inline double getBinSize(){return _bin_size;}
     int voxel_traversal();
     std::vector<double> MapLocationFromInd(int& i);
     int IndFromMapLocation(vector<double>& mapLoc);
     Eigen::Vector3i IndToSub(int& i);
     unsigned SingleRayCasting3D();
     vector<unsigned>& getIndices(){return cellIndices;};
     inline std::vector<Eigen::Vector3i> getVisited(){return visited_voxels;};
     inline void setStartEnd(Eigen::Vector3d ray_start, Eigen::Vector3d ray_end){
         this->ray_start = ray_start;
         this->ray_end = ray_end;

         sensorLocWFVector[0] = ray_start[0];
         sensorLocWFVector[1] = ray_start[1];
         sensorLocWFVector[2] = ray_start[2];
         auto norm_vec =  (ray_end - ray_start);
         this->maxRange = norm_vec.norm();
        //  cout<<maxRange<<" norm of vec"<<endl;
         auto norm_vec2 = norm_vec/maxRange;
        //  cout<<"vec norm: "<< norm_vec2<<endl;
         unitVectorWFVector[0] = norm_vec2[0];
         unitVectorWFVector[1] = norm_vec2[1];
         unitVectorWFVector[2] = norm_vec2[2];


     }
 };

int Ray_base::voxel_traversal() {
    visited_voxels.clear();
    int vox_index = 0;

  // This id of the first/current voxel hit by the ray.
  // Using floor (round down) is actually very important,
  // the implicit int-casting will round up for negative numbers.
  current_voxel[0] = std::floor(ray_start[0]/_bin_size);
  current_voxel[1] = std::floor(ray_start[1]/_bin_size);
  current_voxel[1] = std::floor(ray_start[2]/_bin_size);

  // The id of the last voxel hit by the ray.
  // TODO: what happens if the end point is on a border?
  last_voxel[0] =std::floor(ray_end[0]/_bin_size);
  last_voxel[1]  = std::floor(ray_end[1]/_bin_size);
  last_voxel[2] = std::floor(ray_end[2]/_bin_size);

  // Compute normalized ray direction.
  //Eigen::Vector3d
  ray = ray_end-ray_start;
  //ray.normalize();

  // In which direction the voxel ids are incremented.
  stepX = (ray[0] >= 0) ? 1:-1; // correct
  stepY = (ray[1] >= 0) ? 1:-1; // correct
  stepZ = (ray[2] >= 0) ? 1:-1; // correct

  // Distance along the ray to the next voxel border from the current position (tMaxX, tMaxY, tMaxZ).
  next_voxel_boundary_x = (current_voxel[0]+stepX)*_bin_size; // correct
  next_voxel_boundary_y = (current_voxel[1]+stepY)*_bin_size; // correct
  next_voxel_boundary_z = (current_voxel[2]+stepZ)*_bin_size; // correct

  // tMaxX, tMaxY, tMaxZ -- distance until next intersection with voxel-border
  // the value of t at which the ray crosses the first vertical voxel boundary
  tMaxX = (ray[0]!=0) ? (next_voxel_boundary_x - ray_start[0])/ray[0] : DBL_MAX; //
  tMaxY = (ray[1]!=0) ? (next_voxel_boundary_y - ray_start[1])/ray[1] : DBL_MAX; //
  tMaxZ = (ray[2]!=0) ? (next_voxel_boundary_z - ray_start[2])/ray[2] : DBL_MAX; //

  // tDeltaX, tDeltaY, tDeltaZ --
  // how far along the ray we must move for the horizontal component to equal the width of a voxel
  // the direction in which we traverse the grid
  // can only be FLT_MAX if we never go in that direction
  tDeltaX = (ray[0]!=0) ? _bin_size/ray[0]*stepX : DBL_MAX;
  tDeltaY = (ray[1]!=0) ? _bin_size/ray[1]*stepY : DBL_MAX;
  tDeltaZ = (ray[2]!=0) ? _bin_size/ray[2]*stepZ : DBL_MAX;


  bool neg_ray=false;
  if (current_voxel[0]!=last_voxel[0] && ray[0]<0) { diff[0]--; neg_ray=true; }
  if (current_voxel[1]!=last_voxel[1] && ray[1]<0) { diff[1]--; neg_ray=true; }
  if (current_voxel[2]!=last_voxel[2] && ray[2]<0) { diff[2]--; neg_ray=true; }
  visited_voxels.push_back(current_voxel);
  if (neg_ray) {
    current_voxel+=diff;
    visited_voxels.push_back(current_voxel);
  }

  while(last_voxel != current_voxel) {
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        current_voxel[0] += stepX;
        tMaxX += tDeltaX;
      } else {
        current_voxel[2] += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        current_voxel[1] += stepY;
        tMaxY += tDeltaY;
      } else {
        current_voxel[2] += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    visited_voxels.push_back(current_voxel);
    vox_index++;
  }
  return vox_index;
}


unsigned Ray_base::SingleRayCasting3D(){
    // Given a robot pose and direction of a measurement ray,
    // this function returns the indices of the grid cells
    // according to 'ProbabilisticOccupancyGridMapping::IndFromMapLocation()' & 'MapLocationFromInd()'
    // and the depths of those cells from the robot (all geometry).

  unsigned numCellsAlongRay;
  vector<double> rayMaxLoc(3);// final ray location within sensor range

  for(unsigned i = 0; i < 3; i++)
  {
    rayMaxLoc[i] = sensorLocWFVector[i]+unitVectorWFVector[i]*maxRange;
   // cout<<"maxRange in ray: "<<maxRange<<'\n';
    }
  double correctionRatio(1.0), checkCorrectionRatio;
  if(     rayMaxLoc[0] < mapMinX){
    checkCorrectionRatio = (mapMinX-sensorLocWFVector[0])/(rayMaxLoc[0]-sensorLocWFVector[0]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }
  else if(rayMaxLoc[0] > mapMaxX){
    checkCorrectionRatio = (mapMaxX-sensorLocWFVector[0])/(rayMaxLoc[0]-sensorLocWFVector[0]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }
  if(     rayMaxLoc[1] < mapMinY){
    checkCorrectionRatio = (mapMinY-sensorLocWFVector[1])/(rayMaxLoc[1]-sensorLocWFVector[1]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }
  else if(rayMaxLoc[1] > mapMaxY){
    checkCorrectionRatio = (mapMaxY-sensorLocWFVector[1])/(rayMaxLoc[1]-sensorLocWFVector[1]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }
  if(     rayMaxLoc[2] < mapMinZ){
    checkCorrectionRatio = (mapMinZ-sensorLocWFVector[2])/(rayMaxLoc[2]-sensorLocWFVector[2]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }
  else if(rayMaxLoc[2] > mapMaxZ){
    checkCorrectionRatio = (mapMaxZ-sensorLocWFVector[2])/(rayMaxLoc[2]-sensorLocWFVector[2]);
    if(checkCorrectionRatio < correctionRatio)
      correctionRatio = checkCorrectionRatio;
  }



  for(unsigned i = 0; i < 3; i++)
  {
    rayMaxLoc[i] = sensorLocWFVector[i]+correctionRatio*unitVectorWFVector[i]*maxRange;

}

  int startingInd = IndFromMapLocation(sensorLocWFVector);

  vector<double> snappedStartingLoc =
    MapLocationFromInd(startingInd);

  int endingInd = IndFromMapLocation(rayMaxLoc);

  vector<double> snappedEndingLocation =
    MapLocationFromInd(endingInd);

  unsigned numXCellsAlongRay = floor(abs((snappedEndingLocation[0]-snappedStartingLoc[0])/_bin_size)+0.5);
  unsigned numYCellsAlongRay = floor(abs((snappedEndingLocation[1]-snappedStartingLoc[1])/_bin_size)+0.5);
  unsigned numZCellsAlongRay = floor(abs((snappedEndingLocation[2]-snappedStartingLoc[2])/_bin_size)+0.5);


  unsigned numCellsAlongRayXYZ[] = {numXCellsAlongRay, numYCellsAlongRay, numZCellsAlongRay};
  double dist, minDenomMag(0.000001);// unit vector minimum magnitude component for consideration (TODO: make param)

  double maxRangeRay = correctionRatio*maxRange;
  // cout<<"after multi maxRangeRay: "<<maxRangeRay <<" correctionRatio: "<< correctionRatio<<endl;
  // Ray Casting
  unsigned rayIndices[3] = {0,0,0};
  vector<double> edgeLoc(3), mapLoc(3), slopes(3);

  for(unsigned iEdges = 0; iEdges < 3; iEdges++){// evaluate {x,y,z}-edges

    rayIndices[iEdges] = 0;
    if(abs(unitVectorWFVector[iEdges]) >= minDenomMag){
      double increment = sgn(unitVectorWFVector[iEdges])*_bin_size;// increment size & direction
      edgeLoc[iEdges] = snappedStartingLoc[iEdges]+0.5*increment;// first cell edge

      for(unsigned i = 0; i < 3; i++){
        slopes[i]  = unitVectorWFVector[i]/unitVectorWFVector[iEdges];// 0 <= slopes[i] < inf, slopes[i] = 1 if i == iEdges
        edgeLoc[i] = slopes[i]*(edgeLoc[iEdges]-sensorLocWFVector[iEdges])+sensorLocWFVector[i];// when i == iEdges, edgeLoc[i] is unchanged
      }

      for(unsigned k = 0; k < numCellsAlongRayXYZ[iEdges]; k++){

        // Intersection & distance to this point
        double sqrDist(0.0);
        for(unsigned i = 0; i < 3; i++){
          edgeLoc[i] += slopes[i]*increment;
          sqrDist += pow(edgeLoc[i]-sensorLocWFVector[i],2);
        }
        dist = sqrt(sqrDist);
        // cout<<"dist: "<<dist<<" maxrange: "<<maxRangeRay<<endl;

        if(dist >= minRange){

          if(dist > maxRangeRay)
            break;
          else{
            mapLoc = edgeLoc;
            mapLoc[iEdges] += 0.5*increment;// move from edge to center for intersecting edge

            if(iEdges == 0){
                // cout<<mapLoc[0]<<endl;
              xCellIndices[rayIndices[iEdges]] = IndFromMapLocation(mapLoc);
              xCellDepths [rayIndices[iEdges]] = dist;
            }
            if(iEdges == 1){
              yCellIndices[rayIndices[iEdges]] = IndFromMapLocation(mapLoc);
              yCellDepths [rayIndices[iEdges]] = dist;
            }
            if(iEdges == 2){
              zCellIndices[rayIndices[iEdges]] = IndFromMapLocation(mapLoc);
              zCellDepths [rayIndices[iEdges]] = dist;
            }

            rayIndices[iEdges] += 1;
          }
        }
      }
    }
  }
 // cout<<"test: "<<rayIndices[1]<<endl;

  numXCellsAlongRay = rayIndices[0];
  numYCellsAlongRay = rayIndices[1];
  numZCellsAlongRay = rayIndices[2];
  numCellsAlongRay = numXCellsAlongRay+numYCellsAlongRay+numZCellsAlongRay;

//  fill(cellIndices.begin(), cellIndices.end(), 0);
//  fill(cellDepths.begin(), cellDepths.end(), 1.0);

  unsigned iX(0), iY(0), iZ(0);// index of saved variables

  for(unsigned k = 0; k < numCellsAlongRay; k++){

    unsigned closestEdge(3);// 3 does not correspond to anything
    // Determine if closest intersection is on an x, y, or z edge

    // Start assuming x is closest or at max case, then check and correct
    if(iX < numXCellsAlongRay){
        dist = xCellDepths[iX];
        closestEdge = 0;
    }
    else
      dist = maxRange;
    if(iY < numYCellsAlongRay){
      if(dist > yCellDepths[iY]){
        dist = yCellDepths[iY];
        closestEdge = 1;
      }
    }
    if(iZ < numZCellsAlongRay){
      if(dist > zCellDepths[iZ]){
        dist = zCellDepths[iZ];
        closestEdge = 2;
      }
    }

    // Include closest intersection as next element
    if(     closestEdge == 0){
      cellIndices[k] = xCellIndices[iX];
      sensorRanges [k] = xCellDepths [iX];
      iX++;
    }
    else if(closestEdge == 1){
      cellIndices[k] = yCellIndices[iY];
      sensorRanges [k] = yCellDepths [iY];
      iY++;
    }
    else if(closestEdge == 2){
      cellIndices[k] = zCellIndices[iZ];
      sensorRanges [k] = zCellDepths [iZ];
      iZ++;
    }
  }


  return numCellsAlongRay;
}

int Ray_base::IndFromMapLocation(vector<double>& mapLoc){

    // Location on Map
    double x = mapLoc[0];
    double y = mapLoc[1];
    double z = mapLoc[2];


    if(((x > mapMinX-_bin_size/2) && (x < mapMaxX+_bin_size/2)) ||
        (y > mapMinY-_bin_size/2 && y < mapMaxY+_bin_size/2) ||
        (z > mapMinZ-_bin_size/2 && z < mapMaxZ+_bin_size/2)   )
        {

       // i changes in order of z-y-x [*]change order to x-y-z
    //    int i =   floor((x-mapMinX)/_bin_size+0.5)
    //            + numCellsX*floor((y-mapMinY)/_bin_size+0.5)
    //            + numCellsX*numCellsY*floor((z-mapMinZ)/_bin_size+0.5);

      int i = floor((z-mapMinZ)/_bin_size+0.5)
              + numCellsZ*floor((y-mapMinY)/_bin_size+0.5)
              + numCellsZ*numCellsY*floor((x-mapMinX)/_bin_size+0.5);

      if(i > -1 && i < n)
        return i;
      else
        return -1;
    }
    else
      return -1;
}

vector<double> Ray_base::MapLocationFromInd(int& i){

  // Location on Map
  vector<double> mapLoc(3, 0.0);

  // Index i, resolution _bin_size, limits mapMin{X,Y,Z}, and dimensions numCells{X,Y,Z}
  // provide complete cell location information
  if(i > -1){
    unsigned remainderTemp;

    // Z location
    remainderTemp = floor(fmod(i, numCellsZ)+0.5);
    mapLoc[2] = remainderTemp*_bin_size+mapMinZ;

    // Y location
    i = floor((i-remainderTemp)/numCellsZ+0.5);
    remainderTemp = floor(fmod(i, numCellsY)+0.5);
    mapLoc[1] = remainderTemp*_bin_size+mapMinY;

    // X location
    i = floor((i-remainderTemp)/numCellsY+0.5);
    mapLoc[0] = i*_bin_size+mapMinX;
  }
  else
    printf("In function 'MapLocationFromInd', index request is %d.", i);

  return mapLoc;
}

Eigen::Vector3i Ray_base::IndToSub(int& i){

  // Location on Map
  Eigen::Vector3i mapLoc;

  // Index i, resolution _bin_size, limits mapMin{X,Y,Z}, and dimensions numCells{X,Y,Z}
  // provide complete cell location information
  if(i > -1){
    unsigned remainderTemp;

    // Z location
    remainderTemp = floor(fmod(i, numCellsZ)+0.5);
    mapLoc[2] = remainderTemp;

    // Y location
    i = floor((i-remainderTemp)/numCellsZ+0.5);
    remainderTemp = floor(fmod(i, numCellsY)+0.5);
    mapLoc[1] = remainderTemp;

    // X location
    i = floor((i-remainderTemp)/numCellsY+0.5);
    mapLoc[0] = i;
  }
  else
    printf("In function 'MapLocationFromInd', index request is %d.", i);

  return mapLoc;
}
