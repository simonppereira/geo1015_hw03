/*
  GEO1015.2020
  hw03 
  --
  Pratyush Kumar
  5359252
  [YOUR NAME] 
  [YOUR STUDENT NUMBER] 
*/


#include <string>
#include <vector>
#include <random>

//-- Simple linear algebra library that you can use for distance computations etc
//-- See https://github.com/sgorsten/linalg
#include <linalg/linalg.h>
using double3 = linalg::aliases::double3;
using double2 = linalg::aliases::double2;
using int3 = linalg::aliases::int3;
/*
The main class of this assignment. You are allowed to add functions and member variables for your own use.
*/
class PlaneDetector {
  //-- you can add your own variables and functions here
  /*
  !!! DO NOT MODIFY below this line !!!
  */
  public:
  /*
  We define a Point struct that inherits from the double3 struct that is defined in linalg.h. 
  This means you can use a Point as a double3, ie. with all the linear algebra functions defined for double3 in linalg.h.
  The only thing added here is the segment_id, which we use to indicate to what plane this point is assigned. 

  NOTICE that the segment_id==0 (the default value) means that the point is not assigned to any plane.
  */

  struct Point : double3 {
    using double3::double3;
    int segment_id{0};
    //std::vector<double3> norm_point_storer;
  };

  //-- The main plane detection function where you need to implement the RANSAC algorithm (in the PlaneDetector.cpp file)
  void detect_plane(double epsilon, int min_score, int k, int n_planes, int dist);

  //-- .PLY reading (already implemented for you)
  bool read_ply(std::string filepath);
  //-- .PLY writing (you need to implement in PlaneDetector.cpp)
  void write_ply(std::string filepath);
  

  std::vector<int> dist_return(double3 pt, double dist_around);

  int3 randfunc(std::vector<Point> inp_pts);

  std::vector<double2> get_min_max(std::vector<Point> pt);

  void RANSACinator(std::vector<PlaneDetector::Point *> &ransac_points, double epsilon, int min_score, int k);
  std::vector<double3> PlaneDetector::get_cubes(std::vector<double2> pt);

  //-- point cloud access (for the viewer application)
  const std::vector<Point>& get_input_points() {
    return _input_points;
  };

  std::vector<int> get_segment_0_points(std::vector<Point> inp_pts)
  {
      // returns a vector containing indexes wrt inp_pts of points which have seg_id of 0
      std::vector<int> vect = {};
      for (int i=0; i< inp_pts.size(); i++)
      {
          if ( inp_pts[i].segment_id == 0)
          {
              vect.push_back(i);
          }
      }
      return vect;
  }


  
  int get_seg_id() // function/ method used to get a new ID and append the new value to the end of seg_id_used vector
  {
      ++current_plane_no;
      return   current_plane_no;
  }


  /////////////////////////// P.R.I.V.A.T.E ///////////////////////
  private:
      int current_plane_no = 0;
  //-- This variable holds the entire input point cloud after calling read_ply()
      std::vector<Point> _input_points;
      std::vector<Point> _pts_decreaser = _input_points ;


  //-- random number generator to generate your random numbers (important for RANSAC!)
  std::mt19937 _rand{ std::mt19937{std::random_device{}()} };

};
