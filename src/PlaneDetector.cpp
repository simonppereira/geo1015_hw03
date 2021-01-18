/*
  GEO1015.2020
  hw03
  --
  Georgios Triantafyllou [5381738]
  Simon Pena Pereira     [5391210]
  Pratyush Kumar         [5359252]
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iterator>
#include <algorithm>

#include "PlaneDetector.h"

using double3 = linalg::aliases::double3;
using double2 = linalg::aliases::double2;
using int3 = linalg::aliases::int3;

int count = 0;
/*
!!! TO BE COMPLETED !!!

Function that implements the RANSAC algorithm to detect one plane in the point cloud that is 
accessible from _input_points variable (which contains the point read from the input ply file).

NOTICE that this function can be called multiple times! On each call a *new* plane should be generated
that does not contain any inliers that belong to a previously detected plane. Each plane should 
have a unique segment_id (an int; '1' for the first plane, '2' for the second plane etc.).

Input:
    epsilon:          maximum distance from an inlier to corresponding plane.
    min_score:        minimum score (= number of inlier) of a plane. Planes with a lower score
                      should not be detected.
    k:                number of times a new minimal set and a consencus set is computed.
    
Output:
    Updated .segment_id's on the inliers in the newly detected plane. The inliers contain both the
    consensus set and minimal set.
*/
std::vector<double2> PlaneDetector::get_min_max(std::vector<Point> pt)
{
    // .x gives the min
    // .y gives the max
    double2 x = { 999999, -1000 };
    double2 y = { 999999, -1000 };
    double2 z = { 999999, -1000 };

    for (int i=0; i<pt.size(); i++)
    {
        //update min and max values
        //update x values
        if (pt[i].x < x.x) //min
        {
            x.x = pt[i].x;
        }
        if (pt[i].x > x.y) //max
        {
            x.y = pt[i].x;
        }
        //update y values
        if (pt[i].y < y.x) //min
        {
            y.x = pt[i].y;
        }
        if (pt[i].y > y.y) //max
        {
            y.y = pt[i].y;
        }
        //update z values
        if (pt[i].z < z.x) //min
        {
            z.x = pt[i].z;
        }
        if (pt[i].z > z.y) //max
        {
            z.y = pt[i].z;
        }

    }
    
    std::vector<double2> ptt = { x,y,z };
    return ptt;
}

std::vector<int> PlaneDetector::dist_return(double3 pt, double dist_around)
{
    /*returns a vector of indexes of points which lie within a search radius wrt the given pt*/
    std::vector<int> dist_list = {};
    for (int i = 0; i < _input_points.size(); i++)//i is index from _inpout_points
    {
        double3 pt_from_input = { _input_points[i].x ,_input_points[i].y ,_input_points[i].z };
        double d = distance(pt_from_input, pt);
        if (d < dist_around) dist_list.push_back(i);
    }
    if (dist_list.size() > 100)
    {
        return dist_list;
    }
    else
    {
    std::vector<int> a = {};
        a.push_back(-10);
        return a;
    }

}


int3 PlaneDetector::randfunc(std::vector<Point> inp_pts)
{
    /*returns three indices in inp_pts */
    std::vector<int> zero_vects = PlaneDetector::get_segment_0_points( inp_pts );
    int len_zerovec = zero_vects.size();
    if (len_zerovec <= 3)
    {
        std::cout << "cannot make 3 unique points \n";
            //break;
    }
    int rand1_ = 0;
    int rand2_ = 0;
    int rand3_ = 0;
    
    //get unique rand
    while (rand1_ == rand2_ && rand1_ == rand3_ && rand2_ == rand3_ && len_zerovec>3)
    {
        std::uniform_int_distribution<int> distrib(0, len_zerovec - 1);
        rand1_ = distrib(_rand);
        rand2_ = distrib(_rand);
        rand3_ = distrib(_rand);
    };
    
    // rands are now indexes in zero_vect of points in _input with seg id =0
    int rand1 = zero_vects[rand1_];
    int rand2 = zero_vects[rand2_];
    int rand3 = zero_vects[rand3_];
    
    int3 pt = { rand1,rand2,rand3 };
    return pt; //index of 3 unique random points wrt inp_pts
}

///////////

void PlaneDetector::RANSACinator(std::vector<PlaneDetector::Point *>& ransac_points_arg, double epsilon, int min_score, int k)
{
    // while score > min_score do: keep getting 3 new points, calculate plane
    // find score for that plane
    // if min score achieved then all those points are of a plane, get out of loop

    std::vector<PlaneDetector::Point> ransac_points = {};
    for (int i=0 ; i<ransac_points_arg.size(); i++)
    {
        ransac_points.push_back( (*(ransac_points_arg[i]))  );
    }
    
    int score_best = 0;

    std::vector<int> index_list_best = {};
    int len_vec = ransac_points.size();

    for (int iter_ = 0; iter_ < k; iter_++) //loop through for k times
    {
        std::vector<int> zero_vects = get_segment_0_points(ransac_points);
        int len_zerovec = zero_vects.size();
        //take random indices from 0 segment vector
        if (len_zerovec < min_score)
        {
            break;
        }

        //take 3 random indices
        int3 rand_pt_indexes = randfunc(ransac_points);
        int rand1 = rand_pt_indexes.x;
        int rand2 = rand_pt_indexes.y;
        int rand3 = rand_pt_indexes.z;

        PlaneDetector::Point p1 = ransac_points[rand1];
        PlaneDetector::Point p2 = ransac_points[rand2];
        PlaneDetector::Point p3 = ransac_points[rand3];
    
        //make a plane
        // find params for the plane  |  plane = normal vector and a point of center
        // other two points help make the normal using cross product  | consider p1 to be the point wrt the vectors are formed
        //https://mathinsight.org/distance_point_plane#:~:text=The%20length%20of%20the%20gray,dot%20product%20v%E2%8B%85n this was used for code below

        double3  v1 = { p2.x - p1.x , p2.y - p1.y , p2.z - p1.z };
        double3  v2 = { p3.x - p1.x , p3.y - p1.y , p3.z - p1.z };
        double3 normal = cross(v2, v1);
        double3 normalized_normal = normalize(normal);

        std::vector<int> index_list = {};
        index_list.clear();
        int scorer = 0;
        int flag = 0;

        for (int i = 0; i < len_vec; i++) //for every point in the dataset  except the 3 which define the plane
        {
            //i is the index in ransac_points not in _input_points
            if (i != rand1 || i != rand2 || i != rand3 || ransac_points[i].segment_id == 0) //added last condition in case the point is already CLASSIFIED
            {
                // for all points except the 3 selected ones  calculate the distance and check with epsilon
                double3 point_vec = { ransac_points[i].x - p1.x , ransac_points[i].y - p1.y, ransac_points[i].z - p1.z };
                double dist = abs(dot(point_vec, normalized_normal));

                if (dist < epsilon)
                {
                    scorer++;//one more point added to the plane
                    index_list.push_back(i); //list of indices wrt ransac_points
                }
            }
        } //end child for

        // conditional checking score
        if (scorer > score_best && scorer > min_score) //score obtained in this iter is better than those before
        {
            score_best = scorer; //update best score
            index_list_best = index_list; //update best list of indices
            index_list_best.push_back(rand1);
            index_list_best.push_back(rand2);
            index_list_best.push_back(rand3);
        }
    } //end parent for 

    //update values for the best plane found
    if (index_list_best.size() > min_score)
    {
        int new_id = PlaneDetector::get_seg_id();
        std::cout << "planes are to be updated with value : " << new_id << '\n';
       
        // USUAL IMPLEMENTATION {FOR A SUBSET}
        for (int i = 0; i < index_list_best.size(); i++)
        {
            //index list best has the indices of planes from ransac_points
            // neede to update the segment id of the points in ransac_points
            // but should also update in input_points

            if (  (*(ransac_points_arg[index_list_best[i]])).segment_id == 0)
            {
                (*(ransac_points_arg[ index_list_best[i] ])).segment_id = new_id;
            }
        }
    }

}


/////////////////////////    D.E.T.E.C.T   P.L.A.N.E.  //////////////////////
void PlaneDetector::detect_plane(double epsilon, int min_score, int k, int n_planes, int dist)
{
    //################################//
         //  W.O.R.K. F.L.O.W.  //
    //###############################//

  //for sphere in sphere coordinate list 
  //ransacinator(sphere)
  //get a random point
  //run the while loop till we run out of points to visit or points left <min score

    std::vector<PlaneDetector::Point> pt_to_visit = {};
    std::copy(_input_points.begin(), _input_points.end(), back_inserter(pt_to_visit)); 

    while ( pt_to_visit.size() > min_score )
        {

            std::vector<int> my_vec_of_zeros = {};
            int counter = 0;

            // make a list of indices from input vector which have id =0
            for (int i = 0; i < _input_points.size(); i++)
            {
                if (_input_points[i].segment_id == 0)
                {
                    my_vec_of_zeros.push_back(i);
                }
            }
            PlaneDetector::Point p;
            if (my_vec_of_zeros.size() > min_score) {
                std::uniform_int_distribution<int> distrib(0, my_vec_of_zeros.size() - 1);
                //int rand = 0;
                int rand = distrib(_rand);
                p = _input_points[my_vec_of_zeros[rand]]; //p always has a segment id of 0
            }
            else break;

            double3 selected_point = { p.x,p.y,p.z };
            //indpt in sphere is vector of indicesd wrt _input_points
            std::vector<int> index_pt_in_sphere = dist_return(selected_point, double(dist));

            std::vector<PlaneDetector::Point*> points_for_ransac = {};
            for (int i = 0; i < index_pt_in_sphere.size(); i++)
            {
                //i is the index instance
                points_for_ransac.push_back(&_input_points[index_pt_in_sphere[i]]);
                if (_input_points[index_pt_in_sphere[i]].segment_id != 0)
                {
                    if( !pt_to_visit.empty() ) pt_to_visit.pop_back();
                }
            
                //_input_points[distances_from_pt[i]].segment_id = count;
            }

            for (int planess = 0; planess <= n_planes; planess++)
            {
                PlaneDetector::RANSACinator(points_for_ransac , epsilon, min_score, k);
            }// end child loop
        }//end of while

}//end of func


// PLY I/O

/*
Function that writes the entire point cloud including the segment_id of each point to a .ply file

Input:
   filepath:  path of the .ply file to write the points with segment id
*/
void PlaneDetector::write_ply(std::string filepath)
{
    std::cout<<"Writing file now to " << filepath << std::endl ;
    std::ofstream ofs( filepath.c_str() , std::ofstream::out);
    if (ofs.is_open())
    {     
        char buf[65];
        ofs << "ply" << "\n";
        ofs << "format ascii 1.0" << "\n";
        ofs << "element vertex " << _input_points.size() << "\n";
        ofs << "property float x" << "\n";
        ofs << "property float y" << "\n";
        ofs << "property float z" << "\n";
        ofs << "property int segment_id" << "\n";
        ofs << "end_header" << "\n";

        for (int i = 0; i < _input_points.size(); i++)
        {
            sprintf(buf, "%f %f %f %d",
                _input_points[i].x,
                _input_points[i].y,
                _input_points[i].z,
                _input_points[i].segment_id
            );
            if (i == _input_points.size())  ofs << buf;
            else ofs << buf << '\n';
        }

        ofs.close();       
    }
    else
    {
        std::cout << "Unable to write sed file noises";
    }
}

/*
!!! DO NOT MODIFY read_ply() !!!

This function is already implemented.
*/
bool PlaneDetector::read_ply(std::string filepath) {

  std::cout << "Reading file: " << filepath << std::endl;
  std::ifstream infile(filepath.c_str(), std::ifstream::in);
  if (!infile)
  {
    std::cerr << "Input file not found.\n";
    return false;
  }
  std::string cursor;
  
  // start reading the header
  std::getline(infile, cursor);
  if (cursor != "ply") {
    std::cerr << "Magic ply keyword not found\n";
    return false;
  };

  std::getline(infile, cursor);
  if (cursor != "format ascii 1.0") {
    std::cerr << "Incorrect ply format\n";
    return false;
  };

  // read the remainder of the header
  std::string line = "";
  int vertex_count = 0;
  bool expectVertexProp = false, foundNonVertexElement = false;
  int property_count = 0;
  std::vector<std::string> property_names;
  int pos_x = -1, pos_y = -1, pos_z = -1, pos_segment_id = -1;

  while (line != "end_header") {
    std::getline(infile, line);
    std::istringstream linestream(line);

    linestream >> cursor;

    // read vertex element and properties
    if (cursor == "element") {
      linestream >> cursor;
      if (cursor == "vertex") {
        // check if this is the first element defined in the file. If not exit the function
        if (foundNonVertexElement) {
          std::cerr << "vertex element is not the first element\n";
          return false;
        };

        linestream >> vertex_count;
        expectVertexProp = true;
      } else {
        foundNonVertexElement = true;
      }
    } else if (expectVertexProp) {
      if (cursor != "property") {
        expectVertexProp = false;
      } else {
        // read property type
        linestream >> cursor;
        if (cursor.find("float") != std::string::npos || cursor == "double") {
          // read property name
          linestream >> cursor;
          if (cursor == "x") {
            pos_x = property_count;
          } else if (cursor == "y") {
            pos_y = property_count;
          } else if (cursor == "z") {
            pos_z = property_count;
          }
          ++property_count;
        }
        else if (cursor.find("uint") != std::string::npos || cursor == "int") {
          // read property name
          linestream >> cursor;
          if (cursor == "segment_id") {
            pos_segment_id = property_count;
          }
          ++property_count;
        }
      }
    
    }
  }

  // check if we were able to locate all the coordinate properties
  if ( pos_x == -1 || pos_y == -1 || pos_z == -1) {
    std::cerr << "Unable to locate x, y and z vertex property positions\n";
    return false;
  };

  // read the vertex properties
  for (int vi = 0; vi < vertex_count; ++vi) {
    std::getline(infile, line);
    std::istringstream linestream(line);

    double x{},y{},z{};
    int sid{};
    for (int pi = 0; pi < property_count; ++pi) {
      linestream >> cursor;
      if ( pi == pos_x ) {
        x = std::stod(cursor);
      } else if ( pi == pos_y ) {
        y = std::stod(cursor);
      } else if ( pi == pos_z ) {
        z = std::stod(cursor);
      } else if ( pi == pos_segment_id ) {
        sid = std::stoi(cursor);
      }
    }
    auto p = Point{x, y, z};
    if (pos_segment_id!=-1) {
      p.segment_id = sid;
    }
    _input_points.push_back(p);
  }

  std::cout << "Number of points read from .ply file: " << _input_points.size() << std::endl;

  return true;
}
