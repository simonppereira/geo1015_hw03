/*
  GEO1015.2020
  hw03 
  --
  [YOUR NAME] 
  [YOUR STUDENT NUMBER] 
  [YOUR NAME] 
  [YOUR STUDENT NUMBER] 
*/


#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iterator>
#include <algorithm>

#include "PlaneDetector.h"

using double3 = linalg::aliases::double3;
int var_id = 1;
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

void PlaneDetector::detect_plane(double epsilon, int min_score, int k)
{
  //-- TIP
  //-- access the input points from the _input_points: 

    //################################//
      // W.O.R.K. F.L.O.W. H.E.R.E. //
    //###############################//


    //if score was 50 or more then good

    //// ############# 
    // while score > min_score do: keep getting 3 new points, calculate plane
    // find score for that plane
    // if min score achieved then all those points are of a plane, get out of loop

    // for loop here
    int score_best = 0;
    std::vector<int> index_list_best = {};
    int len_vec = _input_points.size();
    std::vector<int> my_vec_of_zeros = {};
    // make a list of indices from input vector which have id =0
    for (int i=0 ; i< _input_points.size() ; i++)
    {
        if (_input_points[i].segment_id == 0)
        {
            my_vec_of_zeros.push_back(i);
        }
    }
    std::cout << "we are at end of loop 1 \n";

    for (int iter_ = 0; iter_ < k; iter_++) //loop through for k times
    {
        /*
        //take random indices till k is reached:
        std::uniform_int_distribution<int> distrib(0, len_vec - 1);
        int rand1 = distrib(_rand);
        int rand2 = distrib(_rand);
        int rand3 = distrib(_rand);
        */

        //take 3 random points from list of points having segment 0
        
        std::uniform_int_distribution<int> distrib(0, my_vec_of_zeros.size() - 1);
        int rand1 = distrib(_rand);
        int rand2 = distrib(_rand);
        int rand3 = distrib(_rand);




        
        while (rand1 == rand2 && rand1 == rand3 && rand2 == rand3)
        {
            //std::uniform_int_distribution<int> distrib(0, my_vec_of_zeros.size() - 1);
            rand1 = distrib(_rand);
            rand2 = distrib(_rand);
            rand3 = distrib(_rand);
        }

        //take 3 random indices

        PlaneDetector::Point p1 = _input_points[ my_vec_of_zeros[ rand1 ] ];
        PlaneDetector::Point p2 = _input_points[my_vec_of_zeros[rand2]];
        PlaneDetector::Point p3 = _input_points[my_vec_of_zeros[rand3]];
        std::cout << "we are at end of point selection \n";

        /*
        //std::cout << p1.x << '\t' << p2.x << '\t' << p3.x << '\n';
        while(p1.segment_id != 0 || p2.segment_id != 0 || p3.segment_id != 0)
        {
            //get unique rand
            while (rand1 == rand2 && rand1 == rand3 && rand2 == rand3)
            {
                std::uniform_int_distribution<int> distrib(0, len_vec - 1);
                rand1 = distrib(_rand);
                rand2 = distrib(_rand);
                rand3 = distrib(_rand);
                p1 = _input_points[rand1];
                p2 = _input_points[rand2];
                p3 = _input_points[rand3];
            }
        }   
        // TODO add condition if the random points happen to be the same
        */
        //std::cout << rand1 << '\t'<<rand2 <<'\t' << rand3 << '\n' ;
    
        
        //make a plane
        // find params for the plane
        // plane = normal vector and a point of center
        // other two points help make the normal using cross product
        //consider p1 to be the point wrt the vectors are formed
    
        double3  v1  = { p2.x - p1.x , p2.y - p1.y , p2.z - p1.z };
        double3  v2  = { p3.x - p1.x , p3.y - p1.y , p3.z - p1.z };
        double3 normal = cross(v2, v1);
        double3 normalized_normal = normalize(normal);

        std::vector<double3> vector_params = { normalized_normal , p1 };
        //std::cout << normalized_normal.x << '\t' << normalized_normal.y<<'\t' << normalized_normal.z << '\n';
        //std::cout << "normal    " << '\t' << normal.x << '\t' << normal.y << '\t' << normal.z << '\n';
        //std::cout << "normalized" << '\t' << normalized_normal.x << '\t' << normalized_normal.y << '\t' << normalized_normal.z << '\n';
        std::vector<int> index_list = {};
        index_list.clear();
        int scorer = 0;
        int flag = 0;
        for (int i=0 ; i< len_vec ; i++) //for every point in the dataset  except the 3 which define the plane
        {   
            my_vec_of_zeros.clear();
            if ( _input_points[i].segment_id == 0 )
            {
                
                my_vec_of_zeros.push_back(i);

            }
            
            if (i != rand1 || i != rand2 || i != rand3 || _input_points[i].segment_id == 0) //added last condition in case the point is already CLASSIFIED
            {   
                // for all points except the 3 selected ones
                // calculate the distance and check with epsilon
                double3 point_vec = { _input_points[i].x - p1.x , _input_points[i].y - p1.y, _input_points[i].z - p1.z };

                double dist = abs(dot(point_vec, normalized_normal)) ;
            
                if (dist < epsilon)
                {   
                //std::cout << dist << '\n';
                    // increment score by 1
                    scorer++;//one more point added to the plane
                    index_list.push_back(i);
                    
                    if (index_list.size() > min_score)
                    {                       
                        //best plane found with min of 50 points
                        // set flag  =1
                        flag = 1;
                        //break;
                     
                    }
                }
            }
            
        } //end child for
        std::cout << "we are at if condition\n";
        //std::cout << scorer << '\t' << score_best << '\n';
        //insert conditional checking score
        if (scorer > score_best && scorer > min_score) //score obtained in this iter is better than those before
        {
            score_best = scorer; //update best score
            //if (flag == 1) // if there were at least 50 points found in the plane
            //{
            index_list_best = index_list; //update best list of indices
            index_list_best.push_back(my_vec_of_zeros[ rand1 ]);
            index_list_best.push_back(my_vec_of_zeros[ rand2 ]);
            index_list_best.push_back(my_vec_of_zeros[ rand3 ]);
            /*std::cout << index_list_best.size() <<" this is the size of vector of indices \n";*/
            //}
            //else std::cout << "alas we couldnt find a good plane here \n";

        }
        else if (scorer > score_best && scorer < min_score)
        {
            std::cout << "score more than best score but not more than min score" << '\n';
        }
    } //end parent for 
    //update values for the best plane found
    std::cout << "we reached a plane which works \t end of the loops so far" << '\n';
    // initiate a global variable to store the point and normal of the plane so that some other plane does not coincide with it
    std::cout << index_list_best.size() << " this is the size of vector we are now gonna write \n";
    if (index_list_best.size() > min_score)
    {
        std::cout << "list of index is smaller than min score \n";

        int new_id = PlaneDetector::get_seg_id();
        std::cout << "planes are to be updated with value : " << new_id << '\n';
        for (int i = 0; i < index_list_best.size(); i++)
        {
            //read indices and update the segment ID
            //auto pathy = get_seg_id_used();
            //for (auto i : pathy)
            //std::cout << i << ' ';
            if (_input_points[index_list_best[i]].segment_id == 0)
            {
                _input_points[index_list_best[i]].segment_id = new_id;
            }


            //std::cout << _input_points[ index_list_best[i] ].norm_point_storer << '\n';

        }
        //implement the segmentation value updation for vectors here (for the best indices)

    }
    
}



// PLY I/O

/*
!!! TO BE COMPLETED !!!

Function that writes the entire point cloud including the segment_id of each point to a .ply file

Input:
   filepath:  path of the .ply file to write the points with segment id
*/
void PlaneDetector::write_ply(std::string filepath) {
    std::cout<<"Writing file now to " << filepath << std::endl ;
    std::ofstream ofs( filepath.c_str() , std::ofstream::out);
    if (ofs.is_open())
    {
        ofs << "lorem ipsum";
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
