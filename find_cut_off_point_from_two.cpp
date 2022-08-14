
#include <iostream>
#include <cmath>

using namespace std;

#include <ctime>

#include <Eigen/Core>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pyhelper.hpp"

using namespace Eigen;
using namespace cv;
class Mypoint{
  public:
  double x, y ,z;
  Mypoint()
  {
    x = 0;
    y = 0;
    z = 0;
  }
  Mypoint(double mx, double my, double mz)
  {
    x = mx;
    y = my;
    z = mz;
  }
  Mypoint get_middle(Mypoint b)
  {
    Mypoint kq;
    // if( b.x* b.x > x*x)
    //   kq.x = (b.x - x)/2;
    // else
    //   kq.x = (x - b.x)/2;
    // if( b.y* b.y > y*y)
    //   kq.y = (b.y - y)/2;
    // else
    //   kq.y = (y - b.y)/2;
    // if( b.z* b.z > z*z)
    //   kq.z = (b.z - z)/2;
    // else
    //   kq.z = (z - b.z)/2;
      kq.x = b.x/2 + x/2;
      kq.y = b.y/2 + y/2;
      kq.z = b.z/2 + z/2;
    return kq;

  }
};

class Cut_region
{
  public:
  Mypoint a1, a2;
  int value;
  
  Cut_region(double mx1, double my1, double mz1, double mx2, double my2, double mz2, int mvalue)
  {
    a1.x = mx1;
    a1.y = my1;
    a1.z = mz1;
    a2.x = mx2;
    a2.y = my2;
    a2.z = mz2;
    value = mvalue;
  }
  void set(double mx1, double my1, double mz1, double mx2, double my2, double mz2, int mvalue)
  {
    a1.x = mx1;
    a1.y = my1;
    a1.z = mz1;
    a2.x = mx2;
    a2.y = my2;
    a2.z = mz2;
    value = mvalue;
  }
   
};
class List_cut_region
{
  public:
  typedef std::shared_ptr<List_cut_region> Ptr;
  std::vector<Cut_region> list_r;
  void add_cut_region(double x1, double y1, double z1, double x2, double y2, double z2,int value)
  {
    Cut_region c(x1,y1,z1,x2,y2,z2,value);
    int i = 0;
    int n = list_r.size();
    if (n == 0)
      list_r.push_back(c);
    else
    {
      std::vector<Cut_region>::iterator it; 
      it = list_r.begin();
      while ( it != list_r.end() && it->a1.y < c.a1.y)
        it++;
      list_r.insert(it,c);
    }
  }
  void add_cut_region(Cut_region a)
  {
    int i = 0;
    int n = list_r.size();
    if (n == 0)
      list_r.push_back(a);
    else
    {
      std::vector<Cut_region>::iterator it; 
      it = list_r.begin();
      while ( it != list_r.end() && it->a1.y < a.a1.y)
        it++;
      list_r.insert(it,a);
    }
  }


};
// Calculates rotation matrix given euler angles.
Eigen::Matrix<double,3,3>  eulerAnglesToRotationMatrix(double x, double y, double z)
{
    // Calculate rotation about x axis
    Eigen::Matrix<double,3,3> Rx;
    Rx  << 
               1,       0,              0,
               0,       cos(x),   -sin(x),
               0,       sin(x),   cos(x)
               ;

    // Calculate rotation about y axis
     Eigen::Matrix<double,3,3> Ry;
     Ry <<
               cos(y),    0,      sin(y),
               0,               1,      0,
               -sin(y),   0,      cos(y)
               ;

    // Calculate rotation about z axis
     Eigen::Matrix<double,3,3> Rz;
     Rz <<
               cos(z),    -sin(z),      0,
               sin(z),    cos(z),       0,
               0,               0,                  1;

    // Combined rotation matrix
   // Mat Rc = R_z * R_y * R_x;
  Eigen::Matrix<double,3,3> R;
  R = Rz * Ry * Rx;
  return R;

}
  
pcl::visualization::CloudViewer viewer("viewer");
  
typedef pcl::PointXYZRGB PointT;

typedef pcl::PointCloud<PointT> PointCloud;
int colors[28][3];
void init(int colors[][3])
{
  for(int i = 0; i< 28; i++)
  {
    for(int j = 0; j< 3; j++)
      colors[i][j] = 0;
  }
  colors[0][0] = 255;
  colors[1][1] = 255;
  colors[2][2] = 255;
  colors[3][0] = 55;
  colors[3][1] = 155;
  colors[4][0] = 255;
  colors[4][1] = 255;
  colors[5][1] = 255;
  colors[5][2] = 255;
  colors[6][0] = 255;
  colors[6][2] = 255;
  colors[7][0] = 192;
  colors[7][1] = 192;
  colors[7][2] = 192;
  colors[8][0] = 128;
  colors[8][1] = 128;
  colors[8][2] = 128;
  colors[9][0] = 153;
  colors[9][1] = 153;
  colors[9][2] = 255;
  colors[10][0] = 153;
  colors[10][1] = 51;
  colors[10][2] = 102;
  colors[11][0] = 255;
  colors[11][1] = 255;
  colors[11][2] = 204;

   colors[12][0] = 204;
  colors[12][1] = 255;
  colors[12][2] = 255;

   colors[13][0] = 102;
  colors[13][1] = 0;
  colors[13][2] = 102;
 
  colors[14][0] = 255;
  colors[14][1] = 128;
  colors[14][2] = 128;


    CPyInstance hInstance;
    CPyObject pName ;
    CPyObject pModule ;
    CPyObject pFunc ;
      std:string modulePath="image1";
      PyObject* pyModuleName = PyUnicode_DecodeFSDefault(modulePath.c_str());
    pName = PyUnicode_FromString("image1");
    pModule = PyImport_Import(pName);
    if(pModule)
    {
        pFunc = PyObject_GetAttrString(pModule, "converimage");
          PyObject_CallObject(pFunc, NULL);
    }
    else
    {
        std::cout <<"ERROR: Module not imported\n";
    }
}

PointT getequal(PointT a)
{
  PointT b; 
  b.x = a.x;
  b.y = a.y;
  b.z = a.z;
  b.g = a.g;
  b.b = a.b;
  b.r = a.r;
  return b;
}
void setcolor(PointT *a, int i)
{
 // cout <<"\n setcolor " <<i;
  a->r = colors[i][0];
  a->g = colors[i][1];
  a->b = colors[i][2];
}

int compare_xyz(PointT a, PointT b)
{
  if(a.x == b.x && a.y == b.y && a.z == b.z)
    return 1;
  else
    return 0;
}
int compare_xyz(PointT *a, PointT b)
{
  if(a->x == b.x && a->y == b.y && a->z == b.z)
    return 1;
  else
    return 0;
}
int compare_xyzrgb(PointT a, PointT b)
{
  if(a.x == b.x && a.y == b.y && a.z == b.z && a.r == b.r && a.g == b.g && a.b == b.b)
    return 1;
  else
    return 0;
}
double khoangcach(PointT a, PointT b)
{
  float kq = sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) + pow(a.z - b.z,2));
  return kq;
}

int count_component( PointCloud::Ptr tmp, List_cut_region::Ptr list)
{
      int K = 2;
      int g = 0;
      PointCloud::Ptr tmp_f(new PointCloud);
      while ( tmp->size() > 0)
      {
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        // kdtree.setInputCloud (tmp);

        pcl::PointXYZRGB searchPoint;

        kdtree.setInputCloud (tmp);
        searchPoint.x = 0;
        searchPoint.y = 0;
        searchPoint.z = 0;
        
        PointCloud::Ptr tmp_r1(new PointCloud);
        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
      
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) <=1 )
          break;

        searchPoint.x = (*tmp)[ pointIdxKNNSearch[1] ].x;
        searchPoint.y = (*tmp)[ pointIdxKNNSearch[1] ].y;
        searchPoint.z = (*tmp)[ pointIdxKNNSearch[1] ].z;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<pcl::PointXYZRGB> vfind;
        float radius = 0.02;
        int n = 0;
        double x1 = 100,y1 = 100,z1 =100,x2 =-100,y2 =-100,z2 =-100 ;
        do {

              kdtree.setInputCloud (tmp);

              if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
              {
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                { 
                
                  pcl::PointXYZRGB rt = getequal((*tmp)[ pointIdxRadiusSearch[i] ]);
         
                  if(khoangcach(searchPoint, rt) > radius) //kiem tra lai radius vi ham tren tinh radius sai
                    continue;
                  if( i > 0)
                  {
                     
                  }
               
                  pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
                  
                  for (it = tmp->begin(); it != tmp->end();)
                  {

                      bool remove_point = 0;

                      if ( it->z == rt.z && it->y == rt.y && it->x == rt.x     )
                      {
                        n++;
                    //  cout <<"\ndeletel";
                        it = tmp->erase(it);
                        if(x1 > rt.x)
                          x1 = rt.x;
                        if( y1 > rt.y)
                          y1 = rt.y;
                        if(z1 > rt.z)
                          z1 = rt.z;
                        if( x2 < rt.x)
                          x2 = rt.x;
                        if(y2 < rt.y)
                          y2 = rt.y;
                        if( z2 < rt.z)
                          z2 = rt.z;
                      setcolor(&rt,g);
                    //  rt.b = 160;
                   
                        tmp_r1->push_back(rt );
                        vfind.push_back(rt);
                        
                      }
                      else
                      {
                        it++;
                      }
                  }
                  
                }
              }
             
              //them doan add pop tim tiep theo
              if (vfind.size() > 0)
              {
                  searchPoint.x = vfind[0].x;
              //    cout << "\nsearchpoint.x "<<vfind[0].x ;
                  searchPoint.y = vfind[0].y;
                  searchPoint.z = vfind[0].z;
                  vfind.erase(vfind.begin());
              }
            

        }while(vfind.size() > 0 && tmp->size()>0);
     //   cv::waitKey(0);
    //    viewer.showCloud( tmp_r1 );
   //    cout << "\n n=" << n;
    //    cout <<"tmp_r1 size"<<tmp_r1->size();
          if(n > 10)
          {
            list->add_cut_region(x1,y1,z1,x2,y2,z2,g);
            g++;
            *tmp_f += *tmp_r1;
          }  
       
    }
    tmp_f->swap(*tmp);
    cout <<"\nTOng la " <<g;
    return g;
}

void create_pcl(vector<cv::Mat> colorImgs,vector<cv::Mat>  depthImgs,
 vector<Eigen::Isometry3d> poses, PointCloud::Ptr pointCloud , Eigen::Isometry3d T4, int type)
{
   PointCloud::Ptr tmp(new PointCloud);
  int depthScale = 1000;
    double cx[2];
    double cy[2];
    double fx[2];
    double fy[2];
    
    cx[1] =  	        318.60302734375;
    cy[1] =         	239.071395874023;
    fx[1] =          	385.441162109375;
    fy[1] =          	385.441162109375;

    cx[0] =  	        322.952789306641;
    cy[0] =         	238.483856201172;
    fx[0] =          	386.620269775391;
    fy[0] =          	386.620269775391;
   int r =0, g= 0, b = 0;
  for (int i = 0; i < 2; i++) 
        {
       //int i = 1;
          PointCloud::Ptr current(new PointCloud);
      
          cv::Mat color = colorImgs[i];
          cv::Mat depth = depthImgs[i];
          //  Eigen::Isometry3d T = poses[i];
          for (int v = 0; v < color.rows; v+=2)
          {    for (int u = 0; u < color.cols; u+=2) 
              {
                  unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                  Eigen::Vector3d point;
                  point[2] = double(d) / depthScale;
                  if (point[2]== 0 || point[2] >1.0) continue;
                  point[0] = (u - cx[i]) * point[2] / fx[i];
                  point[1] = (v - cy[i]) * point[2] / fy[i];
                  Eigen::Vector3d pointWorld ;
                  // pointWorld = poses[i] * point;
                  PointT p;
                
                  p.x = point[0];
                  p.y = point[1];
                  p.z = point[2];
                  p.b = color.data[v * color.step + u * color.channels()];
                  p.g = color.data[v * color.step + u * color.channels() + 1];
                  p.r = color.data[v * color.step + u * color.channels() + 2];
                  if( i== 1)
                  {
                  // p.g = 50;
                  }
              //    if (( p.r == 128 && p.g == 64 && p.b == 128) || ( p.r == 250 && p.g == 170 && p.b == 160)|| ( p.r == 0 && p.g == 255 && p.b == 0))
                  if(type == 2)
                  {
                      r = 250;
                      g = 170;
                      b = 160;

                  }
                  if(type == 1)
                  {
                    r = 128;
                      g = 64;
                      b = 128;
                  }
                   if(type == 3)
                  {
                    r = 0;
                      g = 255;
                      b = 0;
                  }
                   if(type == 4) //chi lay intersection cua branch
                  {
                      r = 255;
                      g = 0;
                      b = 0;
                  }
                  if(type == 5) //chi lay intersection cua sucker
                  {
                      r = 0;
                      g = 0;
                      b = 255;
                  }
                  if(type == 0) //chi lay tatca
                  {
                     if( (p.r == 0 && p.b == 0 && p.g == 0))
                        continue;
                  }
                  else
                  {
                    if( (p.r != r || p.b!= b || p.g != g))
                                        continue;
                  }  
                   
                  current->points.push_back(p);
                 //// tao them 9 point ben canh
                  for(int h = -1; h < 2 ; h++)
                  {
                    for(int k = -1; k < 2; k++ )
                    {
                      for(int j = -1; j < 2; j++)
                      {
                        if( h==0 && k ==0 & j ==0)
                          continue;
                        PointT p2;
                        p2.x = p.x + 0.005*h;
                        p2.y = p.y + 0.005*k;
                        p2.z = p.z + 0.005*j;
                        p2.b = p.b;
                        p2.g = p.g;
                        p2.r = p.r;
                        current->points.push_back(p2);
                      }
                    }
                  }
                
              }
          }

            // depth filter and statistical removal 
            PointCloud::Ptr tmp(new PointCloud);
              PointCloud::Ptr tmp2(new PointCloud);
         

            pcl::transformPointCloud (*current, *tmp2, poses[i].matrix() );
            if(i == 1)
            {
                PointCloud::Ptr tmp1(new PointCloud);
                pcl::transformPointCloud (*tmp2, *tmp1, T4.matrix() );
                cout << T4.matrix();
                // PointCloud::Ptr tmp2(new PointCloud);
                // pcl::transformPointCloud (*tmp1, *tmp2, T5.matrix() );
                (*pointCloud) += *tmp1;
            }
            else
              (*pointCloud) += *tmp2;
        }

        pointCloud->is_dense = false;
      //filter
      //int
        // voxel filter 
       
          // pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
          //    PointCloud::Ptr tmp1(new PointCloud);
          //       statistical_filter.setMeanK(50);
          //       statistical_filter.setStddevMulThresh(1.0);
          //       statistical_filter.setInputCloud(pointCloud);
          //       statistical_filter.filter(*tmp1);

                 pcl::VoxelGrid<PointT> voxel_filter;
        double resolution = 0.005;
        voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
        
        voxel_filter.setInputCloud(pointCloud);
        voxel_filter.filter(*tmp);
        tmp->swap(*pointCloud);

        cout << "\n1cloud size" << pointCloud->size();
       
}

PointCloud::Ptr createtarget_colorpointcloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d pose, 
            double depthScale, double cx, double cy, double fx, double fy)
{
    PointCloud::Ptr current(new PointCloud);
    current->points.clear();
    for (int v = 0; v < color.rows; v++)
    {    for (int u = 0; u < color.cols; u++) 
        {
            unsigned int d = depth.ptr<unsigned short>(v)[u]; 
            Eigen::Vector3d point;
            point[2] = double(d) / depthScale;
            if (point[2]== 0 || point[2] >10.0) continue;
            point[0] = (u - cx ) * point[2] / fx ;
            point[1] = (v - cy ) * point[2] / fy ;
            Eigen::Vector3d pointWorld ;
              pointWorld = pose.inverse() * point;
            PointT p;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            // p.x = point[0];
            // p.y = point[1];
            // p.z = point[2];
            p.b = color.data[v * color.step + u * color.channels()];
            p.g = color.data[v * color.step + u * color.channels() + 1];
            p.r = color.data[v * color.step + u * color.channels() + 2];
        
            // if( p.r == 0 && p.g == 0 && p.b == 0)
            //   continue;
            current->points.push_back(p);
        }
    }
    return current;
}


void create_3pcl(vector<cv::Mat> colorImgs,vector<cv::Mat>  depthImgs,
 vector<Eigen::Isometry3d> poses, PointCloud::Ptr pointCloud ,PointCloud::Ptr sCloud,PointCloud::Ptr bCloud, Eigen::Isometry3d T4)
{
  cout << "\nmatrix\n";
  cout << T4.matrix();
   PointCloud::Ptr tmp(new PointCloud);
  int depthScale = 1000;
    double cx[2];
    double cy[2];
    double fx[2];
    double fy[2];
    
    cx[1] =  	        318.60302734375;
    cy[1] =         	239.071395874023;
    fx[1] =          	385.441162109375;
    fy[1] =          	385.441162109375;

    cx[0] =  	        322.952789306641;
    cy[0] =         	238.483856201172;
    fx[0] =          	386.620269775391;
    fy[0] =          	386.620269775391;
   int r4 =0, g4= 0, b4 = 0;
  int r5 =0, g5= 0, b5 = 0;
 for (int i = 0; i < 2; i++) 
        {
      //  int i = 0;
          PointCloud::Ptr current1(new PointCloud);
       PointCloud::Ptr current4(new PointCloud);
        PointCloud::Ptr current5(new PointCloud);
          cv::Mat color = colorImgs[i];
          cv::Mat depth = depthImgs[i];
          //  Eigen::Isometry3d T = poses[i];
          for (int v = 0; v < color.rows; v+=2)
          {    for (int u = 0; u < color.cols; u+=2) 
              {
                  unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                  Eigen::Vector3d point;
                  point[2] = double(d) / depthScale;
                  if (point[2]== 0 || point[2] >1.0) continue;
                  point[0] = (u - cx[i]) * point[2] / fx[i];
                  point[1] = (v - cy[i]) * point[2] / fy[i];
                  Eigen::Vector3d pointWorld ;
                  // pointWorld = poses[i] * point;
                  PointT p;
                
                  p.x = point[0];
                  p.y = point[1];
                  p.z = point[2];
                  p.b = color.data[v * color.step + u * color.channels()];
                  p.g = color.data[v * color.step + u * color.channels() + 1];
                  p.r = color.data[v * color.step + u * color.channels() + 2];
                 
                      r4 = 255;
                      g4 = 0;
                      b4 = 0;
                      r5 = 0;
                      g5 = 0;
                      b5 = 255;
                 
                     if( (p.r == 0 && p.b == 0 && p.g == 0))
                        continue;
            
                    if( (p.r == r4 && p.b== b4 && p.g == g4))
                        current4->points.push_back(p);         
                    if( (p.r == r5 && p.b== b5 && p.g == g5))
                        current5->points.push_back(p);    
                  current1->points.push_back(p);
                 //// tao them 9 point ben canh
                  for(int h = -1; h < 2 ; h++)
                  {
                    for(int k = -1; k < 2; k++ )
                    {
                      for(int j = -1; j < 2; j++)
                      {
                        if( h==0 && k ==0 & j ==0)
                          continue;
                        PointT p2;
                        p2.x = p.x + 0.005*h;
                        p2.y = p.y + 0.005*k;
                        p2.z = p.z + 0.005*j;
                        p2.b = p.b;
                        p2.g = p.g;
                        p2.r = p.r;
                        current1->points.push_back(p2);
                        if( (p2.r == r4 && p2.b== b4 && p2.g == g4))
                          current4->points.push_back(p2);         
                        if( (p2.r == r5 && p2.b== b5 && p2.g == g5))
                          current5->points.push_back(p2);    
                      }
                    }
                  }
                
              }
          }

            // depth filter and statistical removal 
          //  PointCloud::Ptr tmp(new PointCloud);
            PointCloud::Ptr tmp21(new PointCloud);
            PointCloud::Ptr tmp24(new PointCloud);
            PointCloud::Ptr tmp25(new PointCloud);
            pcl::transformPointCloud (*current1, *tmp21, poses[i].matrix() );
            pcl::transformPointCloud (*current4, *tmp24, poses[i].matrix() );
            pcl::transformPointCloud (*current5, *tmp25, poses[i].matrix() );
            if(i == 0)
            {
                PointCloud::Ptr tmp11(new PointCloud);
                PointCloud::Ptr tmp14(new PointCloud);
                PointCloud::Ptr tmp15(new PointCloud);

                pcl::transformPointCloud (*tmp21, *tmp11, T4.matrix() );
                pcl::transformPointCloud (*tmp24, *tmp14, T4.matrix() );
                pcl::transformPointCloud (*tmp25, *tmp15, T4.matrix() );
                // PointCloud::Ptr tmp2(new PointCloud);
                // pcl::transformPointCloud (*tmp1, *tmp2, T5.matrix() );
                (*pointCloud) += *tmp11;
                (*sCloud) += *tmp14;
                (*bCloud) += *tmp15;
            }
            else
            {
                (*pointCloud) += *tmp21;
                (*sCloud) += *tmp24;
                (*bCloud) += *tmp25;
            }  
        }
        pointCloud->is_dense = false;
        sCloud->is_dense = false;
        bCloud->is_dense = false;
      //filter
      //int
        // voxel filter 
       
          // pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
          //    PointCloud::Ptr tmp1(new PointCloud);
          //       statistical_filter.setMeanK(50);
          //       statistical_filter.setStddevMulThresh(1.0);
          //       statistical_filter.setInputCloud(pointCloud);
          //       statistical_filter.filter(*tmp1);

        pcl::VoxelGrid<PointT> voxel_filter;
        double resolution = 0.005;
        voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
        voxel_filter.setInputCloud(pointCloud);
        voxel_filter.filter(*tmp);
        tmp->swap(*pointCloud);
        voxel_filter.setInputCloud(sCloud);
        voxel_filter.filter(*tmp);
        tmp->swap(*sCloud);
        voxel_filter.setInputCloud(bCloud);
        voxel_filter.filter(*tmp);
        tmp->swap(*bCloud);

        cout << "\n1cloud size" << pointCloud->size();
        cout << "\n1cloud size" << bCloud->size();
        cout << "\n1cloud size" << sCloud->size();
       
}

int main(int argc, char **argv) {
  init(colors);

  double cx[2];
  double cy[2];
  double fx[2];
  double fy[2];

  cx[1] =  	        318.60302734375;
  cy[1] =         	239.071395874023;
  fx[1] =          	385.441162109375;
  fy[1] =          	385.441162109375;

  cx[0] =  	        322.952789306641;
  cy[0] =         	238.483856201172;
  fx[0] =          	386.620269775391;
  fy[0] =          	386.620269775391;
  Isometry3d T0 = Isometry3d::Identity(); 

  
  Eigen::Matrix<double,3,3> R4;
  R4 <<   0.994091 , -0.0153871 ,   0.107452  ,
       0.0154333 ,   0.999881 , 0.000401526 ,
       -0.107446 , 0.00125919  ,   0.99421  ;

// 0.96128 ,-0.0302404 ,  0.273909 -0.0786335
//  0.0498496 ,  0.996645 ,-0.0649139 -0.0106248
//  -0.271027 , 0.0760547 ,  0.959562 -0.0326127

  Eigen::Matrix<double,3,1> t4(0.0689473 ,-0.0277824, -0.0211084obs);
  Quaterniond q4 = Quaterniond(R4);
  Eigen::Isometry3d T4(q4);
  T4.pretranslate(t4);

// T4
//  0.994091 , -0.0153871 ,   0.107452   0.0689473
//   0.0154333 ,   0.999881 , 0.000401526  -0.0277824
//   -0.107446 , 0.00125919  ,   0.99421  -0.0211084

//    0.999238  , 0.0360457 ,  0.0149858  ,-0.0061054
//  -0.0365761 ,   0.998654,   0.0367701, -0.00439986
//  -0.0136402 ,-0.0372902  ,  0.999211 0.000860977
// 0.999235  ,-0.0208464  , 0.0330721   -0.017823
//   0.0204637  ,   0.99972  ,  0.011867 -0.00269284
//  -0.0333103 , -0.0111811  ,  0.999383 -0.00037414
//   0.96128 , -0.0302404,   0.273909 -0.0786335
//  0.0498496 ,  0.996645, -0.0649139 -0.0106248
//  -0.271027 , 0.0760547 ,  0.959562 -0.0326127


    cv::Mat color1 = cv::imread("/home/giang/lab22/cam2/3.png",cv::IMREAD_UNCHANGED);
    cv::Mat color2 = cv::imread("/home/giang/lab22/cam2/4.png",cv::IMREAD_UNCHANGED);
    cv::Mat depth1 = cv::imread("/home/giang/lab22/cam3/3.png",cv::IMREAD_UNCHANGED);
    cv::Mat depth2 = cv::imread("/home/giang/lab22/cam3/4.png",cv::IMREAD_UNCHANGED);
  


    double depthScale = 1000;
    vector<cv::Mat> colorImgs, depthImgs;    
    vector<Eigen::Isometry3d> poses;     
    colorImgs.push_back(color1);   
    colorImgs.push_back(color2);
    depthImgs.push_back(depth1);
    depthImgs.push_back(depth2);
    poses.push_back(T0);
    poses.push_back(T0);

    const auto window_name = "Display Image"; 
    namedWindow(window_name, WINDOW_AUTOSIZE); 

    
    while (1) 
    { 
      PointCloud::Ptr tmp(new PointCloud);
      PointCloud::Ptr s_tmp(new PointCloud);
      PointCloud::Ptr b_tmp(new PointCloud);

      *tmp += *createtarget_colorpointcloud(color1,  depth1, T0,depthScale, cx[0],cy[0], fx[0], fy[0]);
      pcl::transformPointCloud (*tmp, *tmp, T4.matrix() );
      *tmp += *createtarget_colorpointcloud(color2,  depth2, T0,depthScale, cx[1],cy[1], fx[1], fy[1]);

      viewer.showCloud( tmp );
    //   create_3pcl(colorImgs, depthImgs, poses,  tmp ,s_tmp , b_tmp, T4);
   
    //   cout << "\n2cloud size" << tmp->size();
    //   cout << "lala";
    //   viewer.showCloud( tmp );
      cv::waitKey(0);
      viewer.showCloud( b_tmp );
      cv::waitKey(0);
      viewer.showCloud( b_tmp );
      List_cut_region::Ptr list = List_cut_region::Ptr(new List_cut_region);
      List_cut_region::Ptr list2 = List_cut_region::Ptr(new List_cut_region);
      char key = cv::waitKey(0);
      int count = count_component(b_tmp,list);
      cout <<"\nnumber of different components: " << count;
      viewer.showCloud( b_tmp );
      //list cut region
      int i = 0;
      double xm = 0,ym = 0,zm = 0;
      for ( i= 0; i< list->list_r.size(); i++)
      {
        cout <<"\n"<< i <<". (" << list->list_r[i].a1.x <<","<<list->list_r[i].a1.y <<"," <<list->list_r[i].a1.z <<" | "
            <<list->list_r[i].a2.x <<","<<list->list_r[i].a2.y <<"," <<list->list_r[i].a2.z <<" )";
        cout <<"\n    " << list->list_r[i].value;
        cout << "\n    mid " ;
        Mypoint mid = list->list_r[i].a1.get_middle(list->list_r[i].a2);
        cout << "(" <<mid.x << "," <<mid.y <<"," << mid.z <<")";
      }


      cv::waitKey(0);
      viewer.showCloud( s_tmp );
      count = count_component(s_tmp,list2);
      cout <<"\nnumber of different components: " << count;
      viewer.showCloud( s_tmp );
      i = 0;
      
      for ( i= 0; i< list2->list_r.size(); i++)
      {
        cout <<"\n"<< i <<". (" << list2->list_r[i].a1.x <<","<<list2->list_r[i].a1.y <<"," <<list2->list_r[i].a1.z <<" | "
            <<list2->list_r[i].a2.x <<","<<list2->list_r[i].a2.y <<"," <<list2->list_r[i].a2.z <<" )";
        cout <<"\n    " << list2->list_r[i].value;
        cout << "\n    mid " ;
        Mypoint mid = list2->list_r[i].a1.get_middle(list2->list_r[i].a2);
        cout << "(" <<mid.x << "," <<mid.y <<"," << mid.z <<")";
      }

      key = cv::waitKey(0);
    
      if(key =='s')
        pcl::io::savePCDFileBinary("map3.pcd", *tmp);
    
      if( key == 'o')
        break;
   
    }
  
    
    return 0;
}