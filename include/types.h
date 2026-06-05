#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>



// Define a custom point type that includes the necessary fields
struct CustomPointType
{
    PCL_ADD_POINT4D; 

    float intensity;
    int occu_times;
    int is_occu_times;
    int hor_ind;
    int ver_ind;
    int dyn_flag; // Renamed from 'dyn' to avoid potential conflicts
    float custom_curvature; // Renamed from 'curvature' for clarity

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new point type is aligned
} ;

POINT_CLOUD_REGISTER_POINT_STRUCT (CustomPointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (int, occu_times, occu_times)
    (int, is_occu_times, is_occu_times)
    (int, hor_ind, hor_ind)
    (int, ver_ind, ver_ind)
    (int, dyn_flag, dyn_flag)
    (float, custom_curvature, custom_curvature)
)

typedef CustomPointType PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Eigen::Vector2f V2F;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>



#endif