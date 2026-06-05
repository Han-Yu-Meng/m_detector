#ifndef VOXEL_CLUSTER_H
#define VOXEL_CLUSTER_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define HASH_length     10000
class VOXEL {
public:
    int64_t x, y, z;

    VOXEL(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL &other) const {
        return (this->x == other.x && this->y == other.y && this->z == other.z);
    }
};
typedef pcl::PointXYZINormal PointType;

struct Point_Cloud {
    typedef std::shared_ptr<Point_Cloud> Ptr;
    int bbox_index{-1};
    int points_num{0};
    
    pcl::PointCloud<PointType>* cloud{nullptr};
    std::vector<int>* cloud_index{nullptr};

    Point_Cloud(PointType point, int index)
    {
        this->cloud = new pcl::PointCloud<PointType>();
        this->cloud->points.push_back(point);
        this->bbox_index = index;
    }

    Point_Cloud(int index)
    {
        this->bbox_index = index;
    }
    
    Point_Cloud() = default; // 显式默认，确保为 trivial

    Point_Cloud(PointType point)
    {   
        this->cloud = new pcl::PointCloud<PointType>();
        this->cloud->reserve(5);
        this->cloud->points.push_back(point);
    }

    ~Point_Cloud() = default; // 显式默认，防止 resize 时单线程析构带来的性能损耗

    void reset()
    {   
        if (cloud) {
            delete cloud;
            cloud = nullptr;
        }
        if (cloud_index) {
            delete cloud_index;
            cloud_index = nullptr;
        }
        this->points_num = 0;
        this->bbox_index = -1;
    }
};

struct test_struct{
    int bbox_index{-1};
    int points_num{0};
    pcl::PointCloud<PointType>::Ptr cloud;
    std::vector<int>* cloud_index;

    
    test_struct(int index)
    {
        bbox_index = index;
    }
    
    test_struct(){
        bbox_index = -1;
        points_num = 0;
    }

    ~test_struct(){};

    void reset()
    {   
        this->points_num = 0;
        this->bbox_index = -1;
    };

};
// Hash value
namespace std {
template <> struct hash<VOXEL> {
  int64_t operator()(const VOXEL &s) const {
    using std::hash;
    using std::size_t;
    return (s.z * HASH_length * HASH_length + s.y * HASH_length + s.x);
  }
};
}

class VOXEL_CLUSTER
{
public:
    typedef pcl::PointXYZINormal PointType;
    std::vector<int> voxel_list;
    std::unordered_set<int> voxel_set;

    VOXEL_CLUSTER(){};
    ~VOXEL_CLUSTER(){};
    void setInputCloud(const pcl::PointCloud<PointType> &points_in)
    {
        points_ = points_in;
    }
    
    void setVoxelResolution(float voxel_length, float edge_size_xy, float edge_size_z, const Eigen::Vector3f &xyz_origin_in)
    {
        Voxel_revolusion = voxel_length;
        Grid_edge_size_xy = edge_size_xy;
        Grid_edge_size_z = edge_size_z;
        xyz_origin = xyz_origin_in;
    }

    void setExtendRange(int range)
    {
        max_range = range;
    }

    void setMinClusterSize(int min_cluster_voxels)
    {
        min_cluster_voxels_ = min_cluster_voxels;
    }

    void createVoxelMap(std::vector<Point_Cloud>  &umap_in, std::unordered_set<int> &used_map_set)
    {   
        for (int i = 0; i < points_.size(); i++)
        {
            PointType tmp;
            tmp.x = points_.points[i].x;
            tmp.y = points_.points[i].y;
            tmp.z = points_.points[i].z;
            tmp.intensity = 0;
            int position = floor((tmp.x - xyz_origin(0))/Voxel_revolusion) * Grid_edge_size_xy * Grid_edge_size_z + floor((tmp.y - xyz_origin(1))/Voxel_revolusion) * Grid_edge_size_z + floor((tmp.z- xyz_origin(2))/Voxel_revolusion);
            if(position < 0 || position > Grid_edge_size_xy* Grid_edge_size_xy* Grid_edge_size_z) continue;
            if (umap_in[position].points_num > 0)
            {
                umap_in[position].points_num = umap_in[position].points_num + 1;
                continue;
            }
            else
            {   
                used_map_set.insert(position);
                voxel_list.push_back(position);
                voxel_set.insert(position);
                umap_in[position].cloud = new pcl::PointCloud<PointType>();
                umap_in[position].cloud_index = new std::vector<int>();
                umap_in[position].points_num = 1;
            }    
        }
    }

    void createVoxelMap(std::vector<Point_Cloud> &umap_in, bool index_en)
    {   
        for (int i = 0; i < points_.size(); i++)
        {
            PointType tmp;
            tmp.x = points_.points[i].x;
            tmp.y = points_.points[i].y;
            tmp.z = points_.points[i].z;
            tmp.intensity = 0;
            int position = floor((tmp.x - xyz_origin(0))/Voxel_revolusion) * Grid_edge_size_xy * Grid_edge_size_z + floor((tmp.y - xyz_origin(1))/Voxel_revolusion) * Grid_edge_size_z + floor((tmp.z - xyz_origin(2))/Voxel_revolusion);
            if(position < 0 || position > Grid_edge_size_xy* Grid_edge_size_xy* Grid_edge_size_z) continue;
            if (umap_in[position].points_num > 0)
            {   
                umap_in[position].cloud->push_back(tmp);
                umap_in[position].points_num = umap_in[position].points_num + 1;
                umap_in[position].cloud_index->push_back(i);
                continue;
            }
            else
            {   
                voxel_list.push_back(position);
                voxel_set.insert(position);
                umap_in[position].cloud = new pcl::PointCloud<PointType>();
                umap_in[position].cloud->reserve(5);
                umap_in[position].cloud->push_back(tmp);
                umap_in[position].points_num = 1;
                umap_in[position].cloud_index = new std::vector<int>();
                umap_in[position].cloud_index->reserve(5);
                umap_in[position].cloud_index->push_back(i);
            }    
        }
    }

    void createVoxelMap(std::unordered_map<int, Point_Cloud::Ptr> &umap)
    {
        for (int i = 0; i < points_.size(); i++)
        {
            PointType tmp;
            tmp.x = points_.points[i].x;
            tmp.y = points_.points[i].y;
            tmp.z = points_.points[i].z;
            int position = floor((tmp.x - xyz_origin(0))/Voxel_revolusion) * Grid_edge_size_xy * Grid_edge_size_z + floor((tmp.y - xyz_origin(1))/Voxel_revolusion) * Grid_edge_size_z + floor((tmp.z - xyz_origin(2))/Voxel_revolusion);
            if(position < 0 || position > Grid_edge_size_xy* Grid_edge_size_xy* Grid_edge_size_z) continue;
            if (umap.count(position))
            {   
                umap[position]->cloud->push_back(tmp);
                umap[position]->points_num = umap[position]->points_num + 1;
                umap[position]->cloud_index->push_back(i);
                continue;
            }
            else
            {   
                voxel_list.push_back(position);
                voxel_set.insert(position);
                umap[position].reset(new Point_Cloud(tmp));
                umap[position]->points_num = 1;
                umap[position]->cloud_index = new std::vector<int>(); 
                umap[position]->cloud_index->reserve(5);
                umap[position]->cloud_index->push_back(i);
            }    
        }
    }

    void extendVoxelNeighbor(int voxel, std::unordered_set<int> &voxel_added)
    {
        for (int x_neighbor = -max_range; x_neighbor <= max_range; x_neighbor++)
        {
            for (int y_neighbor = -max_range; y_neighbor <= max_range; y_neighbor++)
            {
                for (int z_neighbor = -max_range; z_neighbor <= max_range; z_neighbor++)
                {
                    float neighbor_dis = sqrt(x_neighbor * x_neighbor + y_neighbor * y_neighbor + z_neighbor * z_neighbor);
                    if (neighbor_dis - (float) max_range < 0.001f)
                    {
                        int voxel_neighbor = voxel + x_neighbor * Grid_edge_size_xy*Grid_edge_size_z + y_neighbor * Grid_edge_size_z + z_neighbor;
                        if(voxel_neighbor < 0 || voxel_neighbor > Grid_edge_size_xy* Grid_edge_size_xy* Grid_edge_size_z) continue;
                        if (voxel_set.count(voxel_neighbor) && !voxel_added.count(voxel_neighbor))
                        {   
                            voxel_added.insert(voxel_neighbor);
                            extendVoxelNeighbor(voxel_neighbor, voxel_added);
                        }
                    }
                }
            }
        }
    }
    
    void extract(std::vector<std::vector<int>> &voxel_clusters)
    {   
        for (int voxel_ind = 0; voxel_ind < voxel_list.size(); voxel_ind++)
        {   
            int voxel_cur = voxel_list[voxel_ind];
            if (voxel_set.count(voxel_cur))
            {
                std::unordered_set<int> voxel_added;
                voxel_added.emplace(voxel_cur);
                extendVoxelNeighbor(voxel_cur, voxel_added);
                int size = 0;
                std::vector<int> voxel_candidate_vec;
                for(auto iter=voxel_added.begin(); iter!=voxel_added.end(); ++iter)
                {
                    voxel_set.erase(*iter);
                    voxel_candidate_vec.push_back(*iter);
                    size++;
                }
                if (size >= min_cluster_voxels_)
                {
                    voxel_clusters.push_back(voxel_candidate_vec);
                }
            }
            else
            {
                continue;
            } 
        }
    }

protected:
    pcl::PointCloud<PointType> points_;
    float Voxel_revolusion;
    float Grid_edge_size_xy;
    float Grid_edge_size_z;
    Eigen::Vector3f xyz_origin;
    std::vector<Point_Cloud> umap;
    int max_range;
    int min_cluster_voxels_;
};

#endif