#include <fins/node.hpp>
#include <omp.h>
#include <mutex>
#include <deque>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "types.h"
#include "m-detector/DynObjFilter.h"

using namespace std;

class DynFilterOdomNode : public fins::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~DynFilterOdomNode() {
        run_timer_ = false;
        if (timer_thread_.joinable()) {
            timer_thread_.join();
        }
    }

    void define() override {
        set_name("M_Detector");
        set_category("SLAM");

        register_input<sensor_msgs::msg::PointCloud2>("points", &DynFilterOdomNode::on_points_callback);
        register_input<nav_msgs::msg::Odometry>("odom", &DynFilterOdomNode::on_odom_callback);

        register_output<sensor_msgs::msg::PointCloud2>("clustered_dynamic_points");
        register_output<sensor_msgs::msg::PointCloud2>("raw_dynamic_points");
        register_output<sensor_msgs::msg::PointCloud2>("static_background");
    }

    void initialize() override {
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        dyn_obj_filt_ = make_shared<DynObjFilter>();
        dyn_obj_filt_->init();

        run_timer_ = true;
        timer_thread_ = std::thread([this]() {
            while (run_timer_) {
                this->on_timer_callback();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

private:
    void on_odom_callback(const nav_msgs::msg::Odometry& msg, fins::AcqTime ts) {
        lock_guard<mutex> lock(mtx_buffer_);
        
        Eigen::Quaterniond cur_q(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        );
        
        M3D cur_rot = cur_q.matrix();
        V3D cur_pos;
        cur_pos << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
        
        buffer_rots_.push_back(cur_rot);
        buffer_poss_.push_back(cur_pos);
        
        double odom_time = fins::to_seconds(ts);
        buffer_times_.push_back(odom_time);
        
        if (buffer_times_.size() > 1000) {
            buffer_rots_.pop_front();
            buffer_poss_.pop_front();
            buffer_times_.pop_front();
        }
    }

    void on_points_callback(const sensor_msgs::msg::PointCloud2& msg, fins::AcqTime ts) {
        shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
        pcl::fromROSMsg(msg, *feats_undistort);
        
        lock_guard<mutex> lock(mtx_buffer_);
        buffer_pcs_.push_back(feats_undistort); 
        buffer_pc_times_.push_back(fins::to_seconds(ts));
        buffer_pc_acq_times_.push_back(ts);

        if (buffer_pcs_.size() > 100) {
            buffer_pcs_.pop_front();
            buffer_pc_times_.pop_front();
            buffer_pc_acq_times_.pop_front();
        }
    }

    void on_timer_callback() {
        shared_ptr<PointCloudXYZI> cur_pc;
        M3D cur_rot_match;
        V3D cur_pos_match;
        double cur_time_match;
        fins::AcqTime cur_acq_time_match;
        bool found_match = false;

        {
            lock_guard<mutex> lock(mtx_buffer_);
            if (!buffer_pcs_.empty() && !buffer_times_.empty()) {
                double pc_time = buffer_pc_times_.at(0);
                
                int closest_idx = -1;
                double min_diff = 1e9;
                
                for (int i = 0; i < (int)buffer_times_.size(); i++) {
                    double diff = fabs(buffer_times_[i] - pc_time);
                    if (diff < min_diff) {
                        min_diff = diff;
                        closest_idx = i;
                    }
                }
                
                if (closest_idx != -1 && min_diff < 0.1) {
                    cur_pc = buffer_pcs_.at(0);
                    cur_acq_time_match = buffer_pc_acq_times_.at(0);
                    cur_rot_match = buffer_rots_.at(closest_idx);
                    cur_pos_match = buffer_poss_.at(closest_idx);
                    cur_time_match = buffer_times_.at(closest_idx);
                    found_match = true;

                    buffer_pcs_.pop_front();
                    buffer_pc_times_.pop_front();
                    buffer_pc_acq_times_.pop_front();
                    
                    for (int i = 0; i < closest_idx; i++) {
                        buffer_rots_.pop_front();
                        buffer_poss_.pop_front();
                        buffer_times_.pop_front();
                    }
                }
            }
        }

        if (found_match) {
            dyn_obj_filt_->filter(cur_pc, cur_rot_match, cur_pos_match, cur_time_match);
            dyn_obj_filt_->publish_dyn(this, cur_time_match);
            cur_frame_++;
        }
    }

    int cur_frame_ = 0;

    mutex mtx_buffer_;
    deque<M3D> buffer_rots_;
    deque<V3D> buffer_poss_;
    deque<double> buffer_times_;
    deque<double> buffer_pc_times_;
    deque<fins::AcqTime> buffer_pc_acq_times_;
    deque<shared_ptr<PointCloudXYZI>> buffer_pcs_;

    std::atomic<bool> run_timer_{false};
    std::thread timer_thread_;
    shared_ptr<DynObjFilter> dyn_obj_filt_;
};

EXPORT_NODE(DynFilterOdomNode)
DEFINE_PLUGIN_ENTRY(fins::STATELESS)