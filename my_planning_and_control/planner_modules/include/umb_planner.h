#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <unordered_map>
#include <glog/logging.h>
#include <iomanip>
#include <fstream>
#include <string>
#include "umbp_config.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "common.h"
#include "fpb_tree.h"
#include "tic_toc.h"
#include "polynomial_curve.h"
#include "derived_object_msgs/msg/object.hpp"
#include "carla_waypoint_types/srv/get_waypoint.hpp"
#include "std_msgs/msg/float64.hpp"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include "matplot/matplot.h"

class UMBPlanner : public rclcpp::Node
{
public:
    using FpbAction = FpbTree::FpbAction;
    using FpbLonAction = FpbTree::FpbLonAction;
    using FpbLatAction = FpbTree::FpbLatAction;
    using Cfg = planning::umbp::Config;

    struct ForwardPropAgent
    {
        int id = 0;
        double time_from_start = 1.0;
        FrenetPoint obs_frenet_point;
        double width = 3.0;
        double length = 5.0;
    };

    struct ForwardPropAgentSet
    {
        std::unordered_map<int, ForwardPropAgent> forward_prop_agents;
    };

    struct SimParam
    {
        double layer_time = 1.0;
        double step = 0.2;
        double tree_height = 1.0;
        double s_sample_distance = 5.0;
        int s_sample_num = 12;
        double l_sample_distance = 1.0;
        int l_sample_num = 8;
        double acc_ref = 3.0;
        double dec_ref = 4.0;
    };

    struct CostParam
    {
        double ego_to_obs = 10.0;
        double ref_line_change = 2.0;
        double discount_factor = 0.8;
        double ego_lack_speed_to_desired_unit_cost = 0.3;
    };

    struct EfficiencyCost
    {
        double ego_to_desired_vel = 0.0;
        double leading_to_desired_vel = 0.0;
        double ave() const
        {
            return (ego_to_desired_vel + leading_to_desired_vel) / 1.0;
        }
    };

    struct SafetyCost
    {
        double ego_to_obs = 0.0;
        double ave() const
        {
            return ego_to_obs;
        }
    };

    struct NavigationCost
    {
        double ref_line_change = 0.0;
        double ave() const
        {
            return ref_line_change;
        }
    };

    struct CostStructure
    {
        // * associate cost with micro action using this index
        int behaviour_index;
        // * efficiency
        EfficiencyCost efficiency;
        // * safety
        SafetyCost safety;
        // * navigation
        NavigationCost navigation;
        double cur_behav_weight = 1.0;
        double efficiency_weight = 1.0;
        double safety_weight = 1.0;
        double navigation_weight = 1.0;
        double ave() const
        {
            double SumCost = efficiency.ave() * efficiency_weight + safety.ave() * safety_weight + navigation.ave() * navigation_weight;

            return SumCost * cur_behav_weight;
        }
    };

    UMBPlanner();

    bool RunOnce(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state,
                 const std::vector<derived_object_msgs::msg::Object> &objects, std::vector<TrajectoryPoint> &trajectory);

    bool RunUmpb(const FrenetPoint ego_state, const ForwardPropAgentSet &forward_prop_agent_set);

    void ObstacleFileter(const std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object> &objects);

    TrajectoryPoint CalculatePlanningStartPoint(std::shared_ptr<VehicleState> ego_state);

    void GetFrenetSamplePoints(std::vector<std::vector<FrenetPoint>> &frenet_sample_point, const FrenetPoint &planning_start_sl,
                               const double &s_sample_distance, const int &s_sample_num,
                               const double &l_sample_distance, const int &l_sample_num);

private:
    bool ReadConfig(const std::string config_path);

    bool GetSimParam(const planning::umbp::Config &cfg,
                     SimParam *sim_param);

    bool GetCostParam(const planning::umbp::Config &cfg,
                      CostParam *cost_param);

    bool GetSurroundingForwardSimAgents(ForwardPropAgentSet &forward_prop_agents,
                                        const std::vector<FrenetPoint> static_obs_frent_coords,
                                        const std::vector<FrenetPoint> dynamic_obs_frent_coords);
    bool PrepareMultiThreadContainers(const int num_sequence);

    bool PropagateActionSequence(const FrenetPoint ego_state,
                                 const ForwardPropAgentSet &surrounding_fsagents,
                                 const std::vector<FpbAction> &action_seq, const int &seq_id);

    bool PropagateScenario(
        const FrenetPoint &ego_state,
        const ForwardPropAgentSet &surrounding_fsagents,
        const std::vector<FpbAction> &action_seq, const int &seq_id,
        const int &sub_seq_id, int *sub_sim_res,
        int *sub_risky_res, std::string *sub_sim_info,
        std::vector<CostStructure> *sub_progress_cost,
        CostStructure *sub_tail_cost,
        std::vector<FrenetPoint> *sub_forward_trajs,
        std::vector<FpbLatAction> *sub_forward_lat_behaviors,
        std::vector<FpbLonAction> *sub_forward_lon_behaviors,
        std::unordered_map<int, std::vector<FrenetPoint>> *sub_surround_trajs);

    bool CalculateCost(const std::vector<FrenetPoint> &forward_trajs,
                       const std::unordered_map<int, std::vector<FrenetPoint>> &surround_trajs,
                       std::vector<CostStructure> *cost, bool *is_risky, std::set<size_t> *risky_ids);

    bool EvaluateMultiThreadSimResults(int *winner_id, double *winner_cost);

    bool IncreaseFpbPath(const std::vector<FrenetPoint> &fpb_path_init, const double &increased_distance,
                         std::vector<FrenetPoint> &fpb_path_increased);

    bool IncreaseFpbSpeedProfile(const std::vector<FrenetPoint> &fpb_path, const double &increased_time, const double &layer_time,
                                 std::vector<STPoint> &speed_profile_increased);

    bool GenerateTrajectory(const std::vector<STPoint> &final_speed_profile, const std::vector<TrajectoryPoint> &path_trajectory,
                            const std::vector<double> &path_index2s, const double &planning_start_point_time_stamped,
                            std::vector<TrajectoryPoint> &trajectory);

    std::vector<derived_object_msgs::msg::Object> _static_obstacles;  // 静态障碍物
    std::vector<derived_object_msgs::msg::Object> _dynamic_obstacles; // 动态障碍物
    std::deque<TrajectoryPoint> _previous_trajectory;                 // 上一周期的轨迹
    std::deque<TrajectoryPoint> _switch_trajectory;                   // 拼接轨迹

    rclcpp::Client<carla_waypoint_types::srv::GetWaypoint>::SharedPtr _get_waypoint_client;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _reference_speed_subscriber;

    double _reference_speed;
    double _current_time;
    std::shared_ptr<VehicleState> _current_ego_state;

    // behaviour
    std::shared_ptr<FpbTree> _fpb_tree_ptr;

    // Config
    Cfg _cfg;
    SimParam _sim_param;
    CostParam _cost_param;

    // sample_points
    std::vector<std::vector<FrenetPoint>> _local_sample_points;
    int _increased_sl_sample_num;

    // result
    int _winner_id = 0;
    double _winner_score = 0.0;
    std::vector<FpbAction> _winner_action_seq;
    std::vector<int> _sim_res = std::vector<int>(64);
    std::vector<int> _risky_res = std::vector<int>(64);
    std::vector<std::string> _sim_info = std::vector<std::string>(64);
    std::vector<double> _final_cost = std::vector<double>(64);
    std::vector<std::vector<CostStructure>> _progress_cost = std::vector<std::vector<CostStructure>>(64);
    std::vector<CostStructure> _tail_cost = std::vector<CostStructure>(64);
    std::vector<std::vector<FrenetPoint>> _forward_trajs = std::vector<std::vector<FrenetPoint>>(64);
    std::vector<std::vector<FpbLatAction>> _forward_lat_behaviors = std::vector<std::vector<FpbLatAction>>(64);
    std::vector<std::vector<FpbLonAction>> _forward_lon_behaviors = std::vector<std::vector<FpbLonAction>>(64);
    std::vector<std::unordered_map<int, std::vector<FrenetPoint>>> _surround_trajs = std::vector<std::unordered_map<int, std::vector<FrenetPoint>>>(64);
    double _time_cost = 0.0;

    // plot
    matplot::figure_handle _path_plot_handle;
    size_t _plot_count;
    matplot::figure_handle _trajectory_plot_handle;
    matplot::figure_handle _STL_plot_handle;
};