#include "umb_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
auto UMBP_LOG = rclcpp::get_logger("UMBPlanner");

UMBPlanner::UMBPlanner() : Node("UMBPlanner")
{
    google::InitGoogleLogging("UMBP");
    google::SetLogDestination(google::GLOG_INFO, "/home/guzizhen/PNC-CARLA/debug_log/umbp_log/");
    google::SetLogDestination(google::GLOG_WARNING, "/home/guzizhen/PNC-CARLA/debug_log/umbp_log/");
    google::SetLogDestination(google::GLOG_ERROR, "/home/guzizhen/PNC-CARLA/debug_log/umbp_log/");

    _get_waypoint_client = this->create_client<carla_waypoint_types::srv::GetWaypoint>(
        "/carla_waypoint_publisher/ego_vehicle/get_waypoint");

    std::string config_path = "/home/guzizhen/PNC-CARLA/my_planning_and_control/planner_modules/config/umbp_config.pb.txt";
    ReadConfig(config_path);
    GetSimParam(_cfg, &_sim_param);
    GetCostParam(_cfg, &_cost_param);
    GetEgoParam(_cfg, &_ego_param);
    GetBezierParam(_cfg, &_bezier_param);

    _reference_speed = _ego_param.reference_speed;
    _fpb_tree_ptr = std::make_shared<FpbTree>(_sim_param.tree_height, _sim_param.layer_time);

    FLAGS_log_prefix = false;
    LOG(INFO) << std::fixed << std::setprecision(4)
              << "[UMBP]******************** RUN START: " << "******************";
    LOG(INFO) << std::fixed << std::setprecision(4)
              << "[UMBP]*************** FPB_Tree Height: " << _fpb_tree_ptr->get_behaviour_tree().size() << "******************";
}

bool UMBPlanner::ReadConfig(const std::string config_path)
{
    LOG(INFO) << " [UMBP] Loading umbp planner config";
    using namespace google::protobuf;
    int fd = open(config_path.c_str(), O_RDONLY);
    io::FileInputStream fstream(fd);
    TextFormat::Parse(&fstream, &_cfg);
    if (!_cfg.IsInitialized())
    {
        LOG(ERROR) << "[UMBP]failed to parse config from " << config_path;
        assert(false);
    }
    return true;
}

bool UMBPlanner::GetSimParam(const planning::umbp::Config &cfg, SimParam *sim_param)
{
    sim_param->layer_time = cfg.propogate().fpb().layer_time();
    sim_param->step = cfg.propogate().fpb().step();
    sim_param->tree_height = cfg.propogate().fpb().tree_height();
    sim_param->tree_height = cfg.propogate().fpb().tree_height();
    sim_param->l_ref_to_left_road_bound = cfg.propogate().fpb().l_ref_to_left_road_bound();
    sim_param->l_ref_to_right_road_bound = cfg.propogate().fpb().l_ref_to_right_road_bound();

    sim_param->s_sample_distance = cfg.propogate().sample().s_sample_distance();
    sim_param->s_sample_num = cfg.propogate().sample().s_sample_num();
    sim_param->l_sample_distance = cfg.propogate().sample().l_sample_distance();
    sim_param->l_sample_num = cfg.propogate().sample().l_sample_num();
    sim_param->acc_ref = cfg.propogate().sample().acc_ref();
    sim_param->dec_ref = cfg.propogate().sample().dec_ref();
    sim_param->increased_sl_distance = cfg.propogate().sample().increased_sl_distance();
    sim_param->increased_st_distance = cfg.propogate().sample().increased_st_distance();
    return true;
}

bool UMBPlanner::GetCostParam(const planning::umbp::Config &cfg,
                              CostParam *cost_param)
{
    cost_param->ego_lack_speed_to_desired_unit_cost = cfg.cost().efficiency().ego_lack_speed_to_desired_unit_cost();
    cost_param->ego_to_obs = cfg.cost().safety().ego_to_obs();
    cost_param->ref_line_change = cfg.cost().navigation().ref_line_change();
    cost_param->hdi_weight = cfg.cost().hdi().hdi_weight();
    cost_param->hdi_win_size = cfg.cost().hdi().hdi_win_size();
    return true;
}

bool UMBPlanner::GetEgoParam(const planning::umbp::Config &cfg,
                             EgoParam *ego_param)
{
    ego_param->car_length = cfg.ego().car().car_length();
    ego_param->car_width = cfg.ego().car().car_width();
    ego_param->reference_speed = cfg.ego().map().reference_speed();
    return true;
}

bool UMBPlanner::GetBezierParam(const planning::umbp::Config &cfg,
                                BezierParam *bezier_param)
{
    bezier_param->weight_P = cfg.bezier().weight().weight_p();
    bezier_param->weight_c = cfg.bezier().weight().weight_c();
    bezier_param->v_lb_s = cfg.bezier().cube().v_lb_s();
    bezier_param->v_lb_l = cfg.bezier().cube().v_lb_l();
    bezier_param->v_ub_s = cfg.bezier().cube().v_ub_s();
    bezier_param->v_ub_l = cfg.bezier().cube().v_ub_l();
    bezier_param->a_lb_s = cfg.bezier().cube().a_lb_s();
    bezier_param->a_lb_l = cfg.bezier().cube().a_lb_l();
    bezier_param->a_ub_s = cfg.bezier().cube().a_ub_s();
    bezier_param->a_ub_l = cfg.bezier().cube().a_ub_l();
    return true;
}

bool UMBPlanner::RunOnce(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state,
                         const std::vector<derived_object_msgs::msg::Object> &obstacles, std::vector<TrajectoryPoint> &final_trajectory, bool &emergency_stop_signal)
{
    emergency_stop_signal = false;
    _current_time = this->now().seconds();
    _current_ego_state = ego_state;
    //-----------------------------------1.障碍物处理--------------------------------------
    // 1.1区别动态与静态障碍物，将其储存在成员变量 std::vector<derived_object_msgs::msg::Object> _static_/dynamic_obstacles中
    UMBPlanner::ObstacleFileter(ego_state, obstacles);

    // 1.2 障碍物转换为前向传播智能体
    ForwardPropAgentSet forward_prop_agent_set;
    if (!GetSurroundingForwardSimAgents(forward_prop_agent_set, reference_line, ego_state))
    {
        LOG(ERROR) << "GetSurroundingForwardSimAgents False! ";
        return false;
    }
    else
    {
        for (const auto &pair : forward_prop_agent_set.forward_prop_agents)
        {
            const auto &agent = pair.second;
            RCLCPP_INFO(UMBP_LOG, "Agent_ID %.i: (s: %.3f,l: %.3f,s_dot: %.3f,l_dot: %.3f)",
                        agent.id, agent.obs_frenet_point.s, agent.obs_frenet_point.l,
                        agent.obs_frenet_point.s_dot, agent.obs_frenet_point.l_dot);
        }
    }

    // 1.3 预测障碍物轨迹
    TicToc Predict_timer;
    if (!PredictTrajectoryDynamicObs(reference_line, ego_state))
    {
        LOG(ERROR) << "PredictTrajectoryDynamicObs False! ";
        return false;
    }
    LOG(INFO) << std::fixed << std::setprecision(4)
              << "[UMBP][Process] PredictTrajectory Time :"
              << Predict_timer.toc() << "ms";

    //----------------------------2.确定规划起点并将其投影到frenet坐标系--------------------------
    // RCLCPP_INFO(this->get_logger(), "Start CalculatePlanningStartPoint !!!");
    TrajectoryPoint planning_start_point = CalculatePlanningStartPoint(ego_state);
    std::vector<FrenetPoint> temp_set;
    cartesion_set_to_frenet_set(planning_start_point, *reference_line, ego_state, temp_set);
    FrenetPoint planning_start_point_frenet = temp_set.front();
    planning_start_point_frenet.l_prime_prime = 0.0;
    LOG(INFO) << std::fixed << std::setprecision(2) << " [UMBP][process] " << "planning_start_point_frenet (s: "
              << planning_start_point_frenet.s << ", l: " << planning_start_point_frenet.l << ")";

    //----------------------------3.进行Frenet采样--------------------------
    std::vector<std::vector<FrenetPoint>> frenet_sample_point;
    GetFrenetSamplePoints(frenet_sample_point, planning_start_point_frenet,
                          _sim_param.s_sample_distance, _sim_param.s_sample_num,
                          _sim_param.l_sample_distance, _sim_param.l_sample_num);
    _local_sample_points = frenet_sample_point;

    //----------------------------4.进行FPB-Tree前向传播--------------------------
    // RCLCPP_INFO(this->get_logger(), "Start RunUmpb !!!");
    TicToc umbp_timer;
    if (!RunUmpb(planning_start_point_frenet, forward_prop_agent_set))
    {
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[UMBP]****** UMBP Cycle FAILED  ******";
        emergency_stop_signal = true;
        return true;
    }
    LOG(INFO) << std::fixed << std::setprecision(4)
              << "[UMBP][Process] RunUmpb Time :"
              << umbp_timer.toc() << "ms";

    std::vector<FrenetPoint> fpb_path = _forward_trajs[_winner_id];

    if ((int)_history_decision.size() >= (int)_cost_param.hdi_win_size)
    {
        _history_decision.pop_front();
    }
    int record_history = 0;
    if (record_history % 2 == 0)
    {
        _history_decision.push_back(fpb_path);
        record_history++;
    }

    if (fpb_path[0].s == fpb_path[1].s)
    {
        emergency_stop_signal = true;
        RCLCPP_INFO(UMBP_LOG, "---------planning moudle emergency_stop_signal!!! -------");
        return true;
    }

    //-------------------------------------5.时空联合规划----------------------------------------
    // 5.1 对决策的SL曲线进行加密
    std::vector<FrenetPoint> fpb_path_increased;
    // int beszier_num_points = 50;
    // _increased_sl_sample_num = static_cast<int>(beszier_num_points / 5);
    // fpb_path_increased = GenerateBezierCurve(fpb_path, static_cast<double>((fpb_path.size() - 1) * _sim_param.layer_time), beszier_num_points);
    // RCLCPP_INFO(this->get_logger(), "Start IncreaseFpbPath !!!");
    if (!IncreaseFpbPath(fpb_path, _sim_param.increased_sl_distance, fpb_path_increased))
    {
        LOG(INFO) << "[UMBP]****** IncreaseFpbPath FAILED  ******";
        emergency_stop_signal = true;
        return true;
    }

    // for (const auto &points : fpb_path_increased)
    // {
    //     LOG(INFO) << std::fixed << std::setprecision(3)
    //               << "fpb_path_increased: (t: " << points.t << " s: " << points.s
    //               << " l: " << points.l << ")";
    // }
    // LOG(INFO) << "----------------------------------";

    // 5.2 将决策SL曲线转换为轨迹
    auto reference_index2s = calculate_index_to_s(reference_line, ego_state);
    auto fpb_path_trajectory = frenet_to_cartesion(fpb_path_increased, reference_line, reference_index2s);

    // 5.3 生成SSC CUBE
    // RCLCPP_INFO(this->get_logger(), "Start GenerateSscCube !!!");
    TicToc generate_cube_timer;
    if (!GenerateSscCube(fpb_path, forward_prop_agent_set, fpb_path_trajectory, planning_start_point_frenet))
    {
        LOG(INFO) << "[UMBP]****** GenerateSscCube FAILED  ******";
        emergency_stop_signal = true;
        return false;
    }
    // LOG(INFO) << std::fixed << std::setprecision(4)
    //           << "[UMBP][Process] GenerateSscCube Time :"
    //           << generate_cube_timer.toc() << "ms";

    // 5.4 求解BezierSpline
    SplineGenerator<5, 2> spline_generator;
    BezierSpline<5, 2> bezier_spline;

    // get start and end constraints
    vec_E<Vecf<2>> start_constraints;
    vec_E<Vecf<2>> end_constraints;
    start_constraints.push_back(Vecf<2>(fpb_path[0].s, fpb_path[0].l));
    start_constraints.push_back(Vecf<2>(fpb_path[0].s_dot, fpb_path[0].l_dot));
    start_constraints.push_back(Vecf<2>(fpb_path[0].s_dot_dot, fpb_path[0].l_dot_dot));
    end_constraints.push_back(Vecf<2>(fpb_path.back().s, fpb_path.back().l));
    end_constraints.push_back(Vecf<2>(fpb_path.back().s_dot, fpb_path.back().l_dot));
    end_constraints.push_back(Vecf<2>(fpb_path.back().s_dot_dot, fpb_path.back().l_dot_dot));

    // get ref points
    std::vector<decimal_t> ref_stamps;
    vec_E<Vecf<2>> ref_points;
    for (size_t i = 0; i < fpb_path.size(); i++)
    {
        ref_points.push_back(Vecf<2>(fpb_path[i].s, fpb_path[i].l));
        ref_stamps.push_back(static_cast<double>(i * _sim_param.layer_time));
    }

    std::vector<double> weight_proximity = {1.0, 1.0};
    // RCLCPP_INFO(this->get_logger(), "GetBezierSplineUsingCorridor !!!");
    TicToc bezier_spline_timer;
    bool getbezierspline = false;
    std::vector<FrenetPoint> bezier_control_points;
    if (spline_generator.GetBezierSplineUsingCorridor(
            _cubes, start_constraints, end_constraints, ref_stamps,
            ref_points, weight_proximity,
            &bezier_spline) != kSuccess)
    {
        // 求解失败，转用备用的五次多项式连接
        LOG(INFO) << "---------OOQP Failed -------";
        // 获取路径规划结果的index2s表
        std::vector<double> path_index2s;
        path_index2s.emplace_back(0.0);
        for (size_t i = 1; i < fpb_path_trajectory.size(); i++)
        {
            path_index2s.emplace_back(path_index2s.back() +
                                      std::hypot(fpb_path_trajectory[i].x - fpb_path_trajectory[i - 1].x, fpb_path_trajectory[i].y - fpb_path_trajectory[i - 1].y));
        }
        // 对决策的ST曲线进行加密
        std::vector<STPoint> fpb_speed_profile_increased;
        if (!IncreaseFpbSpeedProfile(fpb_path_increased, _sim_param.increased_st_distance, _sim_param.layer_time, fpb_speed_profile_increased))
        {
            LOG(ERROR) << "[UMBP]****** IncreaseFpbSpeedProfile FAILED  ******";
            return false;
        }

        // 将路径规划结果和速度规划结果拼成轨迹
        std::vector<TrajectoryPoint> init_trajectory;
        // RCLCPP_INFO(this->get_logger(), "Start GenerateTrajectory !!!");
        if (!GenerateTrajectory(fpb_speed_profile_increased, fpb_path_trajectory, path_index2s, planning_start_point.time_stamped, init_trajectory))
        {
            LOG(ERROR) << "[UMBP]****** GenerateTrajectory FAILED  ******";
            return false;
        }

        // 轨迹拼接
        final_trajectory.clear();
        if (!_switch_trajectory.empty())
        {
            for (auto &&trajectory_point : _switch_trajectory)
            {
                final_trajectory.emplace_back(trajectory_point);
            }
        }

        for (auto &&trajectory_point : init_trajectory)
        {
            final_trajectory.emplace_back(trajectory_point);
        }

        // 上一周期轨迹赋值
        _previous_trajectory.clear();
        for (auto &&trajectory_point : final_trajectory)
        {
            _previous_trajectory.emplace_back(trajectory_point);
        }
    }
    else
    {
        // 求解成功 转换轨迹
        LOG(INFO) << "[UMBP]****** GetBezierSplineUsingCorridor  Success ******";
        getbezierspline = true;
        for (size_t i = 0; i < bezier_spline.ctrl_pts().size(); ++i)
        {
            for (int row = 0; row < (bezier_spline.ctrl_pts()[i].rows() - 1) * 10; ++row)
            {
                FrenetPoint cur_point;
                cur_point.t = i * 1.0 + row * _sim_param.layer_time / ((bezier_spline.ctrl_pts()[i].rows() - 1) * 10);
                Vecf<2> result_derivate_0, result_derivate_1, result_derivate_2;
                bezier_spline.evaluate(cur_point.t, 0, &result_derivate_0);
                bezier_spline.evaluate(cur_point.t, 1, &result_derivate_1);
                bezier_spline.evaluate(cur_point.t, 2, &result_derivate_2);

                cur_point.s = result_derivate_0[0];
                cur_point.l = result_derivate_0[1];
                cur_point.s_dot = result_derivate_1[0];
                cur_point.l_dot = result_derivate_1[1];
                cur_point.s_dot_dot = result_derivate_2[0];
                cur_point.l_dot_dot = result_derivate_2[1];
                bezier_control_points.emplace_back(cur_point);
            }
        }
        FrenetPoint cur_point;
        cur_point.t = 5.0;
        Vecf<2> result_derivate_0, result_derivate_1, result_derivate_2;
        bezier_spline.evaluate(cur_point.t, 0, &result_derivate_0);
        bezier_spline.evaluate(cur_point.t, 1, &result_derivate_1);
        bezier_spline.evaluate(cur_point.t, 2, &result_derivate_2);

        cur_point.s = result_derivate_0[0];
        cur_point.l = result_derivate_0[1];
        cur_point.s_dot = result_derivate_1[0];
        cur_point.l_dot = result_derivate_1[1];
        cur_point.s_dot_dot = result_derivate_2[0];
        cur_point.l_dot_dot = result_derivate_2[1];
        bezier_control_points.emplace_back(cur_point);

        int print_count = 0;
        for (auto point : bezier_control_points)
        {
            if (print_count % 50 == 0)
            {
                LOG(INFO) << std::fixed << std::setprecision(4)
                          << "  Point (s: " << point.s << " l:  " << point.l << " t:  " << point.t << " s_dot:  " << point.s_dot
                          << " l_dot:  " << point.l_dot << " s_dot_dot:  " << point.s_dot_dot << " l_dot_dot:  " << point.l_dot_dot;
            }
            print_count++;
        }

        auto bezier_trajectory_init = BezierPointToTrajectory(bezier_control_points, reference_line, reference_index2s, planning_start_point.time_stamped);

        // 轨迹拼接
        final_trajectory.clear();
        if (!_switch_trajectory.empty())
        {
            for (auto &&trajectory_point : _switch_trajectory)
            {
                final_trajectory.emplace_back(trajectory_point);
            }
        }

        for (auto &&trajectory_point : bezier_trajectory_init)
        {
            final_trajectory.emplace_back(trajectory_point);
        }

        // 上一周期轨迹赋值
        _previous_trajectory.clear();
        for (auto &&trajectory_point : final_trajectory)
        {
            _previous_trajectory.emplace_back(trajectory_point);
        }
    }

    // 绘制SL图
    // RCLCPP_INFO(this->get_logger(), "Start Plot SL !!!");
    if (_plot_count % 20 == 0)
    {
        if (_plot_count == 0) // 初始化窗口，仅执行一次
        {
            matplot::figure(_path_plot_handle);
            matplot::hold(true); // 保持绘图窗口，叠加新内容
        }

        matplot::cla();
        std::vector<double> s, l;
        if (getbezierspline == false)
        {
            for (size_t i = 0; i < fpb_path_increased.size(); i++)
            {
                s.emplace_back(fpb_path_increased[i].s);
                l.emplace_back(fpb_path_increased[i].l);
            }
        }
        else
        {
            for (size_t i = 0; i < bezier_control_points.size(); i++)
            {
                s.emplace_back(bezier_control_points[i].s);
                l.emplace_back(bezier_control_points[i].l);
            }
        }

        matplot::plot(s, l, "bo-")->line_width(4);

        // 只需调用一次的原因是，它的作用是保持当前图形，使后续的绘图命令不会清除之前的内容。直到你需要开始一个新的图形时，可以调用 matplot::hold(false); 来重置。
        matplot::hold(true);
        // // 凸空间
        // matplot::plot(final_dp_path_s, final_dp_path_l_min, "r*-")->line_width(2);
        // matplot::plot(final_dp_path_s, final_dp_path_l_max, "ro-")->line_width(2);

        // 静态障碍物
        for (const auto &pair : forward_prop_agent_set.forward_prop_agents)
        {
            const auto &agent = pair.second;
            double obs_s = agent.obs_frenet_point.s;
            double obs_l = agent.obs_frenet_point.l;

            matplot::line(obs_s - 2.5, obs_l + 1, obs_s + 2.5, obs_l + 1)->line_width(2);
            matplot::line(obs_s + 2.5, obs_l + 1, obs_s + 2.5, obs_l - 1)->line_width(2);
            matplot::line(obs_s + 2.5, obs_l - 1, obs_s - 2.5, obs_l - 1)->line_width(2);
            matplot::line(obs_s - 2.5, obs_l - 1, obs_s - 2.5, obs_l + 1)->line_width(2);
        }
        matplot::xlim({-5, 60});  // 使用std::vector<double> 初始化
        matplot::ylim({-10, 10}); // 使用std::vector<double> 初始化
        matplot::title("SL Path and Obstacles");
    }
    _plot_count++;
    return true;
}

void UMBPlanner::GetFrenetSamplePoints(std::vector<std::vector<FrenetPoint>> &frenet_sample_point, const FrenetPoint &planning_start_sl,
                                       const double &s_sample_distance, const int &s_sample_num,
                                       const double &l_sample_distance, const int &l_sample_num)
{
    for (int level = 0; level <= s_sample_num; level++)
    {
        frenet_sample_point.emplace_back();
        for (int i = 0; i <= l_sample_num; i++)
        {
            FrenetPoint current_point;
            current_point.s = planning_start_sl.s + level * s_sample_distance;
            current_point.l = planning_start_sl.l + (int(l_sample_num / 2) - i) * l_sample_distance;
            frenet_sample_point.back().emplace_back(current_point);
        }
    }
}

bool UMBPlanner::RunUmpb(const FrenetPoint ego_state, const ForwardPropAgentSet &forward_prop_agent_set)
{
    _sim_res = std::vector<int>(500);
    _risky_res = std::vector<int>(500);
    _sim_info = std::vector<std::string>(500);
    _final_cost = std::vector<double>(500);
    _progress_cost = std::vector<std::vector<CostStructure>>(500);
    _tail_cost = std::vector<CostStructure>(500);
    _forward_trajs = std::vector<std::vector<FrenetPoint>>(500);
    _forward_lat_behaviors = std::vector<std::vector<FpbLatAction>>(500);
    _forward_lon_behaviors = std::vector<std::vector<FpbLonAction>>(500);
    _surround_trajs = std::vector<std::unordered_map<int, std::vector<FrenetPoint>>>(500);
    auto behaviour_tree = _fpb_tree_ptr->get_behaviour_tree();
    int num_sequence = behaviour_tree.size();
    // * prepare for multi-threading
    std::vector<std::thread> thread_set(num_sequence);
    PrepareMultiThreadContainers(num_sequence);

    // * threading
    TicToc multi_thread_timer;
    multi_thread_timer.tic();
    // RCLCPP_INFO(this->get_logger(), "Start PropagateActionSequence !!!");
    for (size_t i = 0; i < static_cast<size_t>(num_sequence); i++)
    {
        thread_set[i] = std::thread(&UMBPlanner::PropagateActionSequence, this, ego_state,
                                    forward_prop_agent_set, behaviour_tree[i], i);
    }
    // RCLCPP_INFO(this->get_logger(), "PropagateActionSequence  Done!!!");
    for (size_t i = 0; i < static_cast<size_t>(num_sequence); i++)
    {
        thread_set[i].join();
    }
    // RCLCPP_INFO(this->get_logger(), "Start PropagateActionSequence !!!");
    // for (size_t i = 0; i < static_cast<size_t>(num_sequence); i++)
    // {
    //     PropagateActionSequence(ego_state, forward_prop_agent_set, behaviour_tree[i], i);
    // }
    LOG(INFO) << std::fixed << std::setprecision(4)
              << "[UMBP][Process] PropagateActionSequence Time :"
              << multi_thread_timer.toc() << "ms";

    // * Summary results
    for (int i = 0; i < num_sequence; ++i)
    {
        if (_sim_res[i])
        {
            std::ostringstream line_info;
            line_info << "[UMBP][Result] " << i << " [";
            for (const auto &a : _forward_lon_behaviors[i])
            {
                line_info << FpbTree::GetLonActionName(a);
            }
            line_info << "|";
            for (const auto &a : _forward_lat_behaviors[i])
            {
                line_info << FpbTree::GetLatActionName(a);
            }
            line_info << "]";
            line_info << "[s:" << _sim_res[i] << "|r:" << _risky_res[i]
                      << "|c:" << std::fixed << std::setprecision(3) << _tail_cost[i].ave()
                      << "]";
            line_info << " " << _sim_info[i] << "\n";

            line_info << "[UMBP][Result][e;s;n;hdi:";

            line_info << std::fixed << std::setprecision(2)
                      << _tail_cost[i].efficiency.ego_to_desired_vel << "; "
                      << _tail_cost[i].safety.ego_to_obs << "; "
                      << _tail_cost[i].navigation.ref_line_change << "; "
                      << _tail_cost[i].hdi.path_diff;
            line_info << "|";

            line_info << "]";

            LOG(INFO) << line_info.str();
        }
    }
    int behavior_sucess_num = 0;
    for (auto res : _sim_res)
    {
        behavior_sucess_num += res;
    }
    LOG(INFO) << "[UMBP][Result]Propagate sucess with "
              << behavior_sucess_num << " behaviors.";

    // * Evaluate
    // RCLCPP_INFO(this->get_logger(), "Start EvaluateMultiThreadSimResults !!!");
    if (!EvaluateMultiThreadSimResults(&_winner_id, &_winner_score))
    {
        LOG(ERROR)
            << "[Eudm][Fatal]fail to evaluate multi-thread sim results, no feasible result!!. Exit";
        return false;
    }

    LOG(INFO) << std::fixed << std::setprecision(2)
              << "[UMBP][Result] Best id:"
              << _winner_id << " ;"
              << "Best socre " << _winner_score;

    std::vector<FrenetPoint> Fpb_win_points = _forward_trajs[_winner_id];
    for (const auto &points : Fpb_win_points)
    {
        LOG(INFO) << std::fixed << std::setprecision(2) << "Fpb_win_points: (s: " << points.s << " l: " << points.l << ")";
    }
    return true;
}

TrajectoryPoint UMBPlanner::CalculatePlanningStartPoint(std::shared_ptr<VehicleState> ego_state)
{
    _switch_trajectory.clear();
    TrajectoryPoint planning_start_point;
    // 如果第一次运行
    double delta_T = 0.1;
    if (_previous_trajectory.empty())
    {
        planning_start_point.x = ego_state->x;
        planning_start_point.y = ego_state->y;
        planning_start_point.v = 0.0;
        planning_start_point.heading = ego_state->heading;
        planning_start_point.ax = 0.0;
        planning_start_point.ay = 0.0;
        planning_start_point.kappa = 0.0;
        planning_start_point.time_stamped = _current_time + delta_T;
    }
    // 不是第一次运行，已经有了上一周期规划的轨迹
    else
    {
        // 计算主车位置与目标点之间的误差
        size_t current_time_index = 0;
        if (_current_time <= _previous_trajectory[0].time_stamped)
        {
            current_time_index = 0;
        }
        // 获取当前时刻在上一周期轨迹对应的时间索引
        for (size_t i = 0; i < _previous_trajectory.size() - 2; i++)
        {
            if (_current_time > _previous_trajectory[i].time_stamped && _current_time <= _previous_trajectory[i + 1].time_stamped)
            {
                if ((_current_time - _previous_trajectory[i].time_stamped) <= (_previous_trajectory[i + 1].time_stamped - _current_time))
                {
                    current_time_index = i;
                }
                else
                {
                    current_time_index = i + 1;
                }
            }
        }
        auto target_point = _previous_trajectory[current_time_index];
        Eigen::Vector2d tau_target_point(std::cos(target_point.heading), std::sin(target_point.heading));
        Eigen::Vector2d nor_target_point(-std::sin(target_point.heading), std::cos(target_point.heading));
        Eigen::Vector2d host_to_target_point(target_point.x - ego_state->x, target_point.y - ego_state->y);
        double error_longitudional = std::abs(host_to_target_point.dot(tau_target_point)); // 纵向误差
        double error_lateral = std::abs(host_to_target_point.dot(nor_target_point));       // 横向误差

        // 主车实际位置与目标点差距不大
        if (error_lateral < 0.5 && error_longitudional < 1.5)
        {
            // 在上一周期轨迹上搜索规划起点
            size_t start_time_index = 0;
            double start_time = _current_time + delta_T;
            if (start_time <= _previous_trajectory[0].time_stamped)
            {
                start_time_index = 0;
            }
            for (size_t i = 0; i < _previous_trajectory.size() - 2; i++)
            {
                if (start_time > _previous_trajectory[i].time_stamped && start_time <= _previous_trajectory[i + 1].time_stamped)
                {
                    if ((start_time - _previous_trajectory[i].time_stamped) <= (_previous_trajectory[i + 1].time_stamped - start_time))
                    {
                        start_time_index = i;
                    }
                    else
                    {
                        start_time_index = i + 1;
                    }
                }
            }

            planning_start_point.x = _previous_trajectory[start_time_index].x;
            planning_start_point.y = _previous_trajectory[start_time_index].y;
            planning_start_point.v = _previous_trajectory[start_time_index].v;
            planning_start_point.heading = _previous_trajectory[start_time_index].heading;
            planning_start_point.ax = _previous_trajectory[start_time_index].ax;
            planning_start_point.ay = _previous_trajectory[start_time_index].ay;
            // 这里还要一个值得商讨的问题，实际使用current_time+delta_T还是上一周期上start_time_index所对应轨迹点的时间
            planning_start_point.time_stamped = start_time;

            // 这个时候还要拼接上一段轨迹,向前拼20个点
            if (start_time_index >= 20)
            {
                for (int i = start_time_index - 1; i >= 0 && (start_time_index - i) <= 20; i--)
                {
                    _switch_trajectory.emplace_front(_previous_trajectory[i]); // 这个排列顺序是序号越大，时间越靠后的
                }
            }
            else if (start_time_index > 0)
            {
                for (int i = start_time_index - 1; i >= 0; i--)
                {
                    _switch_trajectory.emplace_front(_previous_trajectory[i]); // 这个排列顺序是序号越大，时间越靠后的
                }
            }
        }
        // 主车实际位置与目标点差距很大
        else
        {
            // 用动力学方程外推,认为在着100ms中加速度变化很小
            planning_start_point.ax = ego_state->ax;
            planning_start_point.ay = ego_state->ay;
            double ego_vx = ego_state->v * std::cos(ego_state->heading);
            double ego_vy = ego_state->v * std::sin(ego_state->heading);
            double planning_start_point_vx = ego_vx + ego_state->ax * delta_T;
            double planning_start_point_vy = ego_vy + ego_state->ay * delta_T;
            planning_start_point.v = std::hypot(planning_start_point_vx, planning_start_point_vy);
            planning_start_point.heading = std::atan2(planning_start_point_vy, planning_start_point_vx);
            planning_start_point.x = ego_state->x + ego_vx * delta_T + 0.5 * ego_state->ax * delta_T * delta_T;
            planning_start_point.y = ego_state->y + ego_vy * delta_T + 0.5 * ego_state->ay * delta_T * delta_T;
            planning_start_point.time_stamped = _current_time + delta_T;
        }
    }
    return planning_start_point;
}

void UMBPlanner::ObstacleFileter(const std::shared_ptr<VehicleState> ego_state, const std::vector<derived_object_msgs::msg::Object> &obstacles)
{
    // 将障碍物分成动态与静态障碍物
    _static_obstacles.clear();
    _dynamic_obstacles.clear();
    _cur_detected_obs_id.clear();
    if (obstacles.empty())
    {
        return;
    }
    for (auto &&obs : obstacles)
    {
        if (obs.id == ego_state->id)
        {
            continue;
        } // 障碍物不包含主车
        double v_obs = std::sqrt(std::pow(obs.twist.linear.x, 2.0) + std::pow(obs.twist.linear.y, 2.0));     // 障碍物速度
        Eigen::Vector2d host_to_obs(obs.pose.position.x - ego_state->x, obs.pose.position.y - ego_state->y); // 主车到障碍物的向量
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        double longitudinal_d = host_to_obs.dot(tau_host); // 纵向距离
        double lateral_d = host_to_obs.dot(nor_host);      // 横向距离
        // 静态障碍物，即使有加速度，一个规划周期是0.1s，障碍物以最大加速度加速也达不到很大的速度
        if (v_obs <= 0.1)
        {
            // 超出1.5个车长即变道
            if (longitudinal_d <= 60 && longitudinal_d >= -7.5 && lateral_d <= 20 && lateral_d >= -20)
            {
                _static_obstacles.push_back(obs);
            }
            else
            {
                continue;
            }
        }
        else // 动态障碍物
        {
            if (longitudinal_d <= 80 && longitudinal_d > -10 && lateral_d <= 25 && lateral_d >= -25)
            {
                _dynamic_obstacles.push_back(obs);
                // 坐标存储，为了甘文乾实现预测算法
                // std::ifstream infile(_filename);
                // bool file_exists = infile.good();
                // infile.close();
                // if (!file_exists)
                // {
                //     std::ofstream file(_filename);
                //     file << "Vehicle_ID,Frame_ID,Local_X,Local_Y,v_Vel\n";
                //     file.close();
                // }
                // std::ofstream file;
                // file.open(_filename, std::ios::app); // 以追加模式打开文件
                // if (!file.is_open())
                // {
                //     LOG(ERROR) << "无法打开文件";
                // }
                // for (const auto obs : _dynamic_obstacles)
                // {
                //     int id = obs.id;
                //     double local_x = lateral_d;
                //     double local_y = longitudinal_d;
                //     double v_obs = std::sqrt(std::pow(obs.twist.linear.x, 2.0) + std::pow(obs.twist.linear.y, 2.0));
                //     double frame_number = _frame_number;
                //     file << id << "," << frame_number << "," << local_x << "," << local_y << "," << v_obs << "\n";
                // }
                // file.close();

                /**
                 * record history trajectory from dynamic_obstacles
                 */
                TrajectoryPoint cur_trajectory;
                cur_trajectory.x = obs.pose.position.x;
                cur_trajectory.y = obs.pose.position.y;
                cur_trajectory.v = std::sqrt(std::pow(obs.twist.linear.x, 2.0) + std::pow(obs.twist.linear.y, 2.0));
                cur_trajectory.vx = obs.twist.linear.x;
                cur_trajectory.vy = obs.twist.linear.y;
                cur_trajectory.ax = obs.accel.linear.x;
                cur_trajectory.ay = obs.accel.linear.y;
                // 利用tf2读取四元数，提取yaw角
                tf2::Quaternion tf2_q;
                tf2::fromMsg(obs.pose.orientation, tf2_q);
                double roll, pitch, yaw;
                tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
                cur_trajectory.heading = yaw;

                _history_trajectory_dynamic_obstacles[obs.id].emplace_back(cur_trajectory);

                if (_history_trajectory_dynamic_obstacles[obs.id].size() > 30)
                {
                    _history_trajectory_dynamic_obstacles[obs.id].erase(_history_trajectory_dynamic_obstacles[obs.id].begin());
                }

                _cur_detected_obs_id.emplace_back(obs.id);
            }
            else
            {
                continue;
            }
        }
    }
    _frame_number++;
}

bool UMBPlanner::GetSurroundingForwardSimAgents(ForwardPropAgentSet &forward_prop_agent_set,
                                                const std::shared_ptr<std::vector<PathPoint>> reference_line,
                                                const std::shared_ptr<VehicleState> ego_state)
{
    std::vector<FrenetPoint> static_obs_frent_coords;
    std::vector<FrenetPoint> dynamic_obs_frent_coords;
    if (!_static_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_static_obstacles, *reference_line, ego_state, static_obs_frent_coords);
    }
    if (!_dynamic_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_dynamic_obstacles, *reference_line, ego_state, dynamic_obs_frent_coords);
    }
    int static_num = 0;
    for (auto &&obs : static_obs_frent_coords)
    {
        ForwardPropAgent cur_agent;
        cur_agent.id = _static_obstacles[static_num].id;
        cur_agent.obs_frenet_point.s = obs.s;
        cur_agent.obs_frenet_point.l = obs.l;
        cur_agent.obs_frenet_point.s_dot = obs.s_dot;
        cur_agent.obs_frenet_point.l_dot = obs.l_dot;
        cur_agent.obs_frenet_point.s_dot_dot = obs.s_dot_dot;
        cur_agent.obs_frenet_point.l_dot_dot = obs.l_dot_dot;
        forward_prop_agent_set.forward_prop_agents.emplace(_static_obstacles[static_num].id, cur_agent);
        static_num++;
    }
    int dynamic_num = 0;
    for (auto &&obs : dynamic_obs_frent_coords)
    {
        int obs_id = _dynamic_obstacles[dynamic_num].id;
        ForwardPropAgent cur_agent;
        cur_agent.id = obs_id;
        cur_agent.obs_frenet_point.s = obs.s;
        cur_agent.obs_frenet_point.l = obs.l;
        cur_agent.obs_frenet_point.s_dot = obs.s_dot;
        cur_agent.obs_frenet_point.l_dot = obs.l_dot;
        cur_agent.obs_frenet_point.s_dot_dot = obs.s_dot_dot;
        cur_agent.obs_frenet_point.l_dot_dot = obs.l_dot_dot;
        forward_prop_agent_set.forward_prop_agents.emplace(obs_id, cur_agent);
        dynamic_num++;
    }
    return true;
}

bool UMBPlanner::PredictTrajectoryDynamicObs(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state)
{
    _predict_trajectory_dynamic_obstacles.clear();
    for (const auto &id : _cur_detected_obs_id)
    {
        auto cur_history_trajectory = _history_trajectory_dynamic_obstacles[id];
        std::vector<TrajectoryPoint> sample_trajectory;
        if (cur_history_trajectory.size() >= 30)
        {
            LOG(INFO) << "Active UnscentedKalmanFilter";
            // Predict by UKF
            // Step1: Init
            UnscentedKalmanFilter ukf;
            Eigen::VectorXd initial_state(STATE_DIM);
            auto tra_point = cur_history_trajectory[0];
            initial_state << tra_point.x, tra_point.y, tra_point.vx, tra_point.vy, tra_point.ax, tra_point.ay, tra_point.heading;
            Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;
            ukf.initialize(initial_state, initial_covariance);

            // Step2: Get measurement
            std::vector<Eigen::VectorXd> measurements;
            for (const auto &tra_point : cur_history_trajectory)
            {
                Eigen::VectorXd measurement(STATE_DIM);
                measurement(0) = tra_point.x;
                measurement(1) = tra_point.y;
                measurement(2) = tra_point.vx;
                measurement(3) = tra_point.vy;
                measurement(4) = tra_point.ax;
                measurement(5) = tra_point.ay;
                measurement(6) = tra_point.heading;
                measurements.emplace_back(measurement);
            }

            // Step3: Update filter by measurement
            double dt = 0.1;
            // 使用测量数据更新滤波器
            for (const auto &measurement : measurements)
            {
                ukf.predict(dt);
                ukf.update(measurement);
            }

            // Step4: Predict
            int future_steps = 50;
            auto predictions = ukf.predictFuturePositions(future_steps, dt);
            sample_trajectory.emplace_back(cur_history_trajectory.back());
            // Step5: Get Predict Trajectory
            for (int i = 1; i <= 5; i++)
            {
                auto prediction = predictions[10 * i - 1];
                TrajectoryPoint temp_point;
                temp_point.x = prediction(0);
                temp_point.y = prediction(1);
                temp_point.vx = prediction(2);
                temp_point.vy = prediction(3);
                temp_point.ax = prediction(4);
                temp_point.ay = prediction(5);
                temp_point.heading = prediction(6);
                sample_trajectory.emplace_back(temp_point);
            }
        }
        else
        {
            // Predict through motion equation
            auto latest_trajectory = cur_history_trajectory.back();
            sample_trajectory.emplace_back(latest_trajectory);
            double dt = 1.0;
            auto temp = latest_trajectory;
            for (int i = 1; i <= 5; i++)
            {
                temp.x = latest_trajectory.x + latest_trajectory.vx * dt + 0.5 * latest_trajectory.ax * dt * dt;
                temp.y = latest_trajectory.y + latest_trajectory.vy * dt + 0.5 * latest_trajectory.ay * dt * dt;
                temp.vx = latest_trajectory.vx;
                temp.vy = latest_trajectory.vy;
                temp.ax = latest_trajectory.ax;
                temp.ay = latest_trajectory.ay;
                temp.heading = latest_trajectory.heading;
                latest_trajectory = temp;
                sample_trajectory.emplace_back(temp);
            }
        }

        // Step6: Transfer to Frenet Point
        std::vector<FrenetPoint> temp_set;
        for (const auto &trajectory : sample_trajectory)
        {
            cartesion_set_to_frenet_set(trajectory, *reference_line, ego_state, temp_set);
            _predict_trajectory_dynamic_obstacles[id].emplace_back(temp_set.front());
            temp_set.clear();
        }
    }

    for (const auto &pair : _predict_trajectory_dynamic_obstacles)
    {
        const auto &agent = pair.second;
        for (int i = 0; i <= 4; i++)
        {
            LOG(INFO) << "Agent_ID " << pair.first << " at time " << i + 1 << " : (s: " << agent[i].s << ",l: "
                      << agent[i].l << ",s_dot: " << agent[i].s_dot << ",l_dot: " << agent[i].l_dot << ")";
        }
    }

    return true;
}

bool UMBPlanner::PrepareMultiThreadContainers(const int num_sequence)
{
    LOG(INFO) << " [UMBP][Process]Prepare multi-threading-" << num_sequence;
    // 然后调整它的大小为 n_sequence，并用 0 填充
    _sim_res.clear();
    _sim_res.resize(num_sequence, 0);

    _risky_res.clear();
    _risky_res.resize(num_sequence, 0);

    _sim_info.clear();
    _sim_info.resize(num_sequence, std::string(""));

    _final_cost.clear();
    _final_cost.resize(num_sequence, 0.0);

    // 可能用于存储每个线程的进度成本
    _progress_cost.clear();
    _progress_cost.resize(num_sequence);

    // 用于存储每个线程的尾部成本
    _tail_cost.clear();
    _tail_cost.resize(num_sequence);

    _forward_trajs.clear();
    _forward_trajs.resize(num_sequence);

    _forward_lat_behaviors.clear();
    _forward_lat_behaviors.resize(num_sequence);

    _forward_lon_behaviors.clear();
    _forward_lon_behaviors.resize(num_sequence);

    _surround_trajs.clear();
    _surround_trajs.resize(num_sequence);
    return true;
}

bool UMBPlanner::PropagateActionSequence(const FrenetPoint ego_state,
                                         const ForwardPropAgentSet &surrounding_fsagents,
                                         const std::vector<FpbAction> &action_seq, const int &seq_id)
{
    // ~ For each ego sequence, we may further branch here, which will create
    // ~ multiple sub threads. Currently, we use n_sub_threads = 1

    // LOG(INFO) << "-------------------------------------------------";
    // LOG(INFO) << "Begin PropagateActionSequence threading " << seq_id;
    // for (auto &a : action_seq)
    // {
    //     LOG(INFO) << "action_seq: " << FpbTree::GetLonActionName(a.lon) << FpbTree::GetLatActionName(a.lat) << "  ";
    // }

    int n_sub_threads = 1;

    std::vector<int> sub_sim_res(n_sub_threads);
    std::vector<int> sub_risky_res(n_sub_threads);
    std::vector<std::string> sub_sim_info(n_sub_threads);
    std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
    std::vector<CostStructure> sub_tail_cost(n_sub_threads);
    std::vector<std::vector<FrenetPoint>> sub_forward_trajs(n_sub_threads);
    std::vector<std::vector<FpbLatAction>> sub_forward_lat_behaviors(n_sub_threads);
    std::vector<std::vector<FpbLonAction>> sub_forward_lon_behaviors(n_sub_threads);
    std::vector<std::unordered_map<int, std::vector<FrenetPoint>>> sub_surround_trajs(n_sub_threads);

    for (int i = 0; i < n_sub_threads; i++)
    {
        int sim_res = 0;
        int risky_res = 0;
        std::string sim_info;
        std::vector<CostStructure> progress_cost;
        CostStructure tail_cost;
        std::vector<FrenetPoint> forward_trajs;
        std::vector<FpbLatAction> forward_lat_behaviors;
        std::vector<FpbLonAction> forward_lon_behaviors;
        std::unordered_map<int, std::vector<FrenetPoint>> surround_trajs;
        // RCLCPP_INFO(this->get_logger(), "Start PropagateScenario %.i !!!", seq_id);
        if (!PropagateScenario(ego_state, surrounding_fsagents, action_seq, seq_id, i, &sim_res,
                               &risky_res, &sim_info, &progress_cost, &tail_cost, &forward_trajs,
                               &forward_lat_behaviors, &forward_lon_behaviors, &surround_trajs))
        {
            // LOG(INFO) << std::fixed << std::setprecision(4)
            //           << "[UMBP]****** ActionSequence " << seq_id << " PropogateScenario " << i << " FAILED  ******";
            sim_res = 0;
            sub_sim_res[i] = sim_res;
            continue;
        }
        sub_sim_res[i] = sim_res;
        sub_risky_res[i] = risky_res;
        sub_sim_info[i] = sim_info;
        sub_progress_cost[i] = progress_cost;
        sub_tail_cost[i] = tail_cost;
        sub_forward_trajs[i] = forward_trajs;
        sub_forward_lat_behaviors[i] = forward_lat_behaviors;
        sub_forward_lon_behaviors[i] = forward_lon_behaviors;
        sub_surround_trajs[i] = surround_trajs;
    }

    double sum = 0;
    for (auto sim_res : sub_sim_res)
    {
        sum += sim_res;
    }
    if (sum < 1)
    {
        // LOG(INFO) << std::fixed << std::setprecision(4)
        //           << "[UMBP]****** ActionSequence " << seq_id << " All Scenario  FAILED  ******";
        return false;
        _sim_res[seq_id] = static_cast<int>(0);
    }
    // RCLCPP_INFO(this->get_logger(), "Start Get PropagateActionSequence %.i Min Cost!!!", seq_id);
    double min_cost = std::numeric_limits<double>::max();
    int index = -1;
    for (int i = 0; i < n_sub_threads; i++)
    {
        if (sub_tail_cost[i].ave() < min_cost && sub_sim_res[i] == static_cast<int>(1))
        {
            min_cost = sub_tail_cost[i].ave();
            index = i;
        }
        else
        {
            continue;
        }
    }
    if (index == static_cast<int>(-1))
    {
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[UMBP]****** Find min cost FAILED  ******";
        _sim_res[seq_id] = static_cast<int>(0);
        return false;
    }
    _sim_res[seq_id] = sub_sim_res[index];
    _risky_res[seq_id] = sub_risky_res[index];
    _sim_info[seq_id] = sub_sim_info[index];
    _progress_cost[seq_id] = sub_progress_cost[index];
    _tail_cost[seq_id] = sub_tail_cost[index];
    _forward_trajs[seq_id] = sub_forward_trajs[index];
    _forward_trajs[seq_id] = sub_forward_trajs[index];
    _forward_lon_behaviors[seq_id] = sub_forward_lon_behaviors[index];
    _forward_lat_behaviors[seq_id] = sub_forward_lat_behaviors[index];
    _surround_trajs[seq_id] = sub_surround_trajs[index];
    // RCLCPP_INFO(this->get_logger(), "Start Get PropagateActionSequence %.i Min Cost Done!!!", seq_id);
    return true;
}

bool UMBPlanner::PropagateScenario(
    const FrenetPoint &ego_state, const ForwardPropAgentSet &surrounding_fsagents,
    const std::vector<FpbAction> &action_seq, const int &seq_id, const int &sub_seq_id,
    int *sub_sim_res, int *sub_risky_res, std::string *sub_sim_info,
    std::vector<CostStructure> *sub_progress_cost,
    CostStructure *sub_tail_cost,
    std::vector<FrenetPoint> *sub_forward_trajs,
    std::vector<FpbLatAction> *sub_forward_lat_behaviors,
    std::vector<FpbLonAction> *sub_forward_lon_behaviors,
    std::unordered_map<int, std::vector<FrenetPoint>> *sub_surround_trajs)
{
    // LOG(INFO) << "Threading " << seq_id << " begin PropagateScenario ";
    // declare variables which will be used to track traces from multiple layers
    sub_forward_trajs->emplace_back(ego_state);
    sub_forward_lat_behaviors->emplace_back(FpbLatAction::Keeping);
    sub_forward_lon_behaviors->emplace_back(FpbLonAction::Maintain);

    // get ego next sample point
    bool is_lat_action_done = false;
    int delta_l_num = 0, delta_s_num = 0, s_index = 0, l_index = (int)(_sim_param.l_sample_num / 2);
    FrenetPoint next_sample_point;
    for (size_t i = 0; i < action_seq.size(); i++)
    {
        delta_l_num = 0;
        delta_s_num = 0;
        auto action_this_layer = action_seq[i];
        // lon behavtiour
        switch (action_this_layer.lon)
        {
        case FpbLonAction::Maintain:
            delta_s_num = static_cast<int>(std::ceil(_current_ego_state->v / _sim_param.s_sample_distance));
            break;
        case FpbLonAction::Accelearte:
            delta_s_num = static_cast<int>(std::ceil((_current_ego_state->v + _sim_param.acc_ref) / _sim_param.s_sample_distance));
            break;
        case FpbLonAction::Decelerate:
            delta_s_num = static_cast<int>(std::floor(std::max(0.0, (_current_ego_state->v - _sim_param.dec_ref))) / _sim_param.s_sample_distance);
            break;
        default:
            LOG(ERROR) << std::fixed << std::setprecision(4)
                       << "[UMBP]****** switch (action_seq[0].lon) FAILED  ******";
            return false;
        }

        // lat behaviour
        if (action_this_layer.lat == FpbLatAction::SwitchLeft && !is_lat_action_done)
        {
            delta_l_num = -1;
            is_lat_action_done = false;
        }
        if (action_this_layer.lat == FpbLatAction::SwitchRight && !is_lat_action_done)
        {
            delta_l_num = 1;
            is_lat_action_done = false;
        }

        // get next_sample_point
        s_index += delta_s_num;
        l_index += delta_l_num;
        l_index = std::max(std::min(l_index, (int)(_sim_param.l_sample_num)), 0);
        s_index = std::min(s_index, (int)(_sim_param.s_sample_num - 1));
        next_sample_point = _local_sample_points[s_index][l_index];
        // check road bound
        if (next_sample_point.l <= -ego_state.l - _sim_param.l_ref_to_right_road_bound + 0.5 || next_sample_point.l >= -ego_state.l + _sim_param.l_ref_to_left_road_bound)
        {
            // LOG(INFO) << std::fixed << std::setprecision(4)
            //           << "******Action Sequence " << seq_id << " Sub-Scenario " << sub_seq_id << " Reach Road Bound Restrict  ******";
            return false;
        }
        if (next_sample_point.s < 1)
        {
            next_sample_point.s = _sim_param.s_sample_distance;
        }

        sub_forward_trajs->emplace_back(next_sample_point);
        sub_forward_lat_behaviors->emplace_back(action_this_layer.lat);
        sub_forward_lon_behaviors->emplace_back(action_this_layer.lon);
    }

    // LOG(INFO) << std::fixed << std::setprecision(3) << "sub_forward_trajs "
    //           << sub_forward_trajs->at(0).s << ", " << sub_forward_trajs->at(0).l << " from 0; "
    //           << sub_forward_trajs->at(1).s << ", " << sub_forward_trajs->at(1).l << " from 1; "
    //           << sub_forward_trajs->at(2).s << ", " << sub_forward_trajs->at(2).l << " from 2; "
    //           << sub_forward_trajs->at(3).s << ", " << sub_forward_trajs->at(3).l << " from 3; "
    //           << sub_forward_trajs->at(4).s << ", " << sub_forward_trajs->at(4).l << " from 4; "
    //           << sub_forward_trajs->at(5).s << ", " << sub_forward_trajs->at(5).l << " from 5; ";

    // 五次多项式插值
    std::vector<FrenetPoint> sub_forward_trajs_inter;
    int sample_num = 10;
    for (size_t i = 0; i < action_seq.size(); i++)
    {
        auto start_point = sub_forward_trajs->at(i);
        auto end_point = sub_forward_trajs->at(i + 1);
        double sample_distance = static_cast<double>((end_point.s - start_point.s) / sample_num);
        PolynomialCurve curve;
        curve.curve_fitting(start_point.s, start_point.l, start_point.l_prime, start_point.l_prime_prime,
                            end_point.s, end_point.l, end_point.l_prime, end_point.l_prime_prime);
        for (int j = 0; j < sample_num; j++)
        {
            FrenetPoint cur_point;
            double cur_s = start_point.s + j * sample_distance;
            cur_point.s = cur_s;
            cur_point.l = curve.value_evaluation(cur_s, 0);
            sub_forward_trajs_inter.emplace_back(cur_point);
        }
    }
    sub_forward_trajs_inter.emplace_back(sub_forward_trajs->back());

    // predict surrounding agent next sample point
    std::unordered_map<int, std::vector<FrenetPoint>> sub_surround_trajs_iter;
    for (const auto &fpa : surrounding_fsagents.forward_prop_agents)
    {
        if (std::find(_cur_detected_obs_id.begin(), _cur_detected_obs_id.end(), fpa.first) != _cur_detected_obs_id.end())
        {
            auto trajectory = _predict_trajectory_dynamic_obstacles.at(fpa.first);
            sub_surround_trajs->emplace(fpa.first, std::vector<FrenetPoint>{});
            for (size_t i = 0; i < action_seq.size() + 1; i++)
            {
                sub_surround_trajs->at(fpa.first).emplace_back(trajectory[i]);
            }
            sub_surround_trajs_iter.emplace(fpa.first, std::vector<FrenetPoint>({fpa.second.obs_frenet_point}));
            // 五次多项式插值,两点之间有9个插值点（10段线段）
            int sample_num = 10;
            for (size_t i = 0; i < action_seq.size(); i++)
            {
                auto start_point = sub_surround_trajs->at(fpa.first)[i];
                auto end_point = sub_surround_trajs->at(fpa.first)[i + 1];
                double sample_distance = static_cast<double>((end_point.s - start_point.s) / sample_num);
                PolynomialCurve curve;
                curve.curve_fitting(start_point.s, start_point.l, start_point.l_prime, start_point.l_prime_prime,
                                    end_point.s, end_point.l, end_point.l_prime, end_point.l_prime_prime);
                for (int j = 0; j < sample_num; j++)
                {
                    if ((i == 0) & (j == 0))
                    {
                        continue;
                    }
                    FrenetPoint cur_point;
                    double cur_s = start_point.s + j * sample_distance;
                    cur_point.s = cur_s;
                    cur_point.l = curve.value_evaluation(cur_s, 0);
                    sub_surround_trajs_iter.at(fpa.first).emplace_back(cur_point);
                }
            }
            sub_surround_trajs_iter.at(fpa.first).emplace_back(sub_surround_trajs->at(fpa.first).back());
        }
        else
        {
            sub_surround_trajs->emplace(fpa.first, std::vector<FrenetPoint>({fpa.second.obs_frenet_point}));
            for (size_t i = 0; i < action_seq.size(); i++)
            {
                sub_surround_trajs->at(fpa.first).emplace_back(fpa.second.obs_frenet_point);
            }
            sub_surround_trajs_iter.emplace(fpa.first, std::vector<FrenetPoint>({fpa.second.obs_frenet_point}));
            for (int i = 0; i < 10 * 5; i++)
            {
                sub_surround_trajs_iter.at(fpa.first).emplace_back(fpa.second.obs_frenet_point);
            }
        }
    }

    // calculate cost
    bool is_risky = false;
    std::set<size_t> risky_ids;
    double path_diff_cost = 0.0;
    // if (!CalculateCost(*sub_forward_trajs, *sub_surround_trajs, sub_progress_cost, &is_risky, &risky_ids))
    if (!CalculateCost(sub_forward_trajs_inter, sub_surround_trajs_iter, sub_progress_cost, &is_risky, &risky_ids))
    {
        // LOG(INFO) << std::fixed << std::setprecision(4)
        //           << "******Action Sequence " << seq_id << " Sub-Scenario " << sub_seq_id << " May Crash with Obs, Dangerous !! ******";
        return false;
    }

    if (_history_decision.size() >= 5)
    {
        if (!CalculatePathDiffCost(sub_forward_trajs, &path_diff_cost))
        {
            LOG(INFO) << std::fixed << std::setprecision(4)
                      << "******Action Sequence " << seq_id << " Sub-Scenario " << sub_seq_id << " CalculatePathDiffCost Failed !! ******";
            return false;
        }
    }

    for (size_t i = 0; i < sub_progress_cost->size(); i++)
    {
        sub_tail_cost->efficiency.ego_to_desired_vel += sub_progress_cost->at(i).efficiency.ego_to_desired_vel;
        sub_tail_cost->navigation.ref_line_change += sub_progress_cost->at(i).navigation.ref_line_change;
        sub_tail_cost->safety.ego_to_obs += sub_progress_cost->at(i).safety.ego_to_obs;
    }
    sub_tail_cost->hdi.path_diff += path_diff_cost * _cost_param.hdi_weight;

    if (is_risky)
    {
        *sub_risky_res = 1;
    }

    // LOG(INFO) << std::fixed << std::setprecision(4)
    //           << "sub_tail_cost->efficiency: " << sub_tail_cost->efficiency.ego_to_desired_vel << " // "
    //           << " sub_tail_cost->navigation: " << sub_tail_cost->navigation.ref_line_change << " // "
    //           << " sub_tail_cost->safety: " << sub_tail_cost->safety.ego_to_obs << " // "
    //           << " sub_tail_cost.ave: " << sub_tail_cost->ave()
    //           << " sub_risky_res: " << *sub_risky_res;

    *sub_sim_res = static_cast<int>(1);
    return true;
}

bool UMBPlanner::CalculateCost(const std::vector<FrenetPoint> &forward_trajs,
                               const std::unordered_map<int, std::vector<FrenetPoint>> &surround_trajs,
                               std::vector<CostStructure> *progress_cost,
                               bool *is_risky, std::set<size_t> *risky_ids)
{
    for (size_t i = 0; i < forward_trajs.size(); i++)
    {
        CostStructure cost_tmp;
        // navigation
        cost_tmp.navigation.ref_line_change = (_cost_param.ref_line_change * std::abs(forward_trajs[i].l)) * std::pow(_cost_param.discount_factor, i);
        // LOG(INFO) << std::fixed << std::setprecision(4) << " cost_tmp.navigation.ref_line_change " << cost_tmp.navigation.ref_line_change;

        // efficiency：包含自车与期望速度之间的差异代价
        double ego_velocity;
        if (i == static_cast<size_t>(0))
        {
            ego_velocity = std::abs(forward_trajs[i].s) / (_sim_param.layer_time);
        }
        else
        {
            ego_velocity = std::abs(forward_trajs[i].s - forward_trajs[i - 1].s) / (_sim_param.layer_time);
        }
        cost_tmp.efficiency.ego_to_desired_vel = _cost_param.ego_lack_speed_to_desired_unit_cost * std::abs(ego_velocity - _reference_speed);
        // LOG(INFO) << std::fixed << std::setprecision(4) << " cost_tmp.efficiency.ego_to_desired_vel " << cost_tmp.efficiency.ego_to_desired_vel;
        // safety:
        for (const auto &surr_traj : surround_trajs)
        {

            if (forward_trajs.size() != surr_traj.second.size())
            {

                LOG(ERROR) << std::fixed << std::setprecision(4)
                           << "[UMBP]****** forward_trajs.size() != surr_traj.second.size()  ******";
                return false;
            }
            double distance = GetMinDistanceFromEgoToObs(forward_trajs[i], surr_traj.second[i], 5.0, 4.0);
            if (distance >= 8.0)
            {
                cost_tmp.safety.ego_to_obs += 0;
            }
            else if (distance >= 1.0)
            {

                cost_tmp.safety.ego_to_obs += (_cost_param.ego_to_obs / distance) * std::pow(_cost_param.discount_factor, i);
            }
            else
            {
                *is_risky = true;
                risky_ids->emplace(i);
                cost_tmp.safety.ego_to_obs += (_cost_param.ego_to_obs / (distance * distance + 1e-6)) * std::pow(_cost_param.discount_factor, i);
            }
            // else
            // {
            //     return false;
            // }
            // LOG(INFO) << std::fixed << std::setprecision(4) << " cost_tmp.safety.ego_to_obs " << cost_tmp.safety.ego_to_obs;
        }
        progress_cost->emplace_back(cost_tmp);
    }
    return true;
}

// bool UMBPlanner::CalculatePathDiffCost(const std::vector<FrenetPoint> *forward_trajs, double *path_diff_cost)
// {
//     std::vector<double> frechet_distance;
//     for (size_t i = 0; i < _history_decision.size(); i++)
//     {
//         double cur_frechet_distance;
//         cur_frechet_distance = GetFrechetDistance(forward_trajs, _history_decision[i]);
//         frechet_distance.push_back(cur_frechet_distance);
//         *path_diff_cost += cur_frechet_distance * std::pow(0.9, static_cast<int>(_history_decision.size() - 1 - i));
//     }
//     return true;
// }

bool UMBPlanner::CalculatePathDiffCost(const std::vector<FrenetPoint> *forward_trajs, double *path_diff_cost)
{
    // RCLCPP_INFO(this->get_logger(), "Start CalculatePathDiffCost !!!");
    // 归一化_history_decision并合并
    std::vector<FrenetPoint> history_decision_norm;
    for (size_t i = 0; i < _history_decision.size(); i++)
    {
        for (size_t i = 0; i < _history_decision.size(); i++)
            for (auto &point : _history_decision[i])
            {
                {
                    FrenetPoint cur_point;
                    cur_point.s = point.s - _history_decision[i][0].s;
                    cur_point.l = point.l - _history_decision[i][0].l;
                    history_decision_norm.emplace_back(cur_point);
                }
            }
    }

    // 归一化forward_trajs
    std::vector<FrenetPoint> forward_trajs_norm;
    double s0 = (*forward_trajs)[0].s;
    double l0 = (*forward_trajs)[0].l;
    for (auto &point : *forward_trajs)
    {
        FrenetPoint cur_point;
        cur_point.s = point.s - s0;
        cur_point.l = point.l - l0;
        forward_trajs_norm.emplace_back(cur_point);
    }

    // 计算 history_decision_norm 的 KDE
    std::vector<double> kde_history = calculate_kde(history_decision_norm);

    // 计算当前 forward_trajs_norm 的 KDE
    std::vector<double> kde_forward = calculate_kde(forward_trajs_norm);

    // 计算两者的 Jensen - Shannon 散度
    *path_diff_cost = calculate_jensen_shannon_divergence(kde_history, kde_forward);

    return true;
}

double UMBPlanner::kde_estimate(const std::vector<FrenetPoint> &curve, double x, double bandwidth)
{
    // RCLCPP_INFO(this->get_logger(), "Start kde_estimate !!!");
    double density = 0.0;
    int n = curve.size();
    for (const auto &point : curve)
    {
        double diff = x - point.s;
        density += std::exp(-(diff * diff) / (2 * bandwidth * bandwidth));
    }
    density /= (n * bandwidth * std::sqrt(2 * M_PI));
    return density;
}

// 计算 Jensen - Shannon 散度
double UMBPlanner::calculate_jensen_shannon_divergence(const std::vector<double> &p, const std::vector<double> &q)
{
    // RCLCPP_INFO(this->get_logger(), "Start calculate_js !!!");
    int n = p.size();
    std::vector<double> m(n);
    double jsd = 0.0;

    for (int i = 0; i < n; ++i)
    {
        double p_value = p[i] > 1e-8 ? p[i] : 1e-8;
        double q_value = q[i] > 1e-8 ? q[i] : 1e-8;
        double m_value = 0.5 * (p_value + q_value);

        if (p_value > 0)
            jsd += 0.5 * p_value * std::log2(p_value / m_value);
        if (q_value > 0)
            jsd += 0.5 * q_value * std::log2(q_value / m_value);
    }
    return jsd;
}

// 计算核密度估计值
std::vector<double> UMBPlanner::calculate_kde(const std::vector<FrenetPoint> &curve, double bandwidth, double step)
{
    // RCLCPP_INFO(this->get_logger(), "Start calculate_kde !!!");
    // x轴上范围
    double min_x = curve[0].s;
    double max_x = curve.back().s;
    std::vector<double> kde_values;

    // 核密度估计
    for (double x = min_x; x <= max_x; x += step)
    { // 步长可以调整
        double density = kde_estimate(curve, x, bandwidth);
        kde_values.push_back(density);
    }
    return kde_values;
}

double UMBPlanner::GetFrechetDistance(const std::vector<FrenetPoint> *forward_trajs, const std::vector<FrenetPoint> &traj)
{
    // 归一化所有点
    std::vector<FrenetPoint> curve1;
    double s0 = (*forward_trajs)[0].s;
    double l0 = (*forward_trajs)[0].l;
    for (auto &point : *forward_trajs)
    {
        FrenetPoint cur_point;
        cur_point.s = point.s - s0;
        cur_point.l = point.l - l0;
        curve1.emplace_back(cur_point);
    }

    std::vector<FrenetPoint> curve2;
    double s1 = traj[0].s;
    double l1 = traj[0].l;
    for (auto &point : traj)
    {
        FrenetPoint cur_point;
        cur_point.s = point.s - s1;
        cur_point.l = point.l - l1;
        curve2.emplace_back(cur_point);
    }

    int n = forward_trajs->size();
    std::vector<std::vector<double>> dp(n, std::vector<double>(n, -1));
    return FrechetDfs(0, 0, curve1, curve2, dp);
}

double UMBPlanner::FrechetDfs(int i, int j, const std::vector<FrenetPoint> &curve1, const std::vector<FrenetPoint> &curve2, std::vector<std::vector<double>> &dp)
{
    int n = curve1.size();

    if (i == n - 1 && j == n - 1)
        return std::hypot(curve1[i].s - curve2[j].s, curve1[i].l - curve2[j].l); // 终点距离

    if (i >= n || j >= n)
        return 1e9; // 越界返回一个很大的值

    if (dp[i][j] != -1)
        return dp[i][j]; // 如果已经计算过，直接返回

    // 递归计算
    double dist = std::hypot(curve1[i].s - curve2[j].s, curve1[i].l - curve2[j].l);
    double res = dist + std::min({FrechetDfs(i + 1, j, curve1, curve2, dp), FrechetDfs(i, j + 1, curve1, curve2, dp), FrechetDfs(i + 1, j + 1, curve1, curve2, dp)});

    dp[i][j] = res;
    return res;
}

bool UMBPlanner::EvaluateMultiThreadSimResults(int *winner_id, double *winner_cost)
{
    double min_cost = std::numeric_limits<double>::max();
    size_t best_id = 0;
    bool get_feasible_result = false;
    for (size_t i = 0; i < _fpb_tree_ptr->get_behaviour_tree().size(); i++)
    {
        if (_sim_res[i] == 0)
        {
            continue;
        }
        double cost = _tail_cost[i].ave();
        if (cost < min_cost)
        {
            min_cost = cost;
            best_id = i;
            get_feasible_result = true;
        }
    }
    if (!get_feasible_result)
    {
        return false;
    }
    *winner_id = best_id;
    *winner_cost = min_cost;
    return true;
}

bool UMBPlanner::IncreaseFpbPath(const std::vector<FrenetPoint> &fpb_path_init, const double &increased_distance, std::vector<FrenetPoint> &fpb_path_increased)
{
    double sample_s = fpb_path_init[1].s - fpb_path_init[0].s;
    _increased_sl_sample_num = static_cast<int>(sample_s / increased_distance);
    for (size_t i = 0; i < fpb_path_init.size() - 1; i++)
    {
        PolynomialCurve poly_curve;
        poly_curve.curve_fitting(fpb_path_init[i].s, fpb_path_init[i].l, fpb_path_init[i].l_prime, fpb_path_init[i].l_prime_prime,
                                 fpb_path_init[i + 1].s, fpb_path_init[i + 1].l, fpb_path_init[i + 1].l_prime, fpb_path_init[i + 1].l_prime_prime);
        for (int j = 0; j < _increased_sl_sample_num; j++)
        {
            double cur_s = fpb_path_init[i].s + j * increased_distance;
            FrenetPoint cur_point;
            cur_point.s = cur_s;
            cur_point.l = poly_curve.value_evaluation(cur_point.s, 0);
            cur_point.l_prime = poly_curve.value_evaluation(cur_point.s, 1);
            cur_point.l_prime_prime = poly_curve.value_evaluation(cur_point.s, 2);
            fpb_path_increased.emplace_back(cur_point);
        }
    }
    fpb_path_increased.emplace_back(fpb_path_init.back());
    return true;
}

bool UMBPlanner::IncreaseFpbSpeedProfile(const std::vector<FrenetPoint> &fpb_path, const double &increased_time, const double &layer_time,
                                         std::vector<STPoint> &speed_profile_increased)
{
    // Get init speed_profile
    std::vector<STPoint> speed_profile_init;
    double dt = layer_time / _increased_sl_sample_num; // 1/4  = 0.25
    double previous_s_dot;
    for (size_t i = 0; i < fpb_path.size() - 1; i++)
    {
        STPoint cur_st_point;
        cur_st_point.s = fpb_path[i].s;
        cur_st_point.s_dot = (fpb_path[i + 1].s - fpb_path[i].s) / dt;
        cur_st_point.t = static_cast<double>(i * dt);
        if (i == 0)
        {
            cur_st_point.s_dot_dot = 0.0;
        }
        else
        {
            cur_st_point.s_dot_dot = (cur_st_point.s_dot - previous_s_dot) / dt;
        }
        previous_s_dot = cur_st_point.s_dot;
        speed_profile_init.emplace_back(cur_st_point);
    }
    STPoint cur_st_point;
    cur_st_point.s = fpb_path.back().s;
    cur_st_point.s_dot = 0;
    cur_st_point.s_dot_dot = 0;
    cur_st_point.t = static_cast<double>((fpb_path.size() - 1) * dt);
    speed_profile_init.emplace_back(cur_st_point);

    // Get increased speed_profile
    for (size_t i = 0; i < speed_profile_init.size() - 1; i++)
    {
        STPoint start_point, end_point;
        start_point = speed_profile_init[i];
        end_point = speed_profile_init[i + 1];
        int sample_num = static_cast<int>((end_point.t - start_point.t) / increased_time);
        PolynomialCurve curve;
        curve.curve_fitting(start_point.t, start_point.s, start_point.s_dot, start_point.s_dot_dot,
                            end_point.t, end_point.s, end_point.s_dot, end_point.s_dot_dot);
        for (int j = 0; j < sample_num; j++)
        {
            STPoint increased_st_point;
            increased_st_point.t = start_point.t + j * increased_time;
            increased_st_point.s = curve.value_evaluation(increased_st_point.t, 0);
            increased_st_point.s_dot = curve.value_evaluation(increased_st_point.t, 1);
            increased_st_point.s_dot_dot = curve.value_evaluation(increased_st_point.t, 2);
            speed_profile_increased.emplace_back(increased_st_point);
        }
    }
    speed_profile_increased.emplace_back(speed_profile_init.back());
    return true;
}

bool UMBPlanner::GenerateTrajectory(const std::vector<STPoint> &final_speed_profile, const std::vector<TrajectoryPoint> &path_trajectory,
                                    const std::vector<double> &path_index2s, const double &planning_start_point_time_stamped,
                                    std::vector<TrajectoryPoint> &trajectory)
{
    for (size_t i = 0; i < final_speed_profile.size(); i++)
    {
        using namespace std::chrono_literals;
        TrajectoryPoint trajectory_point;
        // 从final_speed_profile中读取 v,a,t
        trajectory_point.v = std::min(final_speed_profile[i].s_dot, _reference_speed * 1.2);
        trajectory_point.a_tau = final_speed_profile[i].s_dot_dot;
        trajectory_point.time_stamped = planning_start_point_time_stamped + final_speed_profile[i].t;

        // 从path_index2s中读取 x,y, heading, kappa
        double cur_s = final_speed_profile[i].s;
        int nearest_index = 0;
        if (cur_s >= path_index2s.back())
        {
            nearest_index = path_index2s.size() - 1;
        }
        if (cur_s <= path_index2s.front())
        {
            nearest_index = 0;
        }
        for (size_t j = 0; j < path_index2s.size() - 1; j++)
        {
            if (cur_s >= path_index2s[j] && cur_s < path_index2s[j + 1])
            {
                nearest_index = j;
            }
        }
        trajectory_point.x = path_trajectory[nearest_index].x +
                             ((path_trajectory[nearest_index + 1].x - path_trajectory[nearest_index].x) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);

        trajectory_point.y = path_trajectory[nearest_index].y +
                             ((path_trajectory[nearest_index + 1].y - path_trajectory[nearest_index].y) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);

        trajectory_point.heading = path_trajectory[nearest_index].heading +
                                   ((path_trajectory[nearest_index + 1].heading - path_trajectory[nearest_index].heading) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);

        trajectory_point.kappa = path_trajectory[nearest_index].kappa +
                                 ((path_trajectory[nearest_index + 1].kappa - path_trajectory[nearest_index].kappa) / (path_index2s[nearest_index + 1] - path_index2s[nearest_index])) * (cur_s - path_index2s[nearest_index]);
        trajectory.emplace_back(trajectory_point);
    }
    return true;
}

double UMBPlanner::GetMinDistanceFromEgoToObs(const FrenetPoint &forward_traj, const FrenetPoint &surr_traj,
                                              const double obs_length, const double obs_width)
{
    std::vector<FrenetPoint> obs_points;
    FrenetPoint point_1, point_2, point_3, point_4;
    point_1.s = surr_traj.s + obs_length / 2.0;
    point_2.s = surr_traj.s + obs_length / 2.0;
    point_3.s = surr_traj.s - obs_length / 2.0;
    point_4.s = surr_traj.s - obs_length / 2.0;
    point_1.l = surr_traj.l + obs_width / 2.0;
    point_2.l = surr_traj.l - obs_width / 2.0;
    point_3.l = surr_traj.l - obs_width / 2.0;
    point_4.l = surr_traj.l + obs_width / 2.0;
    obs_points.emplace_back(point_1);
    obs_points.emplace_back(point_2);
    obs_points.emplace_back(point_3);
    obs_points.emplace_back(point_4);

    // return std::hypot((forward_traj.s - surr_traj.s), (forward_traj.l - surr_traj.l));
    double min_distance = std::numeric_limits<double>::max();
    if (forward_traj.s <= point_2.s && forward_traj.s >= point_4.s && forward_traj.l >= point_2.l && forward_traj.l <= point_4.l)
    {
        min_distance = 0.01;
        return min_distance;
    }

    for (auto &point : obs_points)
    {
        double cur_distance = std::hypot((forward_traj.s - point.s), (forward_traj.l - point.l));
        if (cur_distance < min_distance)
        {
            min_distance = cur_distance;
        }
    }
    return min_distance;
}

bool UMBPlanner::GenerateSscCube(const std::vector<FrenetPoint> &fpb_path, const ForwardPropAgentSet &forward_prop_agent_set,
                                 const std::vector<TrajectoryPoint> &path_trajectory, const FrenetPoint &planning_start_point_frenet)
{
    _cubes.clear();
    std::unordered_map<int, std::vector<FrenetPoint>> cur_surround_trajs = _surround_trajs[_winner_id];
    std::vector<FrenetPoint> dynamic_obstacles_frenet_coords;
    if (!_dynamic_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_dynamic_obstacles, path_trajectory, path_trajectory.front(), dynamic_obstacles_frenet_coords);
    }

    for (auto &&obs_frenet : dynamic_obstacles_frenet_coords)
    {
        LOG(INFO) << std::fixed << std::setprecision(4)
                  << "dynamic_obstacles_frenet_coords(s: " << obs_frenet.s
                  << ", l:" << obs_frenet.l << ", s_dot:" << obs_frenet.s_dot << ", l_dot:" << obs_frenet.l_dot
                  << " )";
    }
    for (size_t i = 0; i < fpb_path.size() - 1; i++)
    {
        SpatioTemporalSemanticCubeNd<2> cube;

        cube.t_lb = static_cast<double>(i * _sim_param.layer_time);
        cube.t_ub = cube.t_lb + _sim_param.layer_time;

        // * the SL bound of current cube
        double pos_lb_s, pos_lb_l, pos_ub_s, pos_ub_l;
        if (forward_prop_agent_set.forward_prop_agents.empty())
        {
            pos_lb_s = -5.0;
            pos_ub_s = _sim_param.s_sample_distance * _sim_param.s_sample_num;
            pos_lb_l = -1 * _sim_param.l_ref_to_right_road_bound;
            pos_ub_l = _sim_param.l_ref_to_left_road_bound;
            // LOG(INFO) << "[GenerateSscCube] " << i << " cube.pos_lb_l " << pos_lb_l
            //           << " cube.pos_ub_l " << pos_ub_l;
        }
        else
        {
            double min_l = -1 * _sim_param.l_ref_to_right_road_bound;
            double max_l = _sim_param.l_ref_to_left_road_bound;
            for (const auto &fpa : forward_prop_agent_set.forward_prop_agents)
            {
                // * the L bound of current cube
                double t_in_l = -1, t_out_l = -1;
                bool find_t_in_l = false;
                bool find_t_out_l = false;
                for (size_t j = 0; j < fpb_path.size() - 1; j++)
                {
                    if (cur_surround_trajs.at(fpa.first)[i].s - fpa.second.length / 2 >= fpb_path[j].s && cur_surround_trajs.at(fpa.first)[i].s - fpa.second.length / 2 <= fpb_path[j + 1].s && !find_t_in_l)
                    {
                        t_in_l = static_cast<double>(j * _sim_param.layer_time);
                        find_t_in_l = true;
                    }

                    if (cur_surround_trajs.at(fpa.first)[i].s + fpa.second.length / 2 >= fpb_path[j].s && cur_surround_trajs.at(fpa.first)[i].s + fpa.second.length / 2 <= fpb_path[j + 1].s && !find_t_out_l)
                    {
                        t_out_l = static_cast<double>((j + 1) * _sim_param.layer_time);
                        find_t_out_l = true;
                    }
                }

                // LOG(INFO) << "[GenerateSscCube] obs[" << fpa.first << "] at[" << i << "]:" << " t_in_l: " << t_in_l
                //           << ", t_out_l: " << t_out_l;

                LOG(INFO) << "[GenerateSscCube] cur_surround_trajs of obs[" << fpa.first << "] at[" << i << "]:" << " s: " << cur_surround_trajs.at(fpa.first)[i].s
                          << ", l: " << cur_surround_trajs.at(fpa.first)[i].l;

                if (static_cast<double>(i * _sim_param.layer_time) >= t_in_l && static_cast<double>(i * _sim_param.layer_time) <= t_out_l)
                {
                    if (fpb_path[i].l >= cur_surround_trajs.at(fpa.first)[i].l)
                    {
                        min_l = std::max(min_l, cur_surround_trajs.at(fpa.first)[i].l + fpa.second.width / 2 + _ego_param.car_width / 2);
                    }
                    else
                    {
                        max_l = std::min(max_l, cur_surround_trajs.at(fpa.first)[i].l - fpa.second.width / 2 - _ego_param.car_width / 2);
                    }
                }

                // TODO:dynamic obs, change bound of S
                // if (std::hypot(fpa.second.obs_frenet_point.l_dot, fpa.second.obs_frenet_point.s_dot) >= 0.1)
                // {
                //     pos_lb_s = -5.0;
                //     pos_ub_s = _sim_param.s_sample_distance * _sim_param.s_sample_num;
                // }
            }
            pos_lb_s = -5.0;
            pos_ub_s = _sim_param.s_sample_distance * _sim_param.s_sample_num;
            min_l = -1 * _sim_param.l_ref_to_right_road_bound;
            max_l = _sim_param.l_ref_to_left_road_bound;
            pos_lb_l = min_l;
            pos_ub_l = max_l;
            if (pos_ub_l <= pos_lb_l)
            {
                pos_ub_l = _sim_param.l_ref_to_left_road_bound;
                pos_lb_l = -1.0 * _sim_param.l_ref_to_right_road_bound;
            }
            // LOG(INFO) << "[GenerateSscCube] cube[" << i << "] cube.pos_lb_l: " << pos_lb_l
            //           << ", cube.pos_ub_l: " << pos_ub_l;
            // LOG(INFO) << "------------------------------";
        }
        cube.p_lb = {pos_lb_s, pos_lb_l};
        cube.p_ub = {pos_ub_s, pos_ub_l};
        cube.v_lb = {_bezier_param.v_lb_s, _bezier_param.v_lb_l};
        cube.v_ub = {_bezier_param.v_ub_s, _bezier_param.v_ub_l};
        cube.a_lb = {_bezier_param.a_lb_s, _bezier_param.a_lb_l};
        cube.a_ub = {_bezier_param.a_ub_s, _bezier_param.a_ub_l};
        _cubes.emplace_back(cube);
    }
    return true;
}

std::vector<TrajectoryPoint> UMBPlanner::BezierPointToTrajectory(std::vector<FrenetPoint> &frenet_point_set, const std::shared_ptr<std::vector<PathPoint>> cartesian_path,
                                                                 const std::vector<double> cartesian_path_index2s, const double &planning_start_point_time_stamped)
{
    for (auto &point : frenet_point_set)
    {
        point.l_prime = point.l_dot / (point.s_dot + 1e-3);
        point.l_prime_prime = (point.l_dot_dot - point.l_prime * point.s_dot_dot) / (point.s_dot * point.s_dot + 1e-3);
    }

    std::vector<TrajectoryPoint> trajectory_point_set;

    for (size_t i = 0; i < frenet_point_set.size(); i++)
    {
        FrenetPoint frenet_point_host = frenet_point_set[i];
        // 1.寻找匹配点
        // 处理边界情况
        int match_point_index = -1;
        if (frenet_point_host.s < cartesian_path_index2s.front())
        {
            match_point_index = 0;
        }
        else if (frenet_point_host.s > cartesian_path_index2s.back())
        {
            match_point_index = frenet_point_set.size() - 1;
        }
        else
        {
            for (size_t j = 0; j < cartesian_path_index2s.size(); j++)
            {
                if (frenet_point_host.s >= cartesian_path_index2s[j] && frenet_point_host.s < cartesian_path_index2s[j + 1])
                {
                    if (std::abs(frenet_point_host.s - cartesian_path_index2s[j]) > std::abs(frenet_point_host.s - cartesian_path_index2s[j + 1]))
                    {
                        match_point_index = j + 1;
                        break;
                    }
                    else
                    {
                        match_point_index = j;
                        break;
                    }
                }
            }
        }
        PathPoint match_point = (*cartesian_path)[match_point_index];

        // 2匹配点的位置向量、切向量
        Eigen::Vector2d p_match_point(match_point.x, match_point.y);
        Eigen::Vector2d tau_m(std::cos(match_point.heading), std::sin(match_point.heading));

        // 3投影点
        double delta_s = frenet_point_host.s - cartesian_path_index2s[match_point_index];
        Eigen::Vector2d p_project_point = p_match_point + delta_s * tau_m;
        double kappa_project_point = match_point.kappa;
        double heading_project_point = match_point.heading + delta_s * kappa_project_point;
        Eigen::Vector2d nor_project_point(-std::sin(heading_project_point), std::cos(heading_project_point));

        // 4坐标转换，公式见讲义
        TrajectoryPoint trajectory_point_host;
        Eigen::Vector2d p_host = p_project_point + frenet_point_host.l * nor_project_point;
        trajectory_point_host.x = p_host[0];
        trajectory_point_host.y = p_host[1];
        double c = 1.0 - kappa_project_point * frenet_point_host.l;
        double dealta_heading = std::atan2(frenet_point_host.l_prime, c);
        trajectory_point_host.heading = dealta_heading + heading_project_point;
        trajectory_point_host.kappa = ((frenet_point_host.l_prime_prime + kappa_project_point * frenet_point_host.l_prime * std::tan(dealta_heading)) * std::pow(std::cos(dealta_heading), 2) / c + kappa_project_point) * std::cos(dealta_heading) / c;
        trajectory_point_host.v = std::min(frenet_point_host.s_dot, _reference_speed * 1.2);
        trajectory_point_host.a_tau = frenet_point_host.s_dot_dot;
        trajectory_point_host.time_stamped = planning_start_point_time_stamped + frenet_point_host.t;

        trajectory_point_set.emplace_back(trajectory_point_host);
    }
    return trajectory_point_set;
}