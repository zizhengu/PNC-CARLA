#include "umb_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits>
auto UMBP_LOG = rclcpp::get_logger("UMBPlanner");

UMBPlanner::UMBPlanner() : Node("UMBPlanner")
{
    google::InitGoogleLogging("UMBP");
    google::SetLogDestination(google::GLOG_INFO, "/home/guzizhen/PNC-CARLA/debug_log/umbp_log/");
    google::SetLogDestination(google::GLOG_WARNING, "/home/guzizhen/PNC-CARLA/debug_log/umbp_log/");
    google::SetLogDestination(google::GLOG_ERROR, "/home/guzizhen/PNC-CARLA/debug_log/umbp_log/");
    LOG(INFO) << std::fixed << std::setprecision(4)
              << "[UMBP]******************** RUN START: " << "******************";

    _get_waypoint_client = this->create_client<carla_waypoint_types::srv::GetWaypoint>(
        "/carla_waypoint_publisher/ego_vehicle/get_waypoint");
    _reference_speed = 10.0;
    std::string config_path = "/home/guzizhen/PNC-CARLA/my_planning_and_control/planner_modules/config/umbp_config.pb.txt";

    ReadConfig(config_path);
    GetSimParam(_cfg, &_sim_param);
    GetCostParam(_cfg, &_cost_param);

    LOG(INFO) << " [UMBP]_sim_param.tree_height " << _sim_param.tree_height << " done!";
    LOG(INFO) << " [UMBP]_sim_param.layer_time " << _sim_param.layer_time << " done!";
    LOG(INFO) << " [UMBP]_sim_param.s_sample_distance " << _sim_param.s_sample_distance << " done!";
    LOG(INFO) << " [UMBP]_sim_param.s_sample_num " << _sim_param.s_sample_num << " done!";

    _fpb_tree_ptr = std::make_shared<FpbTree>(_sim_param.tree_height, _sim_param.layer_time);
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
    sim_param->s_sample_distance = cfg.propogate().sample().s_sample_distance();
    sim_param->s_sample_num = cfg.propogate().sample().s_sample_num();
    sim_param->l_sample_distance = cfg.propogate().sample().l_sample_distance();
    sim_param->l_sample_num = cfg.propogate().sample().l_sample_num();
    sim_param->acc_ref = cfg.propogate().sample().acc_ref();
    sim_param->dec_ref = cfg.propogate().sample().dec_ref();
    return true;
}

bool UMBPlanner::GetCostParam(const planning::umbp::Config &cfg,
                              CostParam *cost_param)
{
    cost_param->ego_lack_speed_to_desired_unit_cost = cfg.cost().efficiency().ego_lack_speed_to_desired_unit_cost();
    cost_param->ego_to_obs = cfg.cost().safety().ego_to_obs();
    cost_param->ref_line_change = cfg.cost().navigation().ref_line_change();
    return true;
}

bool UMBPlanner::RunOnce(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state,
                         const std::vector<derived_object_msgs::msg::Object> &obstacles, std::vector<TrajectoryPoint> &trajectory)
{
    _current_time = this->now().seconds();
    _current_ego_state = ego_state;
    //-----------------------------------1.障碍物处理--------------------------------------
    // 1.1区别动态与静态障碍物，将其储存在成员变量 std::vector<derived_object_msgs::msg::Object> _static_/dynamic_obstacles中
    UMBPlanner::ObstacleFileter(ego_state, obstacles);
    std::vector<FrenetPoint> static_obs_frent_coords;
    std::vector<FrenetPoint> dynamic_obs_frent_coords;
    // 1.2障碍物坐标转换
    if (!_static_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_static_obstacles, *reference_line, ego_state, static_obs_frent_coords);
    }
    if (!_dynamic_obstacles.empty())
    {
        cartesion_set_to_frenet_set(_dynamic_obstacles, *reference_line, ego_state, dynamic_obs_frent_coords);
    }

    // 1.3 障碍物转换为前向传播智能体
    ForwardPropAgentSet forward_prop_agent_set;
    if (!GetSurroundingForwardSimAgents(forward_prop_agent_set, static_obs_frent_coords, dynamic_obs_frent_coords))
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

    //----------------------------2.确定规划起点并将其投影到frenet坐标系--------------------------
    TrajectoryPoint planning_start_point = CalculatePlanningStartPoint(ego_state);
    std::vector<FrenetPoint> temp_set;
    cartesion_set_to_frenet_set(planning_start_point, *reference_line, ego_state, temp_set);
    FrenetPoint planning_start_point_frenet = temp_set.front();
    planning_start_point_frenet.l_prime_prime = 0.0;

    //----------------------------3.进行Frenet采样--------------------------
    std::vector<std::vector<FrenetPoint>> frenet_sample_point;
    GetFrenetSamplePoints(frenet_sample_point, planning_start_point_frenet,
                          _sim_param.s_sample_distance, _sim_param.s_sample_num,
                          _sim_param.l_sample_distance, _sim_param.l_sample_num);
    _local_sample_points = frenet_sample_point;

    //----------------------------4.进行FPB-Tree前向传播--------------------------
    TicToc timer;
    if (!RunUmpb(planning_start_point_frenet, forward_prop_agent_set))
    {
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[UMBP]****** UMBP Cycle FAILED  ******";
        return false;
    }

    std::vector<FrenetPoint> Fpb_win_points = _forward_trajs[_winner_id];

    for (const auto &points : Fpb_win_points)
    {
        RCLCPP_INFO(UMBP_LOG, "Fpb_win_points: (s: %.3f,l: %.3f)", points.s, points.l);
    }
    RCLCPP_INFO(UMBP_LOG, "----------------------------------");
    exit(0);
    return true;
}

void UMBPlanner::GetFrenetSamplePoints(std::vector<std::vector<FrenetPoint>> &frenet_sample_point, const FrenetPoint &planning_start_sl,
                                       const double &s_sample_distance, const int &s_sample_num,
                                       const double &l_sample_distance, const int &l_sample_num)
{
    frenet_sample_point.emplace_back();
    frenet_sample_point.back().emplace_back(planning_start_sl);
    for (int level = 1; level <= s_sample_num; level++)
    {
        frenet_sample_point.emplace_back();
        for (int i = 0; i <= l_sample_distance; i++)
        {
            FrenetPoint current_point;
            current_point.s = planning_start_sl.s + level * s_sample_distance;
            current_point.l = (int(l_sample_num / 2) - i) * l_sample_distance;
            frenet_sample_point.back().emplace_back(current_point);
        }
    }
}

bool UMBPlanner::RunUmpb(const FrenetPoint ego_state, const ForwardPropAgentSet &forward_prop_agent_set)
{
    auto behaviour_tree = _fpb_tree_ptr->get_behaviour_tree();
    int num_sequence = behaviour_tree.size();
    // * prepare for multi-threading
    std::vector<std::thread> thread_set(num_sequence);
    PrepareMultiThreadContainers(num_sequence);

    // * threading
    TicToc multi_thread_timer;
    multi_thread_timer.tic();
    for (size_t i = 0; i < static_cast<size_t>(num_sequence); i++)
    {
        thread_set[i] = std::thread(&UMBPlanner::PropagateActionSequence, this, ego_state,
                                    forward_prop_agent_set, behaviour_tree[i], i);
    }

    for (size_t i = 0; i < static_cast<size_t>(num_sequence); i++)
    {
        thread_set[i].join();
    }

    // * Summary results
    for (int i = 0; i < num_sequence; ++i)
    {
        std::ostringstream line_info;
        line_info << "[UMBP][Result]" << i << " [";
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
        if (_sim_res[i])
        {
            line_info << "[UMBP][Result][e;s;n:";
            for (const auto &c : _progress_cost[i])
            {
                line_info << std::fixed << std::setprecision(2)
                          << c.efficiency.ego_to_desired_vel << ";"
                          << c.safety.ego_to_obs << ";"
                          << c.navigation.ref_line_change;
                line_info << "|";
            }
            line_info << "]";
        }
        LOG(INFO) << line_info.str();
    }
    LOG(WARNING) << "[UMBP][Result]Propagate sucess with "
                 << num_sequence << " behaviors.";

    // * Evaluate
    if (!EvaluateMultiThreadSimResults(&_winner_id, &_winner_score))
    {
        LOG(ERROR)
            << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
        return false;
    }
    LOG(INFO) << std::fixed << std::setprecision(2)
              << "[UMBP][Result] Best id: "
              << _winner_id << ";"
              << "Best socre" << _winner_score;
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
        size_t current_time_index = -1;
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
            size_t start_time_index = -1;
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

        double v_obs = std::sqrt(std::pow(obs.twist.linear.x, 2.0) + std::pow(obs.twist.linear.y, 2.0) + std::pow(obs.twist.linear.z, 2.0)); // 障碍物速度
        Eigen::Vector2d host_to_obs(obs.pose.position.x - ego_state->x, obs.pose.position.y - ego_state->y);                                 // 主车到障碍物的向量
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        double longitudinal_d = host_to_obs.dot(tau_host); // 纵向距离
        double lateral_d = host_to_obs.dot(nor_host);      // 横向距离
        // 静态障碍物，即使有加速度，一个规划周期是0.1s，障碍物以最大加速度加速也达不到很大的速度
        if (v_obs <= 0.1)
        {
            // 超出1.5个车长即变道
            if (longitudinal_d <= 60 && longitudinal_d >= -7.5 && lateral_d <= 10 && lateral_d >= -10)
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
            if (longitudinal_d <= 60 && longitudinal_d > -10 && lateral_d <= 25 && lateral_d >= -25)
            {
                _dynamic_obstacles.push_back(obs);
            }
            else
            {
                continue;
            }
        }
    }
}

bool UMBPlanner::GetSurroundingForwardSimAgents(ForwardPropAgentSet &forward_prop_agent_set,
                                                const std::vector<FrenetPoint> static_obs_frent_coords,
                                                const std::vector<FrenetPoint> dynamic_obs_frent_coords)
{
    int obs_id = 1;
    for (auto &&obs : static_obs_frent_coords)
    {
        ForwardPropAgent cur_agent;
        cur_agent.id = obs_id;
        cur_agent.obs_frenet_point.s = obs.s;
        cur_agent.obs_frenet_point.l = obs.l;
        cur_agent.obs_frenet_point.s_dot = obs.s_dot;
        cur_agent.obs_frenet_point.l_dot = obs.l_dot;
        cur_agent.obs_frenet_point.s_dot_dot = obs.s_dot_dot;
        cur_agent.obs_frenet_point.l_dot_dot = obs.l_dot_dot;
        forward_prop_agent_set.forward_prop_agents.emplace(obs_id, cur_agent);
        obs_id++;
    }
    for (auto &&obs : dynamic_obs_frent_coords)
    {
        ForwardPropAgent cur_agent;
        cur_agent.id = obs_id;
        cur_agent.obs_frenet_point.s = obs.s;
        cur_agent.obs_frenet_point.l = obs.l;
        cur_agent.obs_frenet_point.s_dot = obs.s_dot;
        cur_agent.obs_frenet_point.l_dot = obs.l_dot;
        cur_agent.obs_frenet_point.s_dot_dot = obs.s_dot_dot;
        cur_agent.obs_frenet_point.l_dot_dot = obs.l_dot_dot;
        forward_prop_agent_set.forward_prop_agents.emplace(obs_id, cur_agent);
        obs_id++;
    }
    return true;
}

bool UMBPlanner::PrepareMultiThreadContainers(const int num_sequence)
{
    LOG(INFO) << " [UMBP][Process]Prepare multi-threading- " << num_sequence;
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
        std::vector<CostStructure> progress_cost(n_sub_threads);
        CostStructure tail_cost;
        std::vector<FrenetPoint> forward_trajs;
        std::vector<FpbLatAction> forward_lat_behaviors;
        std::vector<FpbLonAction> forward_lon_behaviors;
        std::unordered_map<int, std::vector<FrenetPoint>> surround_trajs;

        if (!PropagateScenario(ego_state, surrounding_fsagents, action_seq, seq_id, 0, &sim_res,
                               &risky_res, &sim_info, &progress_cost, &tail_cost, &forward_trajs,
                               &forward_lat_behaviors, &forward_lon_behaviors, &surround_trajs))
        {
            LOG(ERROR) << std::fixed << std::setprecision(4)
                       << "[UMBP]****** PropogateScenario FAILED  ******";
            return false;
        }
        sub_sim_res.emplace_back(sim_res);
        sub_risky_res.emplace_back(risky_res);
        sub_sim_info.emplace_back(sim_info);
        sub_progress_cost.emplace_back(progress_cost);
        sub_tail_cost.emplace_back(tail_cost);
        sub_forward_trajs.emplace_back(forward_trajs);
        sub_forward_lat_behaviors.emplace_back(forward_lat_behaviors);
        sub_forward_lon_behaviors.emplace_back(forward_lon_behaviors);
        sub_surround_trajs.emplace_back(surround_trajs);
    }

    double min_cost = std::numeric_limits<double>::max();
    int index = -1;
    for (int i = 0; i < n_sub_threads; i++)
    {
        if (sub_tail_cost[i].ave() < min_cost)
        {
            min_cost = sub_tail_cost[i].ave();
            index = i;
        }
        else
        {
            continue;
        }
    }
    if (index == -1)
    {
        return false;
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[UMBP]****** Find min cost FAILED  ******";
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
    // declare variables which will be used to track traces from multiple layers
    sub_forward_trajs->emplace_back(ego_state);
    sub_forward_lat_behaviors->emplace_back(FpbLatAction::Keeping);
    sub_forward_lon_behaviors->emplace_back(FpbLonAction::Maintain);

    for (const auto &fpa : surrounding_fsagents.forward_prop_agents)
    {
        sub_surround_trajs->emplace(fpa.first, std::vector<FrenetPoint>({fpa.second.obs_frenet_point}));
        for (size_t i = 0; i < action_seq.size() + 2; i++)
        {
            FrenetPoint new_frenet_point(sub_surround_trajs->at(fpa.first).back());
            new_frenet_point.l += sub_surround_trajs->at(fpa.first).back().l_dot * _sim_param.l_sample_num;
            new_frenet_point.s += sub_surround_trajs->at(fpa.first).back().s_dot * _sim_param.l_sample_num;
            /**
             * 使用 push_back  points.push_back(p);  // 需要先创建对象，再拷贝/移动
             * 使用 emplace_back points.emplace_back(1, 2);  // 直接在容器中构造对象，无需拷贝/移动
             */
            sub_surround_trajs->at(fpa.first).emplace_back(new_frenet_point);
        }
    }

    // get ego next sample point
    bool is_lat_action_done = false;
    int delta_l_num = 0, delta_s_num = 0, s_index = 0, l_index = (int)(_sim_param.l_sample_num / 2);
    FrenetPoint next_sample_point;
    for (size_t i = 0; i < action_seq.size(); i++)
    {
        delta_l_num = 0;
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
            delta_l_num = 1;
            is_lat_action_done = true;
        }
        if (action_this_layer.lat == FpbLatAction::SwitchRight && !is_lat_action_done)
        {
            delta_l_num = -1;
            is_lat_action_done = true;
        }

        // get next_sample_point
        s_index += delta_s_num;
        l_index += delta_l_num;
        next_sample_point = _local_sample_points[s_index][l_index];

        sub_forward_trajs->emplace_back(next_sample_point);
        sub_forward_lat_behaviors->emplace_back(action_this_layer.lat);
        sub_forward_lon_behaviors->emplace_back(action_this_layer.lon);
    }

    for (int i = 0; i < 2; i++)
    {
        delta_l_num = 0;
        s_index += delta_s_num;
        l_index += delta_l_num;
        next_sample_point = _local_sample_points[s_index][l_index];

        sub_forward_trajs->emplace_back(next_sample_point);
        sub_forward_lat_behaviors->emplace_back(action_seq.back().lat);
        sub_forward_lon_behaviors->emplace_back(action_seq.back().lon);
    }

    // calculate cost
    bool is_risky = false;
    std::set<size_t> risky_ids;
    if (!CalculateCost(*sub_forward_trajs, *sub_surround_trajs, sub_progress_cost, &is_risky, &risky_ids))
    {
        return false;
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[UMBP]****** CalculateCost Error  ******";
    }

    for (size_t i = 0; i < sub_progress_cost->size(); i++)
    {
        sub_tail_cost->efficiency.ego_to_desired_vel += sub_progress_cost->at(i).efficiency.ego_to_desired_vel;
        sub_tail_cost->navigation.ref_line_change += sub_progress_cost->at(i).navigation.ref_line_change;
        sub_tail_cost->safety.ego_to_obs += sub_progress_cost->at(i).safety.ego_to_obs;
    }

    if (is_risky)
    {
        *sub_risky_res = 1;
    }

    *sub_sim_res = 1;
    return true;
}

bool UMBPlanner::CalculateCost(const std::vector<FrenetPoint> &forward_trajs,
                               const std::unordered_map<int, std::vector<FrenetPoint>> &surround_trajs,
                               std::vector<CostStructure> *progress_cost, bool *is_risky, std::set<size_t> *risky_ids)
{

    for (size_t i = 0; i < forward_trajs.size(); i++)
    {
        CostStructure cost_tmp;
        // navigation
        cost_tmp.navigation.ref_line_change += (_cost_param.ref_line_change * std::abs(forward_trajs[i].l)) * std::pow(_cost_param.discount_factor, i);

        // efficiency：包含自车与期望速度之间的差异代价
        double ego_velocity;
        if (i == static_cast<size_t>(0))
        {
            ego_velocity = std::hypot((forward_trajs[i].s), (forward_trajs[i].l)) / (_sim_param.layer_time);
        }
        else
        {
            ego_velocity = std::hypot((forward_trajs[i].s - forward_trajs[i - 1].s), (forward_trajs[i].l - forward_trajs[i - 1].l)) / (_sim_param.layer_time);
        }
        cost_tmp.efficiency.ego_to_desired_vel = _cost_param.ego_lack_speed_to_desired_unit_cost * std::abs(ego_velocity - _reference_speed);

        // safety:
        for (const auto &surr_traj : surround_trajs)
        {
            if (forward_trajs.size() != surr_traj.second.size())
            {
                return false;
                LOG(ERROR) << std::fixed << std::setprecision(4)
                           << "[UMBP]****** forward_trajs.size() != surr_traj.second.size()  ******";
            }
            double distance = std::hypot((forward_trajs[i].s - surr_traj.second[i].s), (forward_trajs[i].l - surr_traj.second[i].l));
            if (distance >= 4)
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
        }
        progress_cost->emplace_back(cost_tmp);
    }
    return true;
}

bool UMBPlanner::EvaluateMultiThreadSimResults(int *winner_id, double *winner_cost)
{
    double min_cost = std::numeric_limits<double>::max();
    int best_id = 0;
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
        }
    }
    *winner_id = best_id;
    *winner_cost = min_cost;
    return true;
}