#include "umb_planner.h"
auto LOG = rclcpp::get_logger("UMBPlanner");

UMBPlanner::UMBPlanner() : Node("UMBPlanner")
{
    _get_waypoint_client = this->create_client<carla_waypoint_types::srv::GetWaypoint>(
        "/carla_waypoint_publisher/ego_vehicle/get_waypoint");
    _reference_speed = 11.0;
}

bool UMBPlanner::RunOnce(const std::shared_ptr<std::vector<PathPoint>> reference_line, const std::shared_ptr<VehicleState> ego_state,
                         const std::vector<derived_object_msgs::msg::Object> &obstacles, std::vector<TrajectoryPoint> &trajectory)
{
    _current_time = this->now().seconds();
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
        RCLCPP_INFO(LOG, "GetSurroundingForwardSimAgents False!");
        return false;
    }
    else
    {
        for (const auto &pair : forward_prop_agent_set.forward_prop_agents)
        {
            const auto &agent = pair.second;
            RCLCPP_INFO(LOG, "Agent_ID %.i: (s: %.3f,l: %.3f,s_dot: %.3f,l_dot: %.3f)",
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

bool UMBPlanner::RunUmpb()
{
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
    RCLCPP_INFO(LOG, "[UMBP][Process]Prepare multi-threading-%.i ", num_sequence);

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
    auto LOG = rclcpp::get_logger("emplanner");
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
