#include "fpb_tree.h"

FpbTree::FpbTree(const int &tree_height, const double &action_time)
{

    tree_height_ = tree_height;
    action_time_ = action_time;
    GenarateFpbTree();
}

bool FpbTree::GenarateFpbTree()
{
    behaviour_tree_.clear();
    std::vector<FpbAction> behaviour_seq;
    FpbAction temp_behaviour;
    int num_lon_action = (int)FpbLonAction::MAX_COUNT;
    // std::vector<std::vector<FpbTree::FpbLatAction>> valid_lat_behaviour_seqs = GenerateLatActionSequences();
    std::vector<std::vector<FpbTree::FpbLatAction>> valid_lat_behaviour_seqs = GenerateValidCombinations(tree_height_);
    for (int lon = 0; lon < num_lon_action; lon++)
    {
        // KKK, LLL, RRR
        for (auto lat_behaviour_seq : valid_lat_behaviour_seqs)
        {
            behaviour_seq.clear();
            for (auto lat_behaviour : lat_behaviour_seq)
            {
                temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(lat_behaviour), action_time_);
                behaviour_seq.push_back(temp_behaviour);
            }
            behaviour_tree_.push_back(behaviour_seq);
        }
    }
    return true;
}

// bool FpbTree::GenarateFpbTree()
// {
//     behaviour_tree_.clear();
//     std::vector<FpbAction> behaviour_seq;
//     FpbAction temp_behaviour;
//     int num_lon_action = (int)FpbLonAction::MAX_COUNT;
//     // std::vector<std::vector<FpbTree::FpbLatAction>> valid_lat_behaviour_seqs = GenerateLatActionSequences();
//     std::vector<std::vector<FpbTree::FpbLatAction>> valid_lat_behaviour_seqs = GenerateValidCombinations(tree_height_);
//     for (int lon = 0; lon < num_lon_action; lon++)
//     {

//         // KLLLL
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KRRRR
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKLLL
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKRRR
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKKLL
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKKRR
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKKKL
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(1), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKKKR
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(2), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);

//         // KKKKK
//         behaviour_seq.clear();
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
//         behaviour_seq.push_back(temp_behaviour);
//         behaviour_tree_.push_back(behaviour_seq);
//     }
//     return true;
// }

bool FpbTree::isValidCombination(const std::vector<FpbLatAction> &combination)
{
    int turnCount = 0;
    for (size_t i = 0; i < combination.size() - 1; ++i)
    {
        // 检查 L 后面不能跟 R, R 后面不能跟 L
        if ((combination[i] == FpbLatAction::SwitchLeft && combination[i + 1] == FpbLatAction::SwitchRight) ||
            (combination[i] == FpbLatAction::SwitchRight && combination[i + 1] == FpbLatAction::SwitchLeft))
        {
            return false;
        }

        // 避免连续的转向动作
        // 规则：如果某个动作已经是 SwitchLeft 或 SwitchRight，下一个动作必须是 Keeping 或至少要有一个 K（保持）来平稳过渡。
        if (combination[i] == FpbLatAction::SwitchLeft || combination[i] == FpbLatAction::SwitchRight)
        {
            if (combination[i + 1] == FpbLatAction::SwitchLeft || combination[i + 1] == FpbLatAction::SwitchRight)
            {
                return false; // 连续转向，不合逻辑
            }
        }

        // 限制最大连续时间内转向次数
        if (combination[i] == FpbLatAction::SwitchLeft || combination[i] == FpbLatAction::SwitchRight)
        {
            turnCount++;
            if (turnCount > 3)
            { // 超过两次连续转向，返回不合法
                return false;
            }
        }
        else
        {
            turnCount = 0; // 重置计数
        }
    }

    return true;
}

std::vector<std::vector<FpbTree::FpbLatAction>> FpbTree::GenerateValidCombinations(const int n)
{
    std::vector<std::vector<FpbLatAction>> result;

    // 枚举所有可能的组合，三种选择（K=0, L=1, R=2）每秒一个
    int totalCombinations = 3; // 每秒三种选择
    int totalSequences = 1;
    for (int i = 0; i < n; ++i)
    {
        totalSequences *= totalCombinations; // 计算总组合数
    }

    // 枚举每一种组合
    for (int seq = 0; seq < totalSequences; ++seq)
    {
        std::vector<FpbLatAction> combination;
        int temp = seq;

        // 对每个序列进行编码，3个状态（K=0, L=1, R=2）
        for (int i = 0; i < n; ++i)
        {
            int action = temp % 3;
            if (action == 0)
            {
                combination.push_back(FpbLatAction::Keeping);
            }
            else if (action == 1)
            {
                combination.push_back(FpbLatAction::SwitchLeft);
            }
            else
            {
                combination.push_back(FpbLatAction::SwitchRight);
            }
            temp /= 3;
        }

        // 检查组合是否合法
        if (isValidCombination(combination))
        {
            result.push_back(combination);
        }
    }

    return result;
}