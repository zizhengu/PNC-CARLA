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
    int num_lat_action = (int)FpbLatAction::MAX_COUNT;
    for (int lon = 0; lon < num_lon_action; lon++)
    {
        // KKK, LLL, RRR
        for (int lat = 0; lat < num_lat_action; lat++)
        {
            behaviour_seq.clear();
            for (int height = 0; height < tree_height_; height++)
            {
                temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(lat), action_time_);
                behaviour_seq.push_back(temp_behaviour);
            }
            behaviour_tree_.push_back(behaviour_seq);
        }
        // KKL, KKR
        for (int lat = 1; lat < num_lat_action; lat++)
        {
            behaviour_seq.clear();
            for (int height = 0; height < tree_height_ - 1; height++)
            {
                temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
                behaviour_seq.push_back(temp_behaviour);
            }
            temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(lat), action_time_);
            behaviour_seq.push_back(temp_behaviour);
            behaviour_tree_.push_back(behaviour_seq);
        }
        // KLL, KRR
        for (int lat = 1; lat < num_lat_action; lat++)
        {
            behaviour_seq.clear();
            temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(0), action_time_);
            behaviour_seq.push_back(temp_behaviour);
            for (int height = 0; height < tree_height_ - 1; height++)
            {
                temp_behaviour = FpbAction(FpbLonAction(lon), FpbLatAction(lat), action_time_);
                behaviour_seq.push_back(temp_behaviour);
            }
            behaviour_tree_.push_back(behaviour_seq);
        }
    }
    assert(int(behaviour_tree_.size()) == num_lon_action * 7);

    for (size_t i = 0; i < behaviour_tree_.size(); ++i)
    {
        std::cout << "----------------------behaviour_tree_[i].size()------------------" << behaviour_tree_[i].size() << std::endl;
        assert(int(behaviour_tree_[i].size()) == tree_height_);
    }
    return true;
}