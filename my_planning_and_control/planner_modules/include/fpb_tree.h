#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cassert>
#include <iostream>

class FpbTree
{
public:
    enum class FpbLonAction
    {
        Maintain = 0,
        Accelearte = 1,
        Decelerate = 2,
        MAX_COUNT = 3
    };

    enum class FpbLatAction
    {
        Keeping = 0,
        SwitchLeft = 1,
        SwitchRight = 2,
        MAX_COUNT = 3
    };

    struct FpbAction
    {
        FpbLonAction lon = FpbLonAction::Maintain;
        FpbLatAction lat = FpbLatAction::Keeping;
        // action duration time
        double t = 0.0;
        friend std::ostream &operator<<(std::ostream &os, const FpbAction &action)
        {
            os << "(lon: " << static_cast<int>(action.lon)
               << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
               << ")";
            return os;
        }

        FpbAction() {}
        FpbAction(const FpbLonAction &lon_, const FpbLatAction &lat_,
                  const double &t_)
            : lon(lon_), lat(lat_), t(t_) {}
    };

    FpbTree(const int &tree_height, const double &action_time);
    ~FpbTree() = default;

    void set_current_action(const FpbAction &action)
    {
        current_action_ = action;
    }

    std::vector<std::vector<FpbAction>> get_behaviour_tree()
    {
        return behaviour_tree_;
    }

    static std::string GetLonActionName(const FpbLonAction &action)
    {
        std::string action_string;
        switch (action)
        {
        case FpbLonAction::Maintain:
        {
            action_string = std::string("M");
            break;
        }
        case FpbLonAction::Accelearte:
        {
            action_string = std::string("A");
            break;
        }
        case FpbLonAction::Decelerate:
        {
            action_string = std::string("D");
            break;
        }
        default:
        {
            action_string = std::string("WrongInput");
            break;
        }
        }
        return action_string;
    };

    static std::string GetLatActionName(const FpbLatAction &action)
    {
        std::string action_string;
        switch (action)
        {
        case FpbLatAction::Keeping:
        {
            action_string = std::string("K");
            break;
        }
        case FpbLatAction::SwitchLeft:
        {
            action_string = std::string("L");
            break;
        }
        case FpbLatAction::SwitchRight:
        {
            action_string = std::string("R");
            break;
        }
        default:
        {
            action_string = std::string("WrongInput");
            break;
        }
        }
        return action_string;
    };

private:
    bool GenarateFpbTree();

    int tree_height_ = 3;
    double action_time_ = 1.0;
    FpbAction current_action_;
    std::vector<std::vector<FpbAction>> behaviour_tree_;
};