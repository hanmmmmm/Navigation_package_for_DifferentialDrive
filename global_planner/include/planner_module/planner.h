#ifndef PLANNER_H
#define PLANNER_H

#include "A_star_module/astar.h"
#include "Dijkstra_module/dijkstra.h"
#include "../global_planner/path_data.h"

class Planner_class //: public pathPoint
{
private:
    AstarClass astar_planner;
    DijkstracClass dijkstra_planner;

public:
    Planner_class();
    ~Planner_class();

    void setup(int plannertype, const int startnode[], const int goalnode[], const float goal_angle, const std::vector<int8_t> &map, const int height, const int width, const int timeout_ms);
    bool search(int plannertype);
    std::deque<pathPoint> get_path(int plannertype);
};

Planner_class::Planner_class() //:pathPoint()
{
}

Planner_class::~Planner_class()
{
}

void Planner_class::setup(int plannertype, const int startnode[], const int goalnode[], const float goal_angle, const std::vector<int8_t> &map, const int height, const int width, const int timeout_ms)
{
    if (plannertype == 0)
    {
        dijkstra_planner.setup(startnode, goalnode, goal_angle, map, height, width, timeout_ms);
    }
    else if (plannertype == 1)
    {
        astar_planner.setup(startnode, goalnode, goal_angle, map, height, width, timeout_ms);
    }
}

bool Planner_class::search(int plannertype)
{
    if (plannertype == 0)
    {
        return dijkstra_planner.search();
    }
    else if (plannertype == 1)
    {
        return astar_planner.search();
    }
}

std::deque<pathPoint> Planner_class::get_path(int plannertype)
{
    if (plannertype == 0)
    {
        return dijkstra_planner.get_path();
    }
    else if (plannertype == 1)
    {
        return astar_planner.get_path();
    }
}

#endif