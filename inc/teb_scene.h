#ifndef TEB_SCENE_H
#define TEB_SCENE_H

#include <string>
#include <utility>

#include "occupancy_grid.h"
#include "pose_se2.h"

namespace teb_local_planner {

struct TebScene
{
    TebScene(OccupancyGridMap grid, const PoseSE2& start, const PoseSE2& goal)
        : grid(std::move(grid)), start(start), goal(goal) {}

    OccupancyGridMap grid;
    PoseSE2 start;
    PoseSE2 goal;
};

TebScene loadTebScene(const std::string& filename);
void saveTebScene(const std::string& filename, const OccupancyGridMap& grid,
                  const PoseSE2& start, const PoseSE2& goal);

} // namespace teb_local_planner

#endif // TEB_SCENE_H
