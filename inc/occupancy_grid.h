#ifndef TEB_OCCUPANCY_GRID_H
#define TEB_OCCUPANCY_GRID_H

#include <Eigen/Core>
#include <vector>
#include <cstdint>
#include "obstacles.h"

namespace teb_local_planner {

class OccupancyGridMap {
public:
    OccupancyGridMap(double resolution, int width, int height,
                     double origin_x, double origin_y);

    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;

    uint8_t getCell(int gx, int gy) const;
    void setCell(int gx, int gy, uint8_t value);
    bool isOccupied(int gx, int gy) const;
    bool isInBounds(int gx, int gy) const;

    void setOccupied(double wx, double wy, double brush_radius);
    void setFree(double wx, double wy, double brush_radius);

    void extractObstacles(ObstContainer& obstacles) const;

    bool isOccupiedWithInflation(int gx, int gy, double robot_radius) const;
    std::vector<std::pair<int,int>> searchPathAStar(int start_gx, int start_gy,
        int goal_gx, int goal_gy, double robot_radius) const;

    void clear();

    int getWidth() const  { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    const std::vector<uint8_t>& data() const { return data_; }

    static constexpr uint8_t OCCUPIED = 100;
    static constexpr uint8_t FREE = 0;

private:
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    std::vector<uint8_t> data_;
};

} // namespace teb_local_planner

#endif // TEB_OCCUPANCY_GRID_H
