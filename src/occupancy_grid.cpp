#include "occupancy_grid.h"
#include <boost/make_shared.hpp>
#include <cmath>
#include <queue>
#include <algorithm>

using namespace teb_local_planner;

OccupancyGridMap::OccupancyGridMap(double resolution, int width, int height,
                                   double origin_x, double origin_y)
    : resolution_(resolution), width_(width), height_(height),
      origin_x_(origin_x), origin_y_(origin_y),
      data_(width_ * height_, FREE)
{}

bool OccupancyGridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - origin_x_) / resolution_);
    gy = static_cast<int>((wy - origin_y_) / resolution_);
    return isInBounds(gx, gy);
}

void OccupancyGridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = origin_x_ + (gx + 0.5) * resolution_;
    wy = origin_y_ + (gy + 0.5) * resolution_;
}

uint8_t OccupancyGridMap::getCell(int gx, int gy) const {
    return isInBounds(gx, gy) ? data_[gy * width_ + gx] : FREE;
}

void OccupancyGridMap::setCell(int gx, int gy, uint8_t value) {
    if (isInBounds(gx, gy))
        data_[gy * width_ + gx] = value;
}

bool OccupancyGridMap::isOccupied(int gx, int gy) const {
    return isInBounds(gx, gy) && data_[gy * width_ + gx] >= 50;
}

bool OccupancyGridMap::isInBounds(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void OccupancyGridMap::clear() {
    std::fill(data_.begin(), data_.end(), FREE);
}

bool OccupancyGridMap::isOccupiedWithInflation(int gx, int gy, double robot_radius) const {
    int radius_cells = static_cast<int>(std::ceil(robot_radius / resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
            if (dist <= robot_radius && isOccupied(gx + dx, gy + dy))
                return true;
        }
    }
    return false;
}

std::vector<std::pair<int,int>> OccupancyGridMap::searchPathAStar(
    int start_gx, int start_gy, int goal_gx, int goal_gy, double robot_radius) const
{
    if (isOccupiedWithInflation(start_gx, start_gy, robot_radius) ||
        isOccupiedWithInflation(goal_gx, goal_gy, robot_radius))
        return {};

    const int N = width_ * height_;
    const double INF = 1e18;
    std::vector<double> g_cost(N, INF);
    std::vector<int> came_from(N, -1);
    std::vector<bool> closed(N, false);

    double goal_wx, goal_wy;
    gridToWorld(goal_gx, goal_gy, goal_wx, goal_wy);

    // 8-connected neighbors: cardinal + diagonal
    const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double cost8[8] = {1, 1, 1, 1, 1, 1, 1, 1}; // in cells, will multiply by resolution

    auto idx = [](int gx, int gy, int w) { return gy * w + gx; };

    using Node = std::pair<double, int>; // (f_cost, flat_index)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    int start_idx = idx(start_gx, start_gy, width_);
    g_cost[start_idx] = 0;
    open.push({0, start_idx});

    while (!open.empty()) {
        auto [f, cur] = open.top(); open.pop();
        if (closed[cur]) continue;
        closed[cur] = true;

        int cx = cur % width_, cy = cur / width_;
        if (cx == goal_gx && cy == goal_gy) break;

        for (int k = 0; k < 8; ++k) {
            int nx = cx + dx8[k], ny = cy + dy8[k];
            if (!isInBounds(nx, ny) || isOccupiedWithInflation(nx, ny, robot_radius))
                continue;

            int ni = idx(nx, ny, width_);
            if (closed[ni]) continue;

            double move_cost = (k < 3 || k > 4) ? resolution_ * std::sqrt(2.0) : resolution_;
            double new_g = g_cost[cur] + move_cost;
            if (new_g < g_cost[ni]) {
                g_cost[ni] = new_g;
                double nwx, nwy;
                gridToWorld(nx, ny, nwx, nwy);
                double h = std::sqrt((nwx - goal_wx)*(nwx - goal_wx) + (nwy - goal_wy)*(nwy - goal_wy));
                open.push({new_g + h, ni});
                came_from[ni] = cur;
            }
        }
    }

    int goal_idx = idx(goal_gx, goal_gy, width_);
    if (g_cost[goal_idx] >= INF) return {};

    std::vector<std::pair<int,int>> path;
    int cur = goal_idx;
    while (cur != -1) {
        path.push_back({cur % width_, cur / width_});
        cur = came_from[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void OccupancyGridMap::setOccupied(double wx, double wy, double brush_radius) {
    int cx, cy;
    if (!worldToGrid(wx, wy, cx, cy)) return;
    int radius_cells = static_cast<int>(std::ceil(brush_radius / resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            int gx = cx + dx, gy = cy + dy;
            if (!isInBounds(gx, gy)) continue;
            double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
            if (dist <= brush_radius)
                data_[gy * width_ + gx] = OCCUPIED;
        }
    }
}

void OccupancyGridMap::setFree(double wx, double wy, double brush_radius) {
    int cx, cy;
    if (!worldToGrid(wx, wy, cx, cy)) return;
    int radius_cells = static_cast<int>(std::ceil(brush_radius / resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            int gx = cx + dx, gy = cy + dy;
            if (!isInBounds(gx, gy)) continue;
            double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
            if (dist <= brush_radius)
                data_[gy * width_ + gx] = FREE;
        }
    }
}

void OccupancyGridMap::extractObstacles(ObstContainer& obstacles) const {
    obstacles.clear();

    std::vector<bool> visited(width_ * height_, false);
    std::vector<std::vector<std::pair<int,int>>> clusters;

    const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            if (!isOccupied(ix, iy) || visited[iy * width_ + ix])
                continue;

            std::vector<std::pair<int,int>> cluster;
            std::queue<std::pair<int,int>> q;
            q.push({ix, iy});
            visited[iy * width_ + ix] = true;

            while (!q.empty()) {
                auto [cx, cy] = q.front(); q.pop();
                cluster.push_back({cx, cy});

                for (int k = 0; k < 8; ++k) {
                    int nx = cx + dx8[k], ny = cy + dy8[k];
                    if (isInBounds(nx, ny) && isOccupied(nx, ny) &&
                        !visited[ny * width_ + nx]) {
                        visited[ny * width_ + nx] = true;
                        q.push({nx, ny});
                    }
                }
            }
            clusters.push_back(std::move(cluster));
        }
    }

    for (const auto& cluster : clusters) {
        if (cluster.size() == 1) {
            double wx, wy;
            gridToWorld(cluster[0].first, cluster[0].second, wx, wy);
            obstacles.push_back(boost::make_shared<PointObstacle>(wx, wy));
        } else {
            std::vector<Eigen::Vector2d> points;
            points.reserve(cluster.size());
            for (auto [ix, iy] : cluster) {
                double wx, wy;
                gridToWorld(ix, iy, wx, wy);
                points.push_back(Eigen::Vector2d(wx, wy));
            }

            // Andrew's monotone chain convex hull
            std::sort(points.begin(), points.end(),
                [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
                });

            auto cross = [](const Eigen::Vector2d& o,
                            const Eigen::Vector2d& a,
                            const Eigen::Vector2d& b) -> double {
                return (a.x() - o.x()) * (b.y() - o.y()) -
                       (a.y() - o.y()) * (b.x() - o.x());
            };

            // Lower hull
            std::vector<Eigen::Vector2d> hull;
            for (const auto& p : points) {
                while (hull.size() >= 2 &&
                       cross(hull[hull.size()-2], hull.back(), p) <= 0)
                    hull.pop_back();
                hull.push_back(p);
            }

            // Upper hull
            size_t lower_count = hull.size();
            for (int i = static_cast<int>(points.size()) - 2; i >= 0; --i) {
                while (hull.size() > lower_count &&
                       cross(hull[hull.size()-2], hull.back(), points[i]) <= 0)
                    hull.pop_back();
                hull.push_back(points[i]);
            }
            hull.pop_back(); // remove duplicate last point

            if (hull.size() >= 3) {
                Point2dContainer vertices;
                vertices.reserve(hull.size());
                for (const auto& v : hull)
                    vertices.push_back(v);
                obstacles.push_back(
                    boost::make_shared<PolygonObstacle>(vertices));
            } else {
                // Degenerate: all points collinear, emit individual points
                for (auto [ix, iy] : cluster) {
                    double wx, wy;
                    gridToWorld(ix, iy, wx, wy);
                    obstacles.push_back(
                        boost::make_shared<PointObstacle>(wx, wy));
                }
            }
        }
    }
}
