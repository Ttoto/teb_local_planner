#include "teb_scene.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace teb_local_planner {
namespace {

constexpr int SCENE_VERSION = 1;
constexpr long long MAX_GRID_CELLS = 10000000;

double finiteNumber(const nlohmann::json& value, const char* name)
{
    if (!value.is_number())
        throw std::runtime_error(std::string(name) + " must be a number");
    const double result = value.get<double>();
    if (!std::isfinite(result))
        throw std::runtime_error(std::string(name) + " must be finite");
    return result;
}

PoseSE2 readPose(const nlohmann::json& value, const char* name)
{
    if (!value.is_object() || !value.contains("x") ||
        !value.contains("y") || !value.contains("theta"))
        throw std::runtime_error(std::string(name) + " must contain x, y, and theta");

    const double x = finiteNumber(value.at("x"), "pose x");
    const double y = finiteNumber(value.at("y"), "pose y");
    const double theta = finiteNumber(value.at("theta"), "pose theta");
    if (theta < -M_PI || theta > M_PI)
        throw std::runtime_error(std::string(name) + " theta must be in [-pi, pi]");
    return PoseSE2(x, y, theta);
}

nlohmann::json writePose(const PoseSE2& pose)
{
    return {{"x", pose.x()}, {"y", pose.y()}, {"theta", pose.theta()}};
}

} // namespace

TebScene loadTebScene(const std::string& filename)
{
    std::ifstream input(filename);
    if (!input)
        throw std::runtime_error("could not open scene file");

    nlohmann::json document;
    try {
        input >> document;
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(std::string("invalid scene JSON: ") + e.what());
    }

    try {
        if (!document.is_object())
            throw std::runtime_error("scene root must be an object");
        if (!document.contains("version") || !document.at("version").is_number_integer())
            throw std::runtime_error("scene version is missing or invalid");
        if (document.at("version").get<int>() != SCENE_VERSION)
            throw std::runtime_error("unsupported scene version");
        if (!document.contains("grid") || !document.at("grid").is_object())
            throw std::runtime_error("scene grid is missing or invalid");

        const auto& grid_json = document.at("grid");
        for (const char* key : {"resolution", "width", "height", "origin_x", "origin_y"}) {
            if (!grid_json.contains(key))
                throw std::runtime_error(std::string("grid field is missing: ") + key);
        }
        if (!grid_json.at("width").is_number_integer() ||
            !grid_json.at("height").is_number_integer())
            throw std::runtime_error("grid width and height must be integers");

        const int width = grid_json.at("width").get<int>();
        const int height = grid_json.at("height").get<int>();
        const double resolution = finiteNumber(grid_json.at("resolution"), "grid resolution");
        const double origin_x = finiteNumber(grid_json.at("origin_x"), "grid origin_x");
        const double origin_y = finiteNumber(grid_json.at("origin_y"), "grid origin_y");
        if (width <= 0 || height <= 0 || resolution <= 0)
            throw std::runtime_error("grid dimensions and resolution must be positive");
        if (static_cast<long long>(width) * height > MAX_GRID_CELLS)
            throw std::runtime_error("grid is too large");

        OccupancyGridMap grid(resolution, width, height, origin_x, origin_y);
        if (!document.contains("occupied_cells") || !document.at("occupied_cells").is_array())
            throw std::runtime_error("occupied_cells is missing or invalid");
        if (document.at("occupied_cells").size() > static_cast<size_t>(width) * height)
            throw std::runtime_error("occupied_cells contains too many entries");
        for (const auto& cell : document.at("occupied_cells")) {
            if (!cell.is_array() || cell.size() != 2 ||
                !cell[0].is_number_integer() || !cell[1].is_number_integer())
                throw std::runtime_error("each occupied cell must be an [x, y] integer pair");
            const int x = cell[0].get<int>();
            const int y = cell[1].get<int>();
            if (!grid.isInBounds(x, y))
                throw std::runtime_error("occupied cell is outside the grid");
            grid.setCell(x, y, OccupancyGridMap::OCCUPIED);
        }

        if (!document.contains("start") || !document.contains("goal"))
            throw std::runtime_error("scene start or goal is missing");
        PoseSE2 start = readPose(document.at("start"), "start");
        PoseSE2 goal = readPose(document.at("goal"), "goal");
        return TebScene(std::move(grid), start, goal);
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(std::string("invalid scene data: ") + e.what());
    }
}

void saveTebScene(const std::string& filename, const OccupancyGridMap& grid,
                  const PoseSE2& start, const PoseSE2& goal)
{
    nlohmann::json occupied = nlohmann::json::array();
    for (int y = 0; y < grid.getHeight(); ++y) {
        for (int x = 0; x < grid.getWidth(); ++x) {
            if (grid.isOccupied(x, y))
                occupied.push_back({x, y});
        }
    }

    nlohmann::json document = {
        {"version", SCENE_VERSION},
        {"grid", {
            {"resolution", grid.getResolution()},
            {"width", grid.getWidth()},
            {"height", grid.getHeight()},
            {"origin_x", grid.getOriginX()},
            {"origin_y", grid.getOriginY()}
        }},
        {"occupied_cells", std::move(occupied)},
        {"start", writePose(start)},
        {"goal", writePose(goal)}
    };

    std::ofstream output(filename);
    if (!output)
        throw std::runtime_error("could not open scene file for writing");
    output << document.dump(2) << '\n';
    if (!output)
        throw std::runtime_error("failed while writing scene file");
}

} // namespace teb_local_planner
