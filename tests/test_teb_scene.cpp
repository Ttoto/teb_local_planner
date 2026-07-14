#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include "teb_scene.h"

using namespace teb_local_planner;

namespace {

void require(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

bool loadFails(const std::string& path)
{
    try {
        loadTebScene(path);
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
}

void writeFile(const std::string& path, const std::string& contents)
{
    std::ofstream output(path);
    output << contents;
}

} // namespace

int main()
{
    const std::string valid_path = "/tmp/teb_scene_round_trip.json";
    const std::string invalid_path = "/tmp/teb_scene_invalid.json";

    OccupancyGridMap grid(0.1, 12, 8, -1.0, -2.0);
    grid.setCell(0, 0, OccupancyGridMap::OCCUPIED);
    grid.setCell(5, 3, OccupancyGridMap::OCCUPIED);
    grid.setCell(11, 7, OccupancyGridMap::OCCUPIED);
    const PoseSE2 start(-0.5, -1.5, -1.23);
    const PoseSE2 goal(0.1, -1.3, 2.34);

    saveTebScene(valid_path, grid, start, goal);
    TebScene loaded = loadTebScene(valid_path);
    require(loaded.grid.getWidth() == 12 && loaded.grid.getHeight() == 8,
            "grid dimensions did not round-trip");
    require(std::abs(loaded.grid.getResolution() - 0.1) < 1e-12,
            "grid resolution did not round-trip");
    require(loaded.grid.isOccupied(0, 0) && loaded.grid.isOccupied(5, 3) &&
            loaded.grid.isOccupied(11, 7), "occupied cells did not round-trip");
    require(std::abs(loaded.start.theta() + 1.23) < 1e-12 &&
            std::abs(loaded.goal.theta() - 2.34) < 1e-12,
            "headings did not round-trip");

    writeFile(invalid_path, "{ not json");
    require(loadFails(invalid_path), "malformed JSON was accepted");
    writeFile(invalid_path,
              R"({"version":2,"grid":{},"occupied_cells":[],"start":{},"goal":{}})");
    require(loadFails(invalid_path), "unsupported version was accepted");
    writeFile(invalid_path,
              R"({"version":1,"grid":{"resolution":0.1,"width":2,"height":2,"origin_x":0,"origin_y":0},"occupied_cells":[[2,0]],"start":{"x":0,"y":0,"theta":0},"goal":{"x":1,"y":1,"theta":0}})");
    require(loadFails(invalid_path), "out-of-bounds occupied cell was accepted");
    writeFile(invalid_path,
              R"({"version":1,"grid":{"resolution":0,"width":2,"height":2,"origin_x":0,"origin_y":0},"occupied_cells":[],"start":{"x":0,"y":0,"theta":0},"goal":{"x":1,"y":1,"theta":0}})");
    require(loadFails(invalid_path), "invalid grid resolution was accepted");
    writeFile(invalid_path,
              R"({"version":1,"grid":{"resolution":0.1,"width":2,"height":2,"origin_x":0,"origin_y":0},"occupied_cells":[],"start":{"x":0,"y":0,"theta":0}})");
    require(loadFails(invalid_path), "missing goal was accepted");

    OccupancyGridMap dense_grid(1.0, 3, 2, 0.0, 0.0);
    for (int y = 0; y < dense_grid.getHeight(); ++y)
        for (int x = 0; x < dense_grid.getWidth(); ++x)
            dense_grid.setCell(x, y, OccupancyGridMap::OCCUPIED);
    saveTebScene(valid_path, dense_grid, PoseSE2(), PoseSE2());
    TebScene dense_loaded = loadTebScene(valid_path);
    for (int y = 0; y < dense_loaded.grid.getHeight(); ++y)
        for (int x = 0; x < dense_loaded.grid.getWidth(); ++x)
            require(dense_loaded.grid.isOccupied(x, y), "dense grid did not round-trip");

    OccupancyGridMap empty_grid(1.0, 2, 2, 0.0, 0.0);
    saveTebScene(valid_path, empty_grid, PoseSE2(), PoseSE2());
    TebScene empty_loaded = loadTebScene(valid_path);
    require(!empty_loaded.grid.isOccupied(0, 0), "empty grid did not round-trip");

    std::remove(valid_path.c_str());
    std::remove(invalid_path.c_str());
    return 0;
}
