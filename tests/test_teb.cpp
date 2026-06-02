/**
 * Standalone TEB unit test: randomly generates grid maps with obstacles,
 * runs TEB planning, and saves results as PNG images.
 *
 * Build: cd build && cmake .. && make test_teb
 * Run:   ./test_teb
 * Output: tests/output/test_000.png ... tests/output/test_049.png
 */

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <string>
#include <algorithm>

#include <QApplication>
#include <QImage>
#include <QPainter>
#include <QColor>
#include <QFont>

#include "teb_config.h"
#include "pose_se2.h"
#include "robot_footprint_model.h"
#include "obstacles.h"
#include "occupancy_grid.h"
#include "optimal_planner.h"

using namespace teb_local_planner;

static const int IMG_SIZE = 1500;
static const int GRID_W = 400;
static const int GRID_H = 400;
static const double RES = 0.05;          // 5cm per cell
static const double ORIGIN_X = -10.0;
static const double ORIGIN_Y = -10.0;
static const double SCALE = 75.0;        // pixels per meter: 1500 / 20m
static const int NUM_TESTS = 50;
static const int PAIRS_PER_MAP = 5;
static const int MIN_OBS = 18;
static const int MAX_OBS = 45;
static const int MIN_RECT_SZ = 3;
static const int MAX_RECT_SZ = 40;
static const double MIN_CIRCLE_R = 0.3;   // meters
static const double MAX_CIRCLE_R = 1.5;   // meters

// Convert world coordinate to image pixel
static void worldToPixel(double wx, double wy, int& sx, int& sy)
{
    sx = static_cast<int>((wx - ORIGIN_X) * SCALE);
    sy = static_cast<int>((wy - ORIGIN_Y) * SCALE);
}

// Pick a random free cell in a given x-range (grid coordinates).
// Free means not occupied and not within robot radius of any obstacle.
// Returns true on success, false if no free cell found after max_attempts.
static bool pickFreeCell(OccupancyGridMap& grid, double robot_radius,
                         int x_min, int x_max,
                         int& out_gx, int& out_gy)
{
    const int max_attempts = 5000;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        int gx = x_min + rand() % (x_max - x_min);
        int gy = rand() % GRID_H;
        if (!grid.isOccupied(gx, gy) &&
            !grid.isOccupiedWithInflation(gx, gy, robot_radius)) {
            out_gx = gx;
            out_gy = gy;
            return true;
        }
    }
    return false;
}

static void drawArrow(QPainter& painter, int cx, int cy,
                      double theta_rad, const QColor& color)
{
    const int arrow_len = 20;
    const int head_len  = 8;
    int tip_x  = cx + static_cast<int>(std::cos(theta_rad) * arrow_len);
    int tip_y  = cy + static_cast<int>(std::sin(theta_rad) * arrow_len);
    int base_x = cx - static_cast<int>(std::cos(theta_rad) * arrow_len);
    int base_y = cy - static_cast<int>(std::sin(theta_rad) * arrow_len);

    painter.setPen(QPen(color, 4));
    painter.drawLine(base_x, base_y, tip_x, tip_y);

    double a1 = theta_rad + M_PI * 0.75;
    double a2 = theta_rad - M_PI * 0.75;
    QPoint head[3] = {
        QPoint(tip_x, tip_y),
        QPoint(tip_x + static_cast<int>(std::cos(a1) * head_len),
                tip_y + static_cast<int>(std::sin(a1) * head_len)),
        QPoint(tip_x + static_cast<int>(std::cos(a2) * head_len),
                tip_y + static_cast<int>(std::sin(a2) * head_len))
    };
    painter.setBrush(color);
    painter.drawPolygon(head, 3);
}

int main(int argc, char* argv[])
{
    // QApplication needed for QPainter::drawText (font handling)
    QApplication app(argc, argv);

    srand(static_cast<unsigned>(time(nullptr)));

    // Setup config, robot model, and planner (reused across iterations)
    TebConfig config;
    auto robot_model = boost::make_shared<CircularRobotFootprint>(0.2);
    TebVisualizationPtr visual = TebVisualizationPtr();
    ViaPointContainer via_points;

    ObstContainer obstacles;
    OccupancyGridMap grid(RES, GRID_W, GRID_H, ORIGIN_X, ORIGIN_Y);

    TebOptimalPlanner planner(config, &obstacles, robot_model,
                              visual, &via_points);
    planner.setGrid(&grid);

    // Statistics
    int succeeded = 0;
    std::vector<double> astar_costs;
    std::vector<double> teb_costs;

    const int total_tests = NUM_TESTS * PAIRS_PER_MAP;
    int test_idx = 0;

    printf("=== TEB Random Map Test: %d maps x %d pairs = %d tests ===\n\n",
           NUM_TESTS, PAIRS_PER_MAP, total_tests);

    for (int map = 0; map < NUM_TESTS; ++map) {

        // ---- 1. Clear grid and generate random obstacles ----
        grid.clear();

        int num_obs = MIN_OBS + rand() % (MAX_OBS - MIN_OBS + 1);
        for (int o = 0; o < num_obs; ++o) {
            int type = rand() % 3;  // 0=rect, 1=circle, 2=L-shape

            switch (type) {
            case 0: {
                // Rectangle
                int rx = 1 + rand() % (GRID_W - MAX_RECT_SZ - 2);
                int ry = 1 + rand() % (GRID_H - MAX_RECT_SZ - 2);
                int rw = MIN_RECT_SZ + rand() % (MAX_RECT_SZ - MIN_RECT_SZ + 1);
                int rh = MIN_RECT_SZ + rand() % (MAX_RECT_SZ - MIN_RECT_SZ + 1);
                if (rx + rw >= GRID_W) rw = GRID_W - rx - 1;
                if (ry + rh >= GRID_H) rh = GRID_H - ry - 1;
                for (int dy = 0; dy < rh; ++dy)
                    for (int dx = 0; dx < rw; ++dx)
                        grid.setCell(rx + dx, ry + dy,
                                     OccupancyGridMap::OCCUPIED);
                break;
            }
            case 1: {
                // Filled circle
                double cr = MIN_CIRCLE_R +
                    (MAX_CIRCLE_R - MIN_CIRCLE_R) * (rand() / (double)RAND_MAX);
                double cx = ORIGIN_X + RES * GRID_W * 0.15 +
                    (RES * GRID_W * 0.7) * (rand() / (double)RAND_MAX);
                double cy = ORIGIN_Y + RES * GRID_H * 0.15 +
                    (RES * GRID_H * 0.7) * (rand() / (double)RAND_MAX);
                grid.setOccupied(cx, cy, cr);
                break;
            }
            case 2: {
                // L-shape: two overlapping perpendicular rectangles
                int lx = 1 + rand() % (GRID_W - MAX_RECT_SZ - 2);
                int ly = 1 + rand() % (GRID_H - MAX_RECT_SZ - 2);
                int lw = MIN_RECT_SZ + 5 + rand() % (MAX_RECT_SZ / 2);
                int lh = MIN_RECT_SZ + 5 + rand() % (MAX_RECT_SZ / 2);
                for (int dy = 0; dy < lh / 3; ++dy)
                    for (int dx = 0; dx < lw; ++dx)
                        if (lx + dx < GRID_W - 1 && ly + dy < GRID_H - 1)
                            grid.setCell(lx + dx, ly + dy,
                                         OccupancyGridMap::OCCUPIED);
                for (int dy = 0; dy < lh; ++dy)
                    for (int dx = 0; dx < lw / 3; ++dx)
                        if (lx + dx < GRID_W - 1 && ly + dy < GRID_H - 1)
                            grid.setCell(lx + dx, ly + dy,
                                         OccupancyGridMap::OCCUPIED);
                break;
            }
            }
        }

        // ---- 2. Extract obstacles ----
        grid.extractObstacles(obstacles);

        // ---- 3. Try multiple start/goal pairs on this same map ----
        for (int pair = 0; pair < PAIRS_PER_MAP; ++pair) {
            printf("Test %d/%d (map %d, pair %d) ... ",
                   test_idx + 1, total_tests, map, pair);
            fflush(stdout);

            int sgx, sgy, ggx, ggy;
            bool start_ok = pickFreeCell(grid, 0.2, 0, GRID_W / 2, sgx, sgy);
            bool goal_ok  = pickFreeCell(grid, 0.2, GRID_W / 2, GRID_W, ggx, ggy);

            if (!start_ok || !goal_ok) {
                printf("SKIP (no free start/goal cell)\n");
                test_idx++;
                continue;
            }

            double swx, swy, gwx, gwy;
            grid.gridToWorld(sgx, sgy, swx, swy);
            grid.gridToWorld(ggx, ggy, gwx, gwy);

            PoseSE2 start(swx, swy, 0.0);
            PoseSE2 goal(gwx, gwy, 0.0);

            // ---- 4. Run planner ----
            planner.clearPlanner();
            bool ok = planner.plan(start, goal);

            double a_cost = planner.getAStarPathCost();
            double t_cost = planner.getCurrentCost();

            if (ok) {
                succeeded++;
                astar_costs.push_back(a_cost);
                teb_costs.push_back(t_cost);
                printf("OK  | A* cost=%.2f  TEB cost=%.2f\n",
                       a_cost, t_cost);
            } else {
                printf("FAIL | A* cost=%.2f  (no path found)\n", a_cost);
            }

            // ---- 5. Render to QImage ----
            QImage img(IMG_SIZE, IMG_SIZE, QImage::Format_RGB888);
            img.fill(Qt::gray);
            QPainter painter(&img);

            // --- occupied cells ---
            for (int iy = 0; iy < GRID_H; ++iy) {
                for (int ix = 0; ix < GRID_W; ++ix) {
                    if (grid.isOccupied(ix, iy)) {
                        double wx = ORIGIN_X + ix * RES;
                        double wy = ORIGIN_Y + iy * RES;
                        int sx, sy;
                        worldToPixel(wx, wy, sx, sy);
                        int cell_px = static_cast<int>(RES * SCALE);
                        painter.fillRect(sx, sy, cell_px, cell_px,
                                         QColor(140, 50, 20));
                    }
                }
            }

            // --- grid lines at 1m intervals ---
            painter.setPen(QPen(QColor(40, 40, 40), 1));
            for (int iy = 0; iy <= GRID_H; iy += 10) {
                double wy = ORIGIN_Y + iy * RES;
                int dummy, hy;
                worldToPixel(0, wy, dummy, hy);
                painter.drawLine(0, hy, IMG_SIZE, hy);
            }
            for (int ix = 0; ix <= GRID_W; ix += 10) {
                double wx = ORIGIN_X + ix * RES;
                int vx, dummy;
                worldToPixel(wx, 0, vx, dummy);
                painter.drawLine(vx, 0, vx, IMG_SIZE);
            }

            // --- polygon obstacle outlines (yellow) ---
            for (const auto& obs : obstacles) {
                auto poly = boost::dynamic_pointer_cast<PolygonObstacle>(obs);
                if (!poly || poly->noVertices() < 3)
                    continue;
                const auto& verts = poly->vertices();

                painter.setPen(QPen(QColor(255, 255, 0), 2));
                for (size_t i = 0; i + 1 < verts.size(); ++i) {
                    int x1, y1, x2, y2;
                    worldToPixel(verts[i].x(), verts[i].y(), x1, y1);
                    worldToPixel(verts[i + 1].x(), verts[i + 1].y(), x2, y2);
                    painter.drawLine(x1, y1, x2, y2);
                }
                {
                    int x1, y1, x2, y2;
                    worldToPixel(verts.back().x(), verts.back().y(), x1, y1);
                    worldToPixel(verts.front().x(), verts.front().y(), x2, y2);
                    painter.drawLine(x1, y1, x2, y2);
                }
                painter.setPen(Qt::NoPen);
                painter.setBrush(QColor(255, 100, 0));
                for (const auto& v : verts) {
                    int vx, vy;
                    worldToPixel(v.x(), v.y(), vx, vy);
                    painter.drawEllipse(QPoint(vx, vy), 4, 4);
                }
                painter.setBrush(Qt::NoBrush);
            }

            // --- start/goal arrows ---
            int s_px, s_py, g_px, g_py;
            worldToPixel(swx, swy, s_px, s_py);
            worldToPixel(gwx, gwy, g_px, g_py);
            drawArrow(painter, s_px, s_py, 0.0, Qt::green);
            drawArrow(painter, g_px, g_py, 0.0, Qt::blue);

            // --- straight start-to-goal line ---
            painter.setPen(QPen(QColor(255, 200, 50), 2));
            painter.drawLine(s_px, s_py, g_px, g_py);

            // --- A* initialization path (cyan) ---
            const auto& astar_path = planner.getAStarPath();
            if (!astar_path.empty()) {
                painter.setPen(QPen(QColor(0, 200, 255), 3));
                for (size_t i = 0; i + 1 < astar_path.size(); ++i) {
                    int x, y, nx, ny;
                    worldToPixel(astar_path[i][0], astar_path[i][1], x, y);
                    worldToPixel(astar_path[i + 1][0], astar_path[i + 1][1],
                                 nx, ny);
                    painter.drawLine(x, y, nx, ny);
                }
            }

            // --- optimized TEB trajectory (white) ---
            std::vector<Eigen::Vector3f> path;
            planner.getFullTrajectory(path);
            if (!path.empty()) {
                painter.setPen(QPen(Qt::white, 1));
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    int x, y, nx, ny;
                    worldToPixel(path[i][0], path[i][1], x, y);
                    worldToPixel(path[i + 1][0], path[i + 1][1], nx, ny);
                    painter.drawLine(x, y, nx, ny);
                }

                painter.setPen(Qt::NoPen);
                for (size_t i = 0; i < path.size(); ++i) {
                    int vx, vy;
                    worldToPixel(path[i][0], path[i][1], vx, vy);
                    if (i == 0)
                        painter.setBrush(Qt::green);
                    else if (i == path.size() - 1)
                        painter.setBrush(Qt::blue);
                    else
                        painter.setBrush(QColor(255, 0, 255));
                    painter.drawEllipse(QPoint(vx, vy), 4, 4);
                }
                painter.setBrush(Qt::NoBrush);
            }

            // --- text overlay ---
            {
                painter.setPen(Qt::white);
                QFont font("monospace", 12);
                font.setBold(true);
                painter.setFont(font);

                char buf[256];
                snprintf(buf, sizeof(buf),
                         "Map %d/%d pair %d  %s  A* cost=%.2f  TEB cost=%.2f",
                         map, NUM_TESTS, pair,
                         ok ? "OK" : "FAIL",
                         a_cost, t_cost);
                painter.drawText(10, 25, QString::fromUtf8(buf));
            }

            painter.end();

            // ---- 6. Save image ----
            char fname[256];
            snprintf(fname, sizeof(fname),
                     "../tests/output/test_%03d_%d.png", map, pair);
            img.save(QString::fromUtf8(fname));

            test_idx++;
        }
    }

    // ---- Summary ----
    printf("\n=== Summary ===\n");
    printf("Total: %d  Succeeded: %d  Failed: %d\n",
           total_tests, succeeded, total_tests - succeeded);

    if (!astar_costs.empty()) {
        auto [amin, amax] = std::minmax_element(
            astar_costs.begin(), astar_costs.end());
        double asum = 0;
        for (double v : astar_costs) asum += v;
        printf("A*  costs: min=%.2f  max=%.2f  avg=%.2f\n",
               *amin, *amax, asum / astar_costs.size());

        auto [tmin, tmax] = std::minmax_element(
            teb_costs.begin(), teb_costs.end());
        double tsum = 0;
        for (double v : teb_costs) tsum += v;
        printf("TEB costs: min=%.2f  max=%.2f  avg=%.2f\n",
               *tmin, *tmax, tsum / teb_costs.size());
    }

    printf("\nOutput images: tests/output/test_*.png\n");
    return 0;
}
