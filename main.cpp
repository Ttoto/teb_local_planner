#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QPainter>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QDialog>
#include <QPlainTextEdit>
#include <QImage>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QButtonGroup>
#include <QFutureWatcher>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QtConcurrent/QtConcurrent>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <vector>

#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/occupancy_grid.h"
#include "inc/optimal_planner.h"
#include "inc/teb_scene.h"
#include <boost/smart_ptr.hpp>

using namespace teb_local_planner;

enum ToolMode { TOOL_DRAW, TOOL_ERASE };

static const double BRUSH_SIZES[] = { 0.1, 0.2, 0.5, 1.0, 2.0 };
static const int BRUSH_SIZE_COUNT = 5;

class TebDisplayWidget : public QWidget
{
    Q_OBJECT
private:
    struct PlanResult
    {
        bool success = false;
        int generation = 0;
        std::vector<Eigen::Vector2d> astar_path;
        std::vector<Eigen::Vector3f> trajectory;
        double astar_path_cost = 0.0;
        double astar_teb_cost = std::numeric_limits<double>::infinity();
        double teb_cost = std::numeric_limits<double>::infinity();
        QString error;
    };

public:
    TebDisplayWidget(QWidget* parent = nullptr)
        : QWidget(parent)
        , _start(-2, 0, 0)
        , _end(2, 0, 0)
        , _image(1500, 1500, QImage::Format_RGB888)
        , _grid(0.05, 400, 400, -10.0, -10.0)
        , _zoom(0.375)
        , _center_x(0)
        , _center_y(0)
    {
        _image.fill(Qt::gray);

        _grid.extractObstacles(_obstacles);
        _robot_model = boost::make_shared<CircularRobotFootprint>(0.2);

        _configFile = "teb_config.json";
        std::ifstream ifs(_configFile);
        if (ifs.good()) {
            ifs.close();
            _config.loadFromFile(_configFile);
        } else {
            _config.saveToFile(_configFile);
        }

        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, this, &TebDisplayWidget::updateDisplay);
        _timer->start(30);

        _plan_watcher = new QFutureWatcher<PlanResult>(this);
        connect(_plan_watcher, &QFutureWatcher<PlanResult>::finished,
                this, &TebDisplayWidget::planningFinished);
    }

    ~TebDisplayWidget() = default;

public slots:
    void setZoom(int percent)
    {
        _zoom = percent / 100.0;
        requestRender();
    }

private:
    double getScale() const { return 200.0 * _zoom; }
    void worldToPixel(double wx, double wy, int& sx, int& sy) const {
        double s = getScale();
        sx = static_cast<int>(wx * s + 750.0 + _center_x);
        sy = static_cast<int>(wy * s + 750.0 + _center_y);
    }
    void pixelToWorld(int sx, int sy, double& wx, double& wy) const {
        double s = getScale();
        wx = (sx - 750.0 - _center_x) / s;
        wy = (sy - 750.0 - _center_y) / s;
    }

public slots:
    void setStartTheta(int value)
    {
        _start_theta = value;
        invalidatePlan();
        requestRender();
    }

    void setEndTheta(int value)
    {
        _end_theta = value;
        invalidatePlan();
        requestRender();
    }

    void setStartX(double v) { _start_x = v; invalidatePlan(); requestRender(); }
    void setStartY(double v) { _start_y = v; invalidatePlan(); requestRender(); }
    void setEndX(double v)   { _end_x = v; invalidatePlan(); requestRender(); }
    void setEndY(double v)   { _end_y = v; invalidatePlan(); requestRender(); }

    void setBrushRadius(double r) { _brush_radius = r; update(); }
    void setToolMode(int mode) { _tool_mode = static_cast<ToolMode>(mode); }
    void setBrushSizeIdx(int idx) { _brush_radius = BRUSH_SIZES[idx]; update(); }

    void setCostLabels(QLabel* astarLabel, QLabel* astarTebLabel, QLabel* tebLabel)
    {
        _astarCostLabel = astarLabel;
        _astarTebCostLabel = astarTebLabel;
        _tebCostLabel = tebLabel;
    }

    void setPlanningControls(QPushButton* planButton, QLabel* statusLabel)
    {
        _planButton = planButton;
        _statusLabel = statusLabel;
    }

    void setPoseControls(QDoubleSpinBox* startX, QDoubleSpinBox* startY,
                         QSlider* startTheta, QLabel* startThetaLabel,
                         QDoubleSpinBox* endX, QDoubleSpinBox* endY,
                         QSlider* endTheta, QLabel* endThetaLabel)
    {
        _startXControl = startX;
        _startYControl = startY;
        _startThetaControl = startTheta;
        _startThetaLabel = startThetaLabel;
        _endXControl = endX;
        _endYControl = endY;
        _endThetaControl = endTheta;
        _endThetaLabel = endThetaLabel;
    }

    void toggleAnimation(bool on)
    {
        _animating = on && _has_plan && !_last_plan.trajectory.empty();
        if (!on)
            _anim_frame = 0;
        requestRender();
    }

    void setAnimSpeed(double speed)
    {
        _anim_speed = speed;
    }

    void clearGrid()
    {
        _grid.clear();
        _obstacles_dirty = true;
        invalidatePlan(true);
        requestRender();
    }

    void saveScene()
    {
        QString initial = _sceneFile.isEmpty() ? "scene.teb_scene.json" : _sceneFile;
        QString filename = QFileDialog::getSaveFileName(
            this, "Save TEB Scene", initial, "TEB Scene (*.teb_scene.json);;JSON Files (*.json)");
        if (filename.isEmpty())
            return;
        if (!filename.endsWith(".json", Qt::CaseInsensitive))
            filename += ".teb_scene.json";

        try {
            PoseSE2 start(_start_x, _start_y, _start_theta * 0.01);
            PoseSE2 goal(_end_x, _end_y, _end_theta * 0.01);
            saveTebScene(filename.toStdString(), _grid, start, goal);
            _sceneFile = filename;
            setStatus(QString("Scene saved: %1").arg(QFileInfo(filename).fileName()));
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Save Scene Failed", e.what());
            setStatus("Scene save failed");
        }
    }

    void openScene()
    {
        QString initial = _sceneFile.isEmpty() ? QString() : _sceneFile;
        QString filename = QFileDialog::getOpenFileName(
            this, "Open TEB Scene", initial, "TEB Scene (*.teb_scene.json);;JSON Files (*.json);;All Files (*)");
        if (filename.isEmpty())
            return;

        try {
            TebScene scene = loadTebScene(filename.toStdString());

            _grid = std::move(scene.grid);
            _start_x = scene.start.x();
            _start_y = scene.start.y();
            _start_theta = static_cast<int>(std::lround(scene.start.theta() * 100.0));
            _end_x = scene.goal.x();
            _end_y = scene.goal.y();
            _end_theta = static_cast<int>(std::lround(scene.goal.theta() * 100.0));
            _start = PoseSE2(_start_x, _start_y, _start_theta * 0.01);
            _end = PoseSE2(_end_x, _end_y, _end_theta * 0.01);
            syncPoseControls();

            _sceneFile = filename;
            _obstacles_dirty = true;
            refreshObstacles();
            invalidatePlan(true);
            requestRender();

            if (_planning)
                _plan_after_current = true;
            else
                runPlanner();
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Open Scene Failed", e.what());
            setStatus("Scene open failed");
        }
    }

    void editConfig()
    {
        auto* dialog = new QDialog(this);
        dialog->setWindowTitle("Edit Config");
        dialog->resize(600, 500);

        auto* edit = new QPlainTextEdit(dialog);
        std::ifstream ifs(_configFile);
        std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        edit->setPlainText(QString::fromStdString(content));

        auto* saveBtn = new QPushButton("Save", dialog);
        auto* cancelBtn = new QPushButton("Cancel", dialog);

        connect(saveBtn, &QPushButton::clicked, [dialog, edit, this]() {
            std::ofstream ofs(_configFile);
            ofs << edit->toPlainText().toStdString();
            ofs.close();
            _config.loadFromFile(_configFile);
            invalidatePlan(true);
            setStatus("Config saved");
            requestRender();
            dialog->accept();
        });
        connect(cancelBtn, &QPushButton::clicked, dialog, &QDialog::reject);

        auto* btnLayout = new QHBoxLayout;
        btnLayout->addStretch();
        btnLayout->addWidget(saveBtn);
        btnLayout->addWidget(cancelBtn);

        auto* layout = new QVBoxLayout(dialog);
        layout->addWidget(edit);
        layout->addLayout(btnLayout);

        dialog->exec();
        delete dialog;
    }

    void updateDisplay()
    {
        if (!_scene_dirty && !_animating)
            return;

        // Advance animation frame
        if (_animating) {
            _anim_frame += static_cast<int>(_anim_speed);
        }

        if (_obstacles_dirty && !_mouse_left_down && !_mouse_right_down)
            refreshObstacles();

        _image.fill(Qt::gray);
        QPainter painter(&_image);

        double s = getScale();
        int gw = _grid.getWidth(), gh = _grid.getHeight();
        double res = _grid.getResolution();
        double ox = _grid.getOriginX(), oy = _grid.getOriginY();

        // Draw grid: occupied cells (skip cells outside viewport for performance)
        int cell_px = static_cast<int>(std::ceil(res * s));
        double wx_min = (0 - 750.0 - _center_x) / s;
        double wx_max = (1500 - 750.0 - _center_x) / s;
        double wy_min = (0 - 750.0 - _center_y) / s;
        double wy_max = (1500 - 750.0 - _center_y) / s;
        int min_ix = std::max(0, static_cast<int>(std::floor((wx_min - ox) / res)));
        int max_ix = std::min(gw - 1, static_cast<int>(std::ceil((wx_max - ox) / res)));
        int min_iy = std::max(0, static_cast<int>(std::floor((wy_min - oy) / res)));
        int max_iy = std::min(gh - 1, static_cast<int>(std::ceil((wy_max - oy) / res)));
        for (int iy = min_iy; iy <= max_iy; ++iy) {
            for (int ix = min_ix; ix <= max_ix; ++ix) {
                if (_grid.isOccupied(ix, iy)) {
                    int sx, sy;
                    worldToPixel(ox + ix * res, oy + iy * res, sx, sy);
                    painter.fillRect(sx, sy, cell_px, cell_px, QColor(140, 50, 20));
                }
            }
        }

        // Draw grid lines (1m spacing = every 20 cells at 0.05m res)
        int line_step = 10;
        painter.setPen(QPen(QColor(40, 40, 40), 1));
        for (int iy = 0; iy <= gh; iy += line_step) {
            double wy = oy + iy * res;
            int hx, hy; worldToPixel(0, wy, hx, hy);
            painter.drawLine(0, hy, 1500, hy);
        }
        for (int ix = 0; ix <= gw; ix += line_step) {
            double wx = ox + ix * res;
            int vx, vy; worldToPixel(wx, 0, vx, vy);
            painter.drawLine(vx, 0, vx, 1500);
        }

        // Draw polygon obstacle convex hulls
        for (const auto& obs : _obstacles) {
            auto poly = boost::dynamic_pointer_cast<PolygonObstacle>(obs);
            if (!poly || poly->noVertices() < 3)
                continue;
            const auto& verts = poly->vertices();
            // Draw edges in yellow
            painter.setPen(QPen(QColor(255, 255, 0), 2));
            for (int i = 0; i < (int)verts.size() - 1; ++i) {
                int x1, y1, x2, y2;
                worldToPixel(verts[i].x(), verts[i].y(), x1, y1);
                worldToPixel(verts[i+1].x(), verts[i+1].y(), x2, y2);
                painter.drawLine(x1, y1, x2, y2);
            }
            // Close the polygon
            {
                int x1, y1, x2, y2;
                worldToPixel(verts.back().x(), verts.back().y(), x1, y1);
                worldToPixel(verts.front().x(), verts.front().y(), x2, y2);
                painter.drawLine(x1, y1, x2, y2);
            }
            // Draw vertices as orange dots
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(255, 100, 0));
            for (const auto& v : verts) {
                int vx, vy;
                worldToPixel(v.x(), v.y(), vx, vy);
                painter.drawEllipse(QPoint(vx, vy), 4, 4);
            }
            painter.setBrush(Qt::NoBrush);
        }

        auto drawArrow = [&](int cx, int cy, double theta_rad, const QColor& color) {
            const int arrow_len = 20;
            const int head_len = 8;
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
        };

        int sx, sy, gx, gy;
        worldToPixel(_start_x, _start_y, sx, sy);
        drawArrow(sx, sy, _start_theta * 0.01, Qt::green);

        worldToPixel(_end_x, _end_y, gx, gy);
        drawArrow(gx, gy, _end_theta * 0.01, Qt::blue);

        _start.x() = _start_x;
        _start.y() = _start_y;
        _start.theta() = _start_theta * 0.01;
        _end.x() = _end_x;
        _end.y() = _end_y;
        _end.theta() = _end_theta * 0.01;

        std::vector<Eigen::Vector3f> path;
        try
        {
            // Draw original planned line (straight from start to goal)
            painter.setPen(QPen(QColor(255, 200, 50), 2));
            int psx, psy, pgx, pgy;
            worldToPixel(_start_x, _start_y, psx, psy);
            worldToPixel(_end_x, _end_y, pgx, pgy);
            painter.drawLine(psx, psy, pgx, pgy);

            // Draw A* initialization path (cyan)
            const auto& astar_path = _last_plan.astar_path;
            if (_has_plan && !astar_path.empty())
            {
                painter.setPen(QPen(QColor(0, 200, 255), 3));
                for (size_t i = 0; i + 1 < astar_path.size(); ++i)
                {
                    int x, y, nx, ny;
                    worldToPixel(astar_path[i][0], astar_path[i][1], x, y);
                    worldToPixel(astar_path[i+1][0], astar_path[i+1][1], nx, ny);
                    painter.drawLine(x, y, nx, ny);
                }
            }

            if (_has_plan)
                path = _last_plan.trajectory;

            // Draw optimized trajectory
            painter.setPen(QPen(Qt::white, 1));
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                int x, y, nx, ny;
                worldToPixel(path[i][0], path[i][1], x, y);
                worldToPixel(path[i + 1][0], path[i + 1][1], nx, ny);
                painter.drawLine(x, y, nx, ny);
            }

            // Draw g2o vertex positions as dots
            painter.setPen(Qt::NoPen);
            for (size_t i = 0; i < path.size(); ++i)
            {
                int vx, vy;
                worldToPixel(path[i][0], path[i][1], vx, vy);
                if (i == 0)
                    painter.setBrush(Qt::green);   // start vertex (fixed)
                else if (i == path.size() - 1)
                    painter.setBrush(Qt::blue);    // goal vertex (fixed)
                else
                    painter.setBrush(QColor(255, 0, 255)); // free vertex (magenta)
                painter.drawEllipse(QPoint(vx, vy), 4, 4);
            }
            painter.setBrush(Qt::NoBrush);

            // Draw animated robot body (only when animating)
            if (_animating && !path.empty())
            {
                // Wrap around when animation reaches end
                if (_anim_frame >= (int)path.size())
                    _anim_frame = 0;

                double rx = path[_anim_frame][0];
                double ry = path[_anim_frame][1];
                double rtheta = path[_anim_frame][2];
                int rpx, rpy;
                worldToPixel(rx, ry, rpx, rpy);

                double robot_radius = 0.2; // matches CircularRobotFootprint
                int r_radius_px = static_cast<int>(robot_radius * getScale());

                // Semi-transparent cyan filled circle
                QColor body_color(0, 220, 220, 160);
                painter.setPen(QPen(QColor(0, 200, 200), 2));
                painter.setBrush(QBrush(body_color));
                painter.drawEllipse(QPoint(rpx, rpy), r_radius_px, r_radius_px);

                // Direction arrow inside the circle
                int arrow_len = r_radius_px;
                int head_len = r_radius_px / 2;
                int tip_x = rpx + static_cast<int>(std::cos(rtheta) * arrow_len);
                int tip_y = rpy + static_cast<int>(std::sin(rtheta) * arrow_len);
                int base_x = rpx - static_cast<int>(std::cos(rtheta) * arrow_len * 0.4);
                int base_y = rpy - static_cast<int>(std::sin(rtheta) * arrow_len * 0.4);

                painter.setPen(QPen(Qt::white, 2));
                painter.setBrush(Qt::NoBrush);
                painter.drawLine(base_x, base_y, tip_x, tip_y);

                double a1 = rtheta + M_PI * 0.75;
                double a2 = rtheta - M_PI * 0.75;
                QPoint head[3] = {
                    QPoint(tip_x, tip_y),
                    QPoint(tip_x + static_cast<int>(std::cos(a1) * head_len),
                            tip_y + static_cast<int>(std::sin(a1) * head_len)),
                    QPoint(tip_x + static_cast<int>(std::cos(a2) * head_len),
                            tip_y + static_cast<int>(std::sin(a2) * head_len))
                };
                painter.setBrush(Qt::white);
                painter.drawPolygon(head, 3);
            }
        }
        catch (...) {}

        painter.end();
        _scene_dirty = false;
        update();
    }

    void runPlanner()
    {
        if (_planning)
            return;

        _animating = false;
        _anim_frame = 0;
        _start.x() = _start_x;
        _start.y() = _start_y;
        _start.theta() = _start_theta * 0.01;
        _end.x() = _end_x;
        _end.y() = _end_y;
        _end.theta() = _end_theta * 0.01;

        const int generation = ++_plan_generation;
        TebConfig config = _config;
        OccupancyGridMap grid = _grid;
        PoseSE2 start = _start;
        PoseSE2 goal = _end;

        _planning = true;
        if (_planButton)
            _planButton->setEnabled(false);
        setStatus("Planning...");

        if (_astarCostLabel)
            _astarCostLabel->setText("A* path len: --");
        if (_astarTebCostLabel)
            _astarTebCostLabel->setText("A* TEB cost: --");
        if (_tebCostLabel)
            _tebCostLabel->setText("TEB cost: --");

        auto future = QtConcurrent::run([generation, config, grid, start, goal]() mutable {
            return computePlan(generation, config, grid, start, goal);
        });
        _plan_watcher->setFuture(future);
        requestRender();
    }

    void planningFinished()
    {
        _planning = false;
        if (_planButton)
            _planButton->setEnabled(true);

        PlanResult result = _plan_watcher->result();
        if (result.generation != _plan_generation) {
            setStatus("Plan discarded; inputs changed");
            if (_plan_after_current) {
                _plan_after_current = false;
                QTimer::singleShot(0, this, &TebDisplayWidget::runPlanner);
            }
            return;
        }

        _last_plan = result;
        _has_plan = result.success && result.error.isEmpty();
        if (!_has_plan) {
            setStatus(result.error.isEmpty() ? "Planning failed" : result.error);
        } else {
            setStatus("Plan ready");
        }

        if (_astarCostLabel)
            _astarCostLabel->setText(QString("A* path len: %1 m").arg(result.astar_path_cost, 0, 'f', 2));
        if (_astarTebCostLabel)
            _astarTebCostLabel->setText(QString("A* TEB cost: %1").arg(result.astar_teb_cost, 0, 'f', 2));
        if (_tebCostLabel)
            _tebCostLabel->setText(QString("TEB cost: %1").arg(result.teb_cost, 0, 'f', 2));

        requestRender();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.drawImage(0, 0, _image);
        // Brush preview circle
        if (_mouse_inside) {
            double s = getScale();
            int r_px = static_cast<int>(_brush_radius * s);
            QColor preview = (_tool_mode == TOOL_DRAW) ? QColor(140, 50, 20, 100) : QColor(200, 200, 200, 100);
            painter.setPen(QPen(preview, 1));
            painter.setBrush(QBrush(preview));
            painter.drawEllipse(_mouse_x - r_px, _mouse_y - r_px, 2 * r_px, 2 * r_px);
        }
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton)  _mouse_left_down = true;
        if (event->button() == Qt::RightButton) _mouse_right_down = true;
        _last_mouse_x = event->pos().x();
        _last_mouse_y = event->pos().y();
        applyBrush(_last_mouse_x, _last_mouse_y);
        requestRender();
    }

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton)  _mouse_left_down = false;
        if (event->button() == Qt::RightButton) _mouse_right_down = false;
        requestRender();
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        _mouse_x = event->pos().x();
        _mouse_y = event->pos().y();
        _mouse_inside = true;
        if (!_mouse_left_down && !_mouse_right_down) { update(); return; }
        int x0 = _last_mouse_x, y0 = _last_mouse_y;
        int x1 = event->pos().x(), y1 = event->pos().y();
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int steps = std::max(dx, dy);
        for (int i = 0; i <= steps; ++i) {
            double t = (steps == 0) ? 0.0 : static_cast<double>(i) / steps;
            int xi = x0 + static_cast<int>((x1 - x0) * t);
            int yi = y0 + static_cast<int>((y1 - y0) * t);
            applyBrush(xi, yi);
        }
        _last_mouse_x = x1;
        _last_mouse_y = y1;
        requestRender();
    }

    void applyBrush(int sx, int sy)
    {
        double wx, wy;
        pixelToWorld(sx, sy, wx, wy);
        if (_mouse_right_down)
            _grid.setFree(wx, wy, _brush_radius);
        else if (_mouse_left_down) {
            if (_tool_mode == TOOL_DRAW)
                _grid.setOccupied(wx, wy, _brush_radius);
            else
                _grid.setFree(wx, wy, _brush_radius);
        }
        _obstacles_dirty = true;
        invalidatePlan();
    }

    void enterEvent(QEvent*) override { _mouse_inside = true; update(); }
    void leaveEvent(QEvent*) override { _mouse_inside = false; update(); }

    void wheelEvent(QWheelEvent* event) override
    {
        int sx = static_cast<int>(event->position().x()), sy = static_cast<int>(event->position().y());
        double wx, wy;
        pixelToWorld(sx, sy, wx, wy);

        double old_zoom = _zoom;
        double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
        _zoom = std::max(0.05, std::min(2.0, _zoom * factor));

        // Keep the world point under cursor fixed
        _center_x = sx - static_cast<int>(wx * getScale() + 750.0);
        _center_y = sy - static_cast<int>(wy * getScale() + 750.0);

        requestRender();
    }

private:
    static PlanResult computePlan(int generation, TebConfig config, OccupancyGridMap grid,
                                  PoseSE2 start, PoseSE2 goal)
    {
        PlanResult result;
        result.generation = generation;

        try {
            ObstContainer obstacles;
            ViaPointContainer via_points;
            grid.extractObstacles(obstacles);

            auto robot_model = boost::make_shared<CircularRobotFootprint>(0.2);
            TebOptimalPlanner planner(config, &obstacles, robot_model,
                                      TebVisualizationPtr(), &via_points);
            planner.setGrid(&grid);
            planner.clearPlanner();
            result.success = planner.plan(start, goal);
            result.astar_path = planner.getAStarPath();
            result.astar_path_cost = planner.getAStarPathCost();
            result.astar_teb_cost = planner.getAStarTEBCost();
            result.teb_cost = planner.getCurrentCost();
            planner.getFullTrajectory(result.trajectory);

            if (!result.success)
                result.error = "Planning failed";
        } catch (const std::exception& e) {
            result.success = false;
            result.error = QString("Planning error: %1").arg(e.what());
        } catch (...) {
            result.success = false;
            result.error = "Planning error";
        }

        return result;
    }

    void refreshObstacles()
    {
        _grid.extractObstacles(_obstacles);
        _obstacles_dirty = false;
    }

    void syncPoseControls()
    {
        if (!_startXControl || !_startYControl || !_startThetaControl ||
            !_startThetaLabel || !_endXControl || !_endYControl ||
            !_endThetaControl || !_endThetaLabel)
            return;

        const QSignalBlocker blockStartX(_startXControl);
        const QSignalBlocker blockStartY(_startYControl);
        const QSignalBlocker blockStartTheta(_startThetaControl);
        const QSignalBlocker blockEndX(_endXControl);
        const QSignalBlocker blockEndY(_endYControl);
        const QSignalBlocker blockEndTheta(_endThetaControl);

        const double min_x = std::min({_grid.getOriginX(), _start_x, _end_x});
        const double max_x = std::max({_grid.getOriginX() + _grid.getWidth() * _grid.getResolution(),
                                       _start_x, _end_x});
        const double min_y = std::min({_grid.getOriginY(), _start_y, _end_y});
        const double max_y = std::max({_grid.getOriginY() + _grid.getHeight() * _grid.getResolution(),
                                       _start_y, _end_y});
        _startXControl->setRange(min_x, max_x);
        _endXControl->setRange(min_x, max_x);
        _startYControl->setRange(min_y, max_y);
        _endYControl->setRange(min_y, max_y);
        _startXControl->setValue(_start_x);
        _startYControl->setValue(_start_y);
        _startThetaControl->setValue(_start_theta);
        _endXControl->setValue(_end_x);
        _endYControl->setValue(_end_y);
        _endThetaControl->setValue(_end_theta);
        _startThetaLabel->setText(QString("theta: %1").arg(_start_theta * 0.01, 0, 'f', 2));
        _endThetaLabel->setText(QString("theta: %1").arg(_end_theta * 0.01, 0, 'f', 2));
    }

    void requestRender()
    {
        _scene_dirty = true;
        updateDisplay();
    }

    void invalidatePlan(bool resetLabels = false)
    {
        ++_plan_generation;
        _has_plan = false;
        _last_plan = PlanResult();
        _animating = false;
        _anim_frame = 0;
        if (resetLabels) {
            if (_astarCostLabel)
                _astarCostLabel->setText("A* path len: --");
            if (_astarTebCostLabel)
                _astarTebCostLabel->setText("A* TEB cost: --");
            if (_tebCostLabel)
                _tebCostLabel->setText("TEB cost: --");
            setStatus("Ready");
        }
    }

    void setStatus(const QString& text)
    {
        if (_statusLabel)
            _statusLabel->setText(QString("Status: %1").arg(text));
    }

    TebConfig _config;
    PoseSE2 _start;
    PoseSE2 _end;
    int _start_theta = 0;
    int _end_theta = 0;
    double _start_x = -2.0, _start_y = 0.0;
    double _end_x = 2.0, _end_y = 0.0;
    QImage _image;
    QTimer* _timer;
    std::string _configFile;
    OccupancyGridMap _grid;
    double _zoom = 0.375;
    int _center_x = 0, _center_y = 0;
    double _brush_radius = 0.5;
    ToolMode _tool_mode = TOOL_DRAW;
    bool _mouse_left_down = false;
    bool _mouse_right_down = false;
    int _last_mouse_x = 0;
    int _last_mouse_y = 0;
    int _mouse_x = 0;
    int _mouse_y = 0;
    bool _mouse_inside = false;
    std::vector<ObstaclePtr> _obstacles;
    ViaPointContainer _via_points;
    RobotFootprintModelPtr _robot_model;
    QLabel* _astarCostLabel = nullptr;
    QLabel* _astarTebCostLabel = nullptr;
    QLabel* _tebCostLabel = nullptr;
    QLabel* _statusLabel = nullptr;
    QPushButton* _planButton = nullptr;
    QDoubleSpinBox* _startXControl = nullptr;
    QDoubleSpinBox* _startYControl = nullptr;
    QSlider* _startThetaControl = nullptr;
    QLabel* _startThetaLabel = nullptr;
    QDoubleSpinBox* _endXControl = nullptr;
    QDoubleSpinBox* _endYControl = nullptr;
    QSlider* _endThetaControl = nullptr;
    QLabel* _endThetaLabel = nullptr;
    QFutureWatcher<PlanResult>* _plan_watcher = nullptr;
    PlanResult _last_plan;
    bool _has_plan = false;
    bool _planning = false;
    bool _plan_after_current = false;
    bool _scene_dirty = true;
    bool _obstacles_dirty = false;
    int _plan_generation = 0;
    QString _sceneFile;

    // Animation state
    bool _animating = false;
    int _anim_frame = 0;
    double _anim_speed = 1.0;  // multiplier: 0.5x ~ 3x

};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QWidget window;
    window.setWindowTitle("TEB Local Planner");

    TebDisplayWidget* display = new TebDisplayWidget;
    display->setFixedSize(1500, 1500);

    // --- Start pose controls ---
    QLabel* startTitle = new QLabel("<b>Start Pose</b>");

    QDoubleSpinBox* startX = new QDoubleSpinBox;
    startX->setRange(-10.0, 10.0);
    startX->setSingleStep(0.1);
    startX->setValue(-2.0);
    startX->setDecimals(1);
    QLabel* startXLabel = new QLabel("x:");

    QDoubleSpinBox* startY = new QDoubleSpinBox;
    startY->setRange(-10.0, 10.0);
    startY->setSingleStep(0.1);
    startY->setValue(0.0);
    startY->setDecimals(1);
    QLabel* startYLabel = new QLabel("y:");

    QSlider* startThetaSlider = new QSlider(Qt::Horizontal);
    startThetaSlider->setRange(-314, 314);
    QLabel* startThetaLabel = new QLabel("theta: 0.00");

    // --- Goal pose controls ---
    QLabel* endTitle = new QLabel("<b>Goal Pose</b>");

    QDoubleSpinBox* endX = new QDoubleSpinBox;
    endX->setRange(-10.0, 10.0);
    endX->setSingleStep(0.1);
    endX->setValue(2.0);
    endX->setDecimals(1);
    QLabel* endXLabel = new QLabel("x:");

    QDoubleSpinBox* endY = new QDoubleSpinBox;
    endY->setRange(-10.0, 10.0);
    endY->setSingleStep(0.1);
    endY->setValue(0.0);
    endY->setDecimals(1);
    QLabel* endYLabel = new QLabel("y:");

    QSlider* endThetaSlider = new QSlider(Qt::Horizontal);
    endThetaSlider->setRange(-314, 314);
    QLabel* endThetaLabel = new QLabel("theta: 0.00");

    // --- Connect signals ---
    QObject::connect(startX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), display, &TebDisplayWidget::setStartX);
    QObject::connect(startY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), display, &TebDisplayWidget::setStartY);
    QObject::connect(endX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), display, &TebDisplayWidget::setEndX);
    QObject::connect(endY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), display, &TebDisplayWidget::setEndY);
    QObject::connect(startThetaSlider, &QSlider::valueChanged, display, &TebDisplayWidget::setStartTheta);
    QObject::connect(endThetaSlider, &QSlider::valueChanged, display, &TebDisplayWidget::setEndTheta);
    QObject::connect(startThetaSlider, &QSlider::valueChanged, [startThetaLabel](int v) {
        startThetaLabel->setText(QString("theta: %1").arg(v * 0.01, 0, 'f', 2));
    });
    QObject::connect(endThetaSlider, &QSlider::valueChanged, [endThetaLabel](int v) {
        endThetaLabel->setText(QString("theta: %1").arg(v * 0.01, 0, 'f', 2));
    });
    display->setPoseControls(startX, startY, startThetaSlider, startThetaLabel,
                             endX, endY, endThetaSlider, endThetaLabel);

    // --- Right panel layout ---
    QVBoxLayout* panelLayout = new QVBoxLayout;

    // Start pose row
    panelLayout->addWidget(startTitle);
    QHBoxLayout* startRow = new QHBoxLayout;
    startRow->addWidget(startXLabel);
    startRow->addWidget(startX);
    startRow->addWidget(startYLabel);
    startRow->addWidget(startY);
    startRow->addWidget(startThetaLabel);
    startRow->addWidget(startThetaSlider);
    panelLayout->addLayout(startRow);

    // Goal pose row
    panelLayout->addWidget(endTitle);
    QHBoxLayout* endRow = new QHBoxLayout;
    endRow->addWidget(endXLabel);
    endRow->addWidget(endX);
    endRow->addWidget(endYLabel);
    endRow->addWidget(endY);
    endRow->addWidget(endThetaLabel);
    endRow->addWidget(endThetaSlider);
    panelLayout->addLayout(endRow);

    QHBoxLayout* sceneRow = new QHBoxLayout;
    QPushButton* openSceneBtn = new QPushButton("Open Scene");
    QPushButton* saveSceneBtn = new QPushButton("Save Scene");
    QObject::connect(openSceneBtn, &QPushButton::clicked, display, &TebDisplayWidget::openScene);
    QObject::connect(saveSceneBtn, &QPushButton::clicked, display, &TebDisplayWidget::saveScene);
    sceneRow->addWidget(openSceneBtn);
    sceneRow->addWidget(saveSceneBtn);
    panelLayout->addLayout(sceneRow);

    QPushButton* editConfigBtn = new QPushButton("Edit Config");
    QObject::connect(editConfigBtn, &QPushButton::clicked, display, &TebDisplayWidget::editConfig);
    panelLayout->addWidget(editConfigBtn);

    // Tool mode + Brush size control
    QHBoxLayout* toolRow = new QHBoxLayout;
    QPushButton* drawBtn = new QPushButton("Draw");
    QPushButton* eraseBtn = new QPushButton("Erase");
    drawBtn->setCheckable(true);
    eraseBtn->setCheckable(true);
    drawBtn->setChecked(true);
    QButtonGroup* toolGroup = new QButtonGroup;
    toolGroup->addButton(drawBtn, TOOL_DRAW);
    toolGroup->addButton(eraseBtn, TOOL_ERASE);
    QObject::connect(toolGroup, &QButtonGroup::idClicked,
        display, &TebDisplayWidget::setToolMode);
    toolRow->addWidget(drawBtn);
    toolRow->addWidget(eraseBtn);

    toolRow->addWidget(new QLabel("  Size:"));
    QSlider* brushSizeSlider = new QSlider(Qt::Horizontal);
    brushSizeSlider->setRange(0, BRUSH_SIZE_COUNT - 1);
    brushSizeSlider->setValue(2); // default 0.5m
    QLabel* brushSizeLabel = new QLabel("0.5m");
    QObject::connect(brushSizeSlider, &QSlider::valueChanged, display, &TebDisplayWidget::setBrushSizeIdx);
    QObject::connect(brushSizeSlider, &QSlider::valueChanged, [brushSizeLabel](int idx) {
        brushSizeLabel->setText(QString("%1m").arg(BRUSH_SIZES[idx], 0, 'f', 1));
    });
    toolRow->addWidget(brushSizeSlider);
    toolRow->addWidget(brushSizeLabel);
    panelLayout->addLayout(toolRow);

    // Plan button
    QPushButton* planBtn = new QPushButton("Plan");
    QObject::connect(planBtn, &QPushButton::clicked, display, &TebDisplayWidget::runPlanner);
    panelLayout->addWidget(planBtn);

    // Animate button + speed slider
    QPushButton* animateBtn = new QPushButton("Animate");
    animateBtn->setCheckable(true);
    QObject::connect(animateBtn, &QPushButton::toggled, display, &TebDisplayWidget::toggleAnimation);
    QHBoxLayout* animRow = new QHBoxLayout;
    animRow->addWidget(animateBtn);
    animRow->addWidget(new QLabel("Speed:"));
    QDoubleSpinBox* animSpeedSpin = new QDoubleSpinBox;
    animSpeedSpin->setRange(0.5, 3.0);
    animSpeedSpin->setSingleStep(0.5);
    animSpeedSpin->setValue(1.0);
    animSpeedSpin->setDecimals(1);
    animSpeedSpin->setSuffix("x");
    QObject::connect(animSpeedSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     display, &TebDisplayWidget::setAnimSpeed);
    animRow->addWidget(animSpeedSpin);
    panelLayout->addLayout(animRow);

    // Clear Grid button
    QPushButton* clearBtn = new QPushButton("Clear Grid");
    QObject::connect(clearBtn, &QPushButton::clicked, display, &TebDisplayWidget::clearGrid);
    panelLayout->addWidget(clearBtn);

    // Zoom control
    QHBoxLayout* zoomRow = new QHBoxLayout;
    QLabel* zoomLabel = new QLabel("Zoom:");
    QSlider* zoomSlider = new QSlider(Qt::Horizontal);
    zoomSlider->setRange(5, 200);
    zoomSlider->setValue(38);
    QLabel* zoomValue = new QLabel("0.38x");
    zoomRow->addWidget(zoomLabel);
    zoomRow->addWidget(zoomSlider);
    zoomRow->addWidget(zoomValue);
    QObject::connect(zoomSlider, &QSlider::valueChanged, display, &TebDisplayWidget::setZoom);
    QObject::connect(zoomSlider, &QSlider::valueChanged, [zoomValue](int v) {
        zoomValue->setText(QString("%1x").arg(v / 100.0, 0, 'f', 2));
    });
    panelLayout->addLayout(zoomRow);

    // Cost display
    QLabel* astarCostLabel = new QLabel("A* path len: --");
    QLabel* astarTebCostLabel = new QLabel("A* TEB cost: --");
    QLabel* tebCostLabel = new QLabel("TEB cost: --");
    QLabel* statusLabel = new QLabel("Status: Ready");
    panelLayout->addWidget(astarCostLabel);
    panelLayout->addWidget(astarTebCostLabel);
    panelLayout->addWidget(tebCostLabel);
    panelLayout->addWidget(statusLabel);
    display->setCostLabels(astarCostLabel, astarTebCostLabel, tebCostLabel);
    display->setPlanningControls(planBtn, statusLabel);

    panelLayout->addStretch();

    // --- Main layout: display left, panel right ---
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(display);
    layout->addLayout(panelLayout);

    window.setLayout(layout);
    window.resize(1800, 1520);
    window.show();

    // Run initial plan
    display->runPlanner();

    return app.exec();
}

#include "main.moc"
