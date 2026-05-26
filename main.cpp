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
#include <cmath>
#include <fstream>

#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/occupancy_grid.h"
#include "inc/optimal_planner.h"
#include <boost/smart_ptr.hpp>

using namespace teb_local_planner;

enum ToolMode { TOOL_DRAW, TOOL_ERASE };

static const double BRUSH_SIZES[] = { 0.1, 0.2, 0.5, 1.0, 2.0 };
static const int BRUSH_SIZE_COUNT = 5;

class TebDisplayWidget : public QWidget
{
    Q_OBJECT
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
        _robot_model = boost::make_shared<CircularRobotFootprint>(0.4);
        _visual = TebVisualizationPtr(new TebVisualization(_config));

        _configFile = "teb_config.json";
        std::ifstream ifs(_configFile);
        if (ifs.good()) {
            ifs.close();
            _config.loadFromFile(_configFile);
        } else {
            _config.saveToFile(_configFile);
        }
        _planner = new TebOptimalPlanner(_config, &_obstacles, _robot_model, _visual, &_via_points);

        _timer = new QTimer(this);
        connect(_timer, &QTimer::timeout, this, &TebDisplayWidget::runPlanner);
        _timer->start(30);
    }

    ~TebDisplayWidget()
    {
        delete _planner;
    }

public slots:
    void setZoom(int percent)
    {
        _zoom = percent / 100.0;
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
    }

    void setEndTheta(int value)
    {
        _end_theta = value;
    }

    void setStartX(double v) { _start_x = v; }
    void setStartY(double v) { _start_y = v; }
    void setEndX(double v)   { _end_x = v; }
    void setEndY(double v)   { _end_y = v; }

    void setBrushRadius(double r) { _brush_radius = r; }
    void setToolMode(int mode) { _tool_mode = static_cast<ToolMode>(mode); }
    void setBrushSizeIdx(int idx) { _brush_radius = BRUSH_SIZES[idx]; }

    void clearGrid()
    {
        _grid.clear();
        _grid.extractObstacles(_obstacles);
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
            recreatePlanner();
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

    void runPlanner()
    {
        _grid.extractObstacles(_obstacles);

        _image.fill(Qt::gray);
        QPainter painter(&_image);

        double s = getScale();
        int gw = _grid.getWidth(), gh = _grid.getHeight();
        double res = _grid.getResolution();
        double ox = _grid.getOriginX(), oy = _grid.getOriginY();

        // Draw grid: occupied cells (skip cells outside viewport for performance)
        int cell_px = static_cast<int>(std::ceil(res * s));
        int min_ix = std::max(0, static_cast<int>(std::ceil((-750.0 - _center_x - ox) / (res * s))));
        int max_ix = std::min(gw - 1, static_cast<int>(std::floor((750.0 - _center_x - ox) / (res * s))));
        int min_iy = std::max(0, static_cast<int>(std::ceil((-750.0 - _center_y - oy) / (res * s))));
        int max_iy = std::min(gh - 1, static_cast<int>(std::floor((750.0 - _center_y - oy) / (res * s))));
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

        try
        {
            // Draw original planned line (straight from start to goal)
            painter.setPen(QPen(QColor(255, 200, 50), 2));
            int psx, psy, pgx, pgy;
            worldToPixel(_start_x, _start_y, psx, psy);
            worldToPixel(_end_x, _end_y, pgx, pgy);
            painter.drawLine(psx, psy, pgx, pgy);

            _planner->plan(_start, _end);

            std::vector<Eigen::Vector3f> path;
            _planner->getFullTrajectory(path);

            // Draw optimized trajectory
            painter.setPen(QPen(Qt::white, 1));
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                int x, y, nx, ny;
                worldToPixel(path[i][0], path[i][1], x, y);
                worldToPixel(path[i + 1][0], path[i + 1][1], nx, ny);
                painter.drawLine(x, y, nx, ny);
            }
        }
        catch (...)
        {
            _timer->stop();
        }

        painter.end();
        update();
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
    }

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton)  _mouse_left_down = false;
        if (event->button() == Qt::RightButton) _mouse_right_down = false;
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

        update();
    }

private:
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
    TebVisualizationPtr _visual;
    TebOptimalPlanner* _planner;

    void recreatePlanner()
    {
        delete _planner;
        _planner = new TebOptimalPlanner(_config, &_obstacles, _robot_model, _visual, &_via_points);
    }
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

    // --- Layout ---
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(display);

    // Start pose row
    layout->addWidget(startTitle);
    QHBoxLayout* startRow = new QHBoxLayout;
    startRow->addWidget(startXLabel);
    startRow->addWidget(startX);
    startRow->addWidget(startYLabel);
    startRow->addWidget(startY);
    startRow->addWidget(startThetaLabel);
    startRow->addWidget(startThetaSlider);
    layout->addLayout(startRow);

    // Goal pose row
    layout->addWidget(endTitle);
    QHBoxLayout* endRow = new QHBoxLayout;
    endRow->addWidget(endXLabel);
    endRow->addWidget(endX);
    endRow->addWidget(endYLabel);
    endRow->addWidget(endY);
    endRow->addWidget(endThetaLabel);
    endRow->addWidget(endThetaSlider);
    layout->addLayout(endRow);

    QPushButton* editConfigBtn = new QPushButton("Edit Config");
    QObject::connect(editConfigBtn, &QPushButton::clicked, display, &TebDisplayWidget::editConfig);
    layout->addWidget(editConfigBtn);

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
    layout->addLayout(toolRow);

    // Clear Grid button
    QPushButton* clearBtn = new QPushButton("Clear Grid");
    QObject::connect(clearBtn, &QPushButton::clicked, display, &TebDisplayWidget::clearGrid);
    layout->addWidget(clearBtn);

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
    layout->addLayout(zoomRow);

    window.setLayout(layout);
    window.show();

    return app.exec();
}

#include "main.moc"
