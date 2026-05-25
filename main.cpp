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

class TebDisplayWidget : public QWidget
{
    Q_OBJECT
public:
    TebDisplayWidget(QWidget* parent = nullptr)
        : QWidget(parent)
        , _start(-2, 0, 0)
        , _end(2, 0, 0)
        , _image(1000, 1000, QImage::Format_RGB888)
        , _grid(0.05, 100, 100, -2.5, -2.5)
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

        // Draw grid: occupied cells
        int gw = _grid.getWidth(), gh = _grid.getHeight();
        double res = _grid.getResolution();
        double ox = _grid.getOriginX(), oy = _grid.getOriginY();
        int cell_px = static_cast<int>(std::ceil(res * 200.0));
        for (int iy = 0; iy < gh; ++iy) {
            for (int ix = 0; ix < gw; ++ix) {
                if (_grid.isOccupied(ix, iy)) {
                    int sx = static_cast<int>((ox + ix * res) * 200.0 + 500);
                    int sy = static_cast<int>((oy + iy * res) * 200.0 + 500);
                    painter.fillRect(sx, sy, cell_px, cell_px, QColor(140, 50, 20));
                }
            }
        }

        // Draw grid lines (1m spacing = every 20 cells at 0.05m res)
        painter.setPen(QPen(QColor(40, 40, 40), 1));
        for (int iy = 0; iy <= gh; iy += 20) {
            int sy = static_cast<int>((oy + iy * res) * 200.0 + 500);
            painter.drawLine(0, sy, 1000, sy);
        }
        for (int ix = 0; ix <= gw; ix += 20) {
            int sx = static_cast<int>((ox + ix * res) * 200.0 + 500);
            painter.drawLine(sx, 0, sx, 1000);
        }

        auto drawArrow = [&](int cx, int cy, double theta_rad, const QColor& color) {
            const int arrow_len = 18;
            const int head_len = 7;
            int tip_x  = cx + static_cast<int>(std::cos(theta_rad) * arrow_len);
            int tip_y  = cy + static_cast<int>(std::sin(theta_rad) * arrow_len);
            int base_x = cx - static_cast<int>(std::cos(theta_rad) * arrow_len);
            int base_y = cy - static_cast<int>(std::sin(theta_rad) * arrow_len);

            painter.setPen(QPen(color, 2));
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

        int sx = static_cast<int>(_start_x * 200.0 + 500);
        int sy = static_cast<int>(_start_y * 200.0 + 500);
        drawArrow(sx, sy, _start_theta * 0.01, Qt::green);

        int gx = static_cast<int>(_end_x * 200.0 + 500);
        int gy = static_cast<int>(_end_y * 200.0 + 500);
        drawArrow(gx, gy, _end_theta * 0.01, Qt::blue);

        _start.x() = _start_x;
        _start.y() = _start_y;
        _start.theta() = _start_theta * 0.01;
        _end.x() = _end_x;
        _end.y() = _end_y;
        _end.theta() = _end_theta * 0.01;

        try
        {
            _planner->plan(_start, _end);

            std::vector<Eigen::Vector3f> path;
            _planner->getFullTrajectory(path);

            painter.setPen(QPen(Qt::white, 1));
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                int x = static_cast<int>(path[i][0] * 200.f + 500);
                int y = static_cast<int>(path[i][1] * 200.f + 500);
                int nx = static_cast<int>(path[i + 1][0] * 200.f + 500);
                int ny = static_cast<int>(path[i + 1][1] * 200.f + 500);
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
        if (!_mouse_left_down && !_mouse_right_down) return;
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
        double wx = (sx - 500.0) / 200.0;
        double wy = (sy - 500.0) / 200.0;
        if (_mouse_left_down)
            _grid.setOccupied(wx, wy, _brush_radius);
        else if (_mouse_right_down)
            _grid.setFree(wx, wy, _brush_radius);
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
    double _brush_radius = 0.15;
    bool _mouse_left_down = false;
    bool _mouse_right_down = false;
    int _last_mouse_x = 0;
    int _last_mouse_y = 0;
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
    display->setFixedSize(1000, 1000);

    // --- Start pose controls ---
    QLabel* startTitle = new QLabel("<b>Start Pose</b>");

    QDoubleSpinBox* startX = new QDoubleSpinBox;
    startX->setRange(-5.0, 5.0);
    startX->setSingleStep(0.1);
    startX->setValue(-2.0);
    startX->setDecimals(1);
    QLabel* startXLabel = new QLabel("x:");

    QDoubleSpinBox* startY = new QDoubleSpinBox;
    startY->setRange(-5.0, 5.0);
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
    endX->setRange(-5.0, 5.0);
    endX->setSingleStep(0.1);
    endX->setValue(2.0);
    endX->setDecimals(1);
    QLabel* endXLabel = new QLabel("x:");

    QDoubleSpinBox* endY = new QDoubleSpinBox;
    endY->setRange(-5.0, 5.0);
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

    // Brush radius control
    QHBoxLayout* brushRow = new QHBoxLayout;
    QLabel* brushLabel = new QLabel("Brush (m):");
    QDoubleSpinBox* brushSpin = new QDoubleSpinBox;
    brushSpin->setRange(0.05, 1.0);
    brushSpin->setSingleStep(0.05);
    brushSpin->setValue(0.15);
    brushSpin->setDecimals(2);
    QObject::connect(brushSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        display, &TebDisplayWidget::setBrushRadius);
    brushRow->addWidget(brushLabel);
    brushRow->addWidget(brushSpin);
    layout->addLayout(brushRow);

    // Clear Grid button
    QPushButton* clearBtn = new QPushButton("Clear Grid");
    QObject::connect(clearBtn, &QPushButton::clicked, display, &TebDisplayWidget::clearGrid);
    layout->addWidget(clearBtn);

    window.setLayout(layout);
    window.show();

    return app.exec();
}

#include "main.moc"
