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
#include <fstream>

#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
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
        , _image(500, 500, QImage::Format_RGB888)
    {
        _image.fill(Qt::black);

        _obstacles.emplace_back(boost::make_shared<PointObstacle>(0, 0));
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
        _image.fill(Qt::black);
        QPainter painter(&_image);

        try
        {
            _start.x() = _start_x;
            _start.y() = _start_y;
            _start.theta() = _start_theta * 0.01;
            _end.x() = _end_x;
            _end.y() = _end_y;
            _end.theta() = _end_theta * 0.01;

            _planner->plan(_start, _end);

            std::vector<Eigen::Vector3f> path;
            _planner->getFullTrajectory(path);

            painter.setPen(QPen(Qt::white, 1));
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                int x = static_cast<int>(path[i][0] * 100.f + 250);
                int y = static_cast<int>(path[i][1] * 100.f + 250);
                int nx = static_cast<int>(path[i + 1][0] * 100.f + 250);
                int ny = static_cast<int>(path[i + 1][1] * 100.f + 250);
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
    display->setFixedSize(500, 500);

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

    window.setLayout(layout);
    window.show();

    return app.exec();
}

#include "main.moc"
