#ifndef PARABOLAVIWER_H
#define PARABOLAVIWER_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector3D>
#include <QVector>
#include <QApplication>
#include <QMainWindow>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>

#include <QFont>
#include <GL/glu.h>

#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <random>


namespace detectAndTrack
{

class ParabolaViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit ParabolaViewer(QWidget *parent = nullptr);
    void setPoints(const std::vector<std::vector<float>>& points);
    void clearAll();
    void setPlaneHeight(double height);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void drawPlane();
    void fitParabola3D(const QVector<QVector3D>& points);
    void drawCoordinateSystem();
    void drawParabola();
    void drawMarkerPoints();

    // 3D parabola parameters
    double parabola_ax = 0.0;
    double parabola_bx = 0.0;
    double parabola_cx = 0.0;
    double parabola_ay = 0.0;
    double parabola_by = 0.0;
    double parabola_cy = 0.0;
    double parabola_az = 0.0;
    double parabola_bz = 0.0;
    double parabola_cz = 0.0;
    
    double t_start = -2.0;  // Default values
    double t_end = 2.0;

    QVector<QVector3D> markerPoints;
    QPoint lastPos;
    float xRot = 0.0f;
    float yRot = 0.0f;
    float scale = 1.0f;

    std::vector<double> intersection_times;
    double plane_y = 0.0;
    QVector3D intersection_point1;
    QVector3D intersection_point2;
    bool has_intersection = false;

};

} // namespace predictAndTrack

#endif // PARABOLAVIEWER_


