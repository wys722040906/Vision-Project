#include "predict/parabolaViwer.h"

namespace detectAndTrack{

ParabolaViewer::ParabolaViewer(QWidget *parent) 
    : QOpenGLWidget(parent)  // Properly initialize the parent class
{
    setFocusPolicy(Qt::StrongFocus);
}
void ParabolaViewer::setPlaneHeight(double height)
{
    plane_y = height;
    update();
}
void ParabolaViewer::drawPlane()
{
    const float AXIS_LENGTH = 250.0f;
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // 绘制半透明平面
    glColor4f(0.5f, 0.5f, 0.5f, 0.3f);
    glBegin(GL_QUADS);
    glVertex3f(-AXIS_LENGTH, plane_y, -AXIS_LENGTH);
    glVertex3f(AXIS_LENGTH, plane_y, -AXIS_LENGTH);
    glVertex3f(AXIS_LENGTH, plane_y, AXIS_LENGTH);
    glVertex3f(-AXIS_LENGTH, plane_y, AXIS_LENGTH);
    glEnd();
    
    // 绘制平面边框
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-AXIS_LENGTH, plane_y, -AXIS_LENGTH);
    glVertex3f(AXIS_LENGTH, plane_y, -AXIS_LENGTH);
    glVertex3f(AXIS_LENGTH, plane_y, AXIS_LENGTH);
    glVertex3f(-AXIS_LENGTH, plane_y, AXIS_LENGTH);
    glEnd();
    
    glDisable(GL_BLEND);
}

void ParabolaViewer::setPoints(const std::vector<std::vector<float>>& points)
{
    if (points.empty() || points[0].size() != 3) {
        return;
    }

    markerPoints.clear();
    
    for (const auto& point : points) {
        float x = point[0];
        float y = point[1];
        float z = point[2];
        markerPoints.append(QVector3D(x, y, z));
    }
    // 错误在这里：直接使用 markerPoints，不需要转换
    fitParabola3D(markerPoints);
    update();
}

void ParabolaViewer::clearAll()
{
    markerPoints.clear();
    parabola_ax = 0.0; parabola_bx = 0.0; parabola_cx = 0.0;
    parabola_ay = 0.0; parabola_by = 0.0; parabola_cy = 0.0;
    parabola_az = 0.0; parabola_bz = 0.0; parabola_cz = 0.0;
    update();
}

void ParabolaViewer::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

void ParabolaViewer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void ParabolaViewer::paintGL() 
{
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, width() / (float)height(), 0.1f, 1000.0f);  // 增加远平面距离

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -500.0f);  // 调整观察距离以看到整个坐标系

        glRotatef(xRot, 1.0f, 0.0f, 0.0f);
        glRotatef(yRot, 0.0f, 1.0f, 0.0f);
        glScalef(scale, scale, scale);

        drawCoordinateSystem();
        drawPlane();  // 在绘制抛物线之前绘制平面
        drawParabola();
        drawMarkerPoints();

        // 在OpenGL渲染完成后使用QPainter绘制文本
        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 10));
            
        // 显示方程
        QString equation = QString("x = %1t² + %2t + %3\ny = %4t² + %5t + %6\nz = %7t² + %8t + %9")
            .arg(parabola_ax, 0, 'f', 3)
            .arg(parabola_bx, 0, 'f', 3)
            .arg(parabola_cx, 0, 'f', 3)
            .arg(parabola_ay, 0, 'f', 3)
            .arg(parabola_by, 0, 'f', 3)
            .arg(parabola_cy, 0, 'f', 3)
            .arg(parabola_az, 0, 'f', 3)
            .arg(parabola_bz, 0, 'f', 3)
            .arg(parabola_cz, 0, 'f', 3);
        painter.drawText(10, 20, equation);
        
        // 显示平面高度
        painter.drawText(10, 80, QString("Plane Height: y = %1").arg(plane_y, 0, 'f', 2));
        
        // 显示交点坐标
        if (has_intersection) {
            QString point1_text = QString("Intersection 1: (%1, %2, %3)")
                .arg(intersection_point1.x(), 0, 'f', 2)
                .arg(intersection_point1.y(), 0, 'f', 2)
                .arg(intersection_point1.z(), 0, 'f', 2);
            painter.drawText(10, 100, point1_text);
            
            if (intersection_times.size() == 2) {
                QString point2_text = QString("Intersection 2: (%1, %2, %3)")
                    .arg(intersection_point2.x(), 0, 'f', 2)
                    .arg(intersection_point2.y(), 0, 'f', 2)
                    .arg(intersection_point2.z(), 0, 'f', 2);
                painter.drawText(10, 120, point2_text);
            }
        }
}

void ParabolaViewer::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void ParabolaViewer::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton)
    {
        xRot += dy;
        yRot += dx;
        update();
    }
    lastPos = event->pos();
}

void ParabolaViewer::wheelEvent(QWheelEvent *event)
{
    float delta = event->angleDelta().y() / 120.0f;
    scale += delta * 0.1f;
    if(scale < 0.1f) scale = 0.1f;
    if(scale > 10.0f) scale = 10.0f;
    update();
}


//最小二乘法的抛物线拟合
// void ParabolaViewer::fitParabola3D(const QVector<QVector3D>& points)
// {
//     if (points.size() < 3) return;
    
//     std::vector<double> t_values;
//     double total_length = 0.0;
    
//     t_values.push_back(0.0);
//     for (size_t i = 1; i < points.size(); i++) {
//         QVector3D diff = points[i] - points[i-1];
//         total_length += diff.length();
//         t_values.push_back(total_length);
//     }
    
//     for (double& t : t_values) {
//         t /= total_length;
//     }
    
//     size_t n = points.size();
//     Eigen::MatrixXd A(n, 3);
//     Eigen::VectorXd bx(n), by(n), bz(n);
    
//     for (size_t i = 0; i < n; i++) {
//         double t = t_values[i];
//         A(i, 0) = t * t;
//         A(i, 1) = t;
//         A(i, 2) = 1.0;
//         bx(i) = points[i].x();
//         by(i) = points[i].y();
//         bz(i) = points[i].z();
//     }
    
//     Eigen::Vector3d sol_x = A.colPivHouseholderQr().solve(bx);
//     Eigen::Vector3d sol_y = A.colPivHouseholderQr().solve(by);
//     Eigen::Vector3d sol_z = A.colPivHouseholderQr().solve(bz);
    
//     parabola_ax = sol_x(0); parabola_bx = sol_x(1); parabola_cx = sol_x(2);
//     parabola_ay = sol_y(0); parabola_by = sol_y(1); parabola_cy = sol_y(2);
//     parabola_az = sol_z(0); parabola_bz = sol_z(1); parabola_cz = sol_z(2);
// }

//物理约束法的抛物线拟合--已知重力方向
void ParabolaViewer::fitParabola3D(const QVector<QVector3D>& points)
{
/*
g: 用于补偿重力影响，影响抛物线在z方向的拟合精度
500.0: 用于从空间距离估算时间间隔，影响时间参数化的准确性--相邻数据的时间差
    相机帧率：
        double fps = 60.0;  // 相机帧率
        double dt = 1.0 / fps;  // 每帧时间间隔
    时间戳（更准确）：
        for (size_t i = 1; i < points.size(); i++) {
            double dt = points[i].timestamp - points[i-1].timestamp;
            total_time += dt;
            time_values.push_back(total_time);
        }
0.5和1.5: 控制拟合曲线的显示范围，决定抛物线向前和向后延伸多远
*/

    if (points.size() < 3) return;
    
    // 重力加速度 (cm/s²)
    const double g = 980.0;  // 9.8 m/s² = 980 cm/s²
    
    size_t n = points.size();
    Eigen::MatrixXd A(n, 4);  // 增加一列用于时间平方项
    Eigen::VectorXd bx(n), by(n), bz(n);
    
    // 估计时间间隔
    double total_time = 0.0;
    std::vector<double> time_values;
    time_values.push_back(0.0);
    
    // 使用点间距离估计时间
    for (size_t i = 1; i < points.size(); i++) {
        QVector3D diff = points[i] - points[i-1];
        double dist = diff.length();
        // 假设平均速度为一个合理值（例如500 cm/s）
        // double dt = dist / 500.0;
        double dt = 0.033333333;
        total_time += dt;
        time_values.push_back(total_time);
    }
    
    // 构建方程组
    for (size_t i = 0; i < n; i++) {
        double t = time_values[i];
        A(i, 0) = t * t;  // x: t²项
        A(i, 1) = t;      // x: t项
        A(i, 2) = 1.0;    // x: 常数项
        A(i, 3) = 0.0;    // x: 不需要重力项
        
        bx(i) = points[i].x();
        by(i) = points[i].y() - 0.5 * g * t * t;  // 在y负方向补偿重力影响（注意符号改为减号）
        bz(i) = points[i].z();
    }
    
    // 解最小二乘问题
    Eigen::Vector4d sol_x = A.colPivHouseholderQr().solve(bx);
    Eigen::Vector4d sol_y = A.colPivHouseholderQr().solve(by);
    Eigen::Vector4d sol_z = A.colPivHouseholderQr().solve(bz);
    
    // 保存拟合参数
    parabola_ax = sol_x(0);
    parabola_bx = sol_x(1);
    parabola_cx = sol_x(2);
    
    parabola_ay = sol_y(0);
    parabola_by = sol_y(1);
    parabola_cy = sol_y(2);
    
    // z方向考虑重力影响
    parabola_az = sol_z(0);
    parabola_bz = sol_z(1);
    parabola_cz = sol_z(2);
    
    // 更新绘制范围
    double max_time = time_values.back();
    t_start = -max_time * 0.5;  // 向前延伸
    t_end = max_time * 1.5;     // 向后延伸
}

//RANSAC算法鲁棒性抛物线拟合--噪声离散点
// void ParabolaViewer::fitParabola3D(const QVector<QVector3D>& points)
// {
//     if (points.size() < 3) return;
    
//     const int iterations = 100;
//     const double threshold = 10.0;  // 内点阈值（厘米）
    
//     std::random_device rd;
//     std::mt19937 gen(rd());
    
//     double best_ax = 0, best_bx = 0, best_cx = 0;
//     double best_ay = 0, best_by = 0, best_cy = 0;
//     double best_az = 0, best_bz = 0, best_cz = 0;
//     int max_inliers = 0;
    
//     for (int iter = 0; iter < iterations; iter++) {
//         // 随机选择3个点
//         std::vector<int> indices(points.size());
//         std::iota(indices.begin(), indices.end(), 0);
//         std::shuffle(indices.begin(), indices.end(), gen);
        
//         if (points.size() < 3) continue;
        
//         QVector<QVector3D> sample_points;
//         for (int i = 0; i < 3; i++) {
//             sample_points.push_back(points[indices[i]]);
//         }
        
//         // 使用3个点拟合抛物线
//         std::vector<double> t_values = {0.0, 0.5, 1.0};
        
//         Eigen::Matrix3d A;
//         Eigen::Vector3d bx, by, bz;
        
//         for (int i = 0; i < 3; i++) {
//             double t = t_values[i];
//             A(i, 0) = t * t;
//             A(i, 1) = t;
//             A(i, 2) = 1.0;
            
//             bx(i) = sample_points[i].x();
//             by(i) = sample_points[i].y();
//             bz(i) = sample_points[i].z();
//         }
        
//         Eigen::Vector3d sol_x = A.colPivHouseholderQr().solve(bx);
//         Eigen::Vector3d sol_y = A.colPivHouseholderQr().solve(by);
//         Eigen::Vector3d sol_z = A.colPivHouseholderQr().solve(bz);
        
//         // 计算内点数量
//         int inliers = 0;
//         for (const auto& point : points) {
//             // 找到最近的投影点
//             double min_dist = std::numeric_limits<double>::max();
//             for (double t = 0; t <= 1.0; t += 0.01) {
//                 double x = sol_x(0) * t * t + sol_x(1) * t + sol_x(2);
//                 double y = sol_y(0) * t * t + sol_y(1) * t + sol_y(2);
//                 double z = sol_z(0) * t * t + sol_z(1) * t + sol_z(2);
                
//                 double dist = std::sqrt(
//                     std::pow(x - point.x(), 2) +
//                     std::pow(y - point.y(), 2) +
//                     std::pow(z - point.z(), 2)
//                 );
//                 min_dist = std::min(min_dist, dist);
//             }
            
//             if (min_dist < threshold) {
//                 inliers++;
//             }
//         }
        
//         // 更新最佳模型
//         if (inliers > max_inliers) {
//             max_inliers = inliers;
//             best_ax = sol_x(0); best_bx = sol_x(1); best_cx = sol_x(2);
//             best_ay = sol_y(0); best_by = sol_y(1); best_cy = sol_y(2);
//             best_az = sol_z(0); best_bz = sol_z(1); best_cz = sol_z(2);
//         }
//     }
    
//     // 使用最佳参数
//     parabola_ax = best_ax; parabola_bx = best_bx; parabola_cx = best_cx;
//     parabola_ay = best_ay; parabola_by = best_by; parabola_cy = best_cy;
//     parabola_az = best_az; parabola_bz = best_bz; parabola_cz = best_cz;
// }


void ParabolaViewer::drawCoordinateSystem()
{
        // 修改坐标轴范围为 ±250 (±2.5米)
        const float AXIS_LENGTH = 250.0f;
        const float TICK_INTERVAL = 10.0f;  // 每10cm一个刻度

        glLineWidth(2.0f);
        
        // 主坐标轴
        glBegin(GL_LINES);
        // X轴 (红色)
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(-AXIS_LENGTH, 0.0f, 0.0f);
        glVertex3f(AXIS_LENGTH, 0.0f, 0.0f);

        // Y轴 (绿色)
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, -AXIS_LENGTH, 0.0f);
        glVertex3f(0.0f, AXIS_LENGTH, 0.0f);

        // Z轴 (蓝色)
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, -AXIS_LENGTH);
        glVertex3f(0.0f, 0.0f, AXIS_LENGTH);
        glEnd();

        // 绘制刻度
        glColor3f(0.7f, 0.7f, 0.7f);
        glLineWidth(1.0f);

        // 绘制刻度线
        for(float i = -AXIS_LENGTH; i <= AXIS_LENGTH; i += TICK_INTERVAL) {
            if(i == 0) continue;
            
            // X轴刻度
            glBegin(GL_LINES);
            glVertex3f(i, -2.0f, 0.0f);
            glVertex3f(i, 2.0f, 0.0f);
            glEnd();

            // Y轴刻度
            glBegin(GL_LINES);
            glVertex3f(-2.0f, i, 0.0f);
            glVertex3f(2.0f, i, 0.0f);
            glEnd();

            // Z轴刻度
            glBegin(GL_LINES);
            glVertex3f(0.0f, -2.0f, i);
            glVertex3f(0.0f, 2.0f, i);
            glEnd();
        }

        // 使用QPainter绘制文本标注
        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 8));

        // 转换3D坐标到屏幕坐标
        GLdouble modelview[16];
        GLdouble projection[16];
        GLint viewport[4];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        glGetIntegerv(GL_VIEWPORT, viewport);

        // 绘制坐标轴标签
        GLdouble winX, winY, winZ;
        
        // X轴标签
        gluProject(AXIS_LENGTH + 10, 0.0, 0.0, modelview, projection, viewport, &winX, &winY, &winZ);
        painter.drawText(winX, height() - winY, "X (cm)");
        
        // Y轴标签
        gluProject(0.0, AXIS_LENGTH + 10, 0.0, modelview, projection, viewport, &winX, &winY, &winZ);
        painter.drawText(winX, height() - winY, "Y (cm)");
        
        // Z轴标签
        gluProject(0.0, 0.0, AXIS_LENGTH + 10, modelview, projection, viewport, &winX, &winY, &winZ);
        painter.drawText(winX, height() - winY, "Z (cm)");

        // 每50cm绘制一个刻度值（减少标签密度）
        for(float i = -AXIS_LENGTH; i <= AXIS_LENGTH; i += 50.0f) {
            if(i == 0) continue;
            
            // X轴刻度值
            gluProject(i, -5.0, 0.0, modelview, projection, viewport, &winX, &winY, &winZ);
            painter.drawText(winX - 10, height() - winY + 15, QString::number(static_cast<int>(i)));
            
            // Y轴刻度值
            gluProject(-5.0, i, 0.0, modelview, projection, viewport, &winX, &winY, &winZ);
            painter.drawText(winX - 25, height() - winY, QString::number(static_cast<int>(i)));
            
            // Z轴刻度值
            gluProject(0.0, -5.0, i, modelview, projection, viewport, &winX, &winY, &winZ);
            painter.drawText(winX + 5, height() - winY, QString::number(static_cast<int>(i)));
        }

        // 原点标记
        gluProject(5.0, 5.0, 5.0, modelview, projection, viewport, &winX, &winY, &winZ);
        painter.drawText(winX, height() - winY, "O");
}

void ParabolaViewer::drawParabola()
{
    if (markerPoints.size() < 3) return;

    const int numPoints = 200;
    const float AXIS_LENGTH = 250.0f;
    const float EXTENSION = 50.0f;  // 允许超出数据点的范围
    const float BELOW_PLANE = 50.0f;  // 允许在平面下显示的范围
    
    // 找到数据点的范围
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& point : markerPoints) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }
    
    // 扩展范围
    min_x -= EXTENSION;
    max_x += EXTENSION;
    min_z -= EXTENSION;
    max_z += EXTENSION;
    
    // 计算合适的参数范围
    double t_min = -3.0;
    double t_max = 3.0;
    
    // 找到在坐标轴范围内的t值范围
    std::vector<double> valid_t;
    for (int i = 0; i <= numPoints; i++) {
        double t = t_min + (t_max - t_min) * i / static_cast<double>(numPoints);
        double x = parabola_ax * t * t + parabola_bx * t + parabola_cx;
        double y = parabola_ay * t * t + parabola_by * t + parabola_cy;
        double z = parabola_az * t * t + parabola_bz * t + parabola_cz;
        
        if (std::abs(x) <= AXIS_LENGTH && 
            std::abs(y) <= AXIS_LENGTH && 
            std::abs(z) <= AXIS_LENGTH) {
            valid_t.push_back(t);
        }
    }
    
    if (valid_t.empty()) return;
    
    t_min = *std::min_element(valid_t.begin(), valid_t.end());
    t_max = *std::max_element(valid_t.begin(), valid_t.end());
    
    intersection_times.clear();
    has_intersection = false;
    
    // 先绘制数据点
    drawMarkerPoints();
    
    // 启用线条抗锯齿
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // 分两段绘制抛物线：平面上部分和平面下部分
    
    // 1. 平面上部分（实线）
    glLineWidth(4.0f);
    glColor4f(1.0f, 1.0f, 0.0f, 0.8f);  // 黄色
    
    bool started = false;
    glBegin(GL_LINE_STRIP);
    
    for (int i = 0; i <= numPoints; i++) {
        double t = t_min + (t_max - t_min) * i / static_cast<double>(numPoints);
        double x = parabola_ax * t * t + parabola_bx * t + parabola_cx;
        double y = parabola_ay * t * t + parabola_by * t + parabola_cy;
        double z = parabola_az * t * t + parabola_bz * t + parabola_cz;
        
        if (std::abs(x) <= AXIS_LENGTH && 
            std::abs(y) <= AXIS_LENGTH && 
            std::abs(z) <= AXIS_LENGTH && 
            y >= plane_y) {
            
            if (!started) started = true;
            glVertex3f(x, y, z);
        }
    }
    glEnd();
    
    // 2. 平面下部分（虚线）
    glLineWidth(3.0f);
    glColor4f(1.0f, 1.0f, 0.0f, 0.4f);  // 半透明黄色
    
    glEnable(GL_LINE_STIPPLE);  // 启用虚线模式
    glLineStipple(1, 0x00FF);   // 设置虚线样式
    
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= numPoints; i++) {
        double t = t_min + (t_max - t_min) * i / static_cast<double>(numPoints);
        double x = parabola_ax * t * t + parabola_bx * t + parabola_cx;
        double y = parabola_ay * t * t + parabola_by * t + parabola_cy;
        double z = parabola_az * t * t + parabola_bz * t + parabola_cz;
        
        if (std::abs(x) <= AXIS_LENGTH && 
            std::abs(y) <= AXIS_LENGTH && 
            std::abs(z) <= AXIS_LENGTH && 
            y < plane_y && 
            y >= (plane_y - BELOW_PLANE)) {  // 只显示平面下50cm范围内
            
            glVertex3f(x, y, z);
        }
    }
    glEnd();
    
    glDisable(GL_LINE_STIPPLE);  // 关闭虚线模式
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);
    
    // 检查和绘制交点
    for (int i = 1; i <= numPoints; i++) {
        double t = t_min + (t_max - t_min) * i / static_cast<double>(numPoints);
        double prev_t = t_min + (t_max - t_min) * (i-1) / static_cast<double>(numPoints);
        
        double y = parabola_ay * t * t + parabola_by * t + parabola_cy;
        double prev_y = parabola_ay * prev_t * prev_t + parabola_by * prev_t + parabola_cy;
        
        if ((prev_y - plane_y) * (y - plane_y) <= 0) {
            double t_mid = (prev_t + t) / 2;
            for (int iter = 0; iter < 10; iter++) {
                double y_mid = parabola_ay * t_mid * t_mid + parabola_by * t_mid + parabola_cy;
                if (std::abs(y_mid - plane_y) < 0.001) break;
                if ((y_mid - plane_y) * (prev_y - plane_y) <= 0) {
                    t = t_mid;
                    y = y_mid;
                } else {
                    prev_t = t_mid;
                    prev_y = y_mid;
                }
                t_mid = (prev_t + t) / 2;
            }
            
            double x_intersect = parabola_ax * t_mid * t_mid + parabola_bx * t_mid + parabola_cx;
            double z_intersect = parabola_az * t_mid * t_mid + parabola_bz * t_mid + parabola_cz;
            
            if (std::abs(x_intersect) <= AXIS_LENGTH && 
                std::abs(z_intersect) <= AXIS_LENGTH) {
                intersection_times.push_back(t_mid);
                if (intersection_times.size() == 1) {
                    intersection_point1 = QVector3D(x_intersect, plane_y, z_intersect);
                    has_intersection = true;
                }
            }
        }
    }
    
    // 绘制交点
    if (has_intersection) {
        glPointSize(12.0f);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        glVertex3f(intersection_point1.x(), intersection_point1.y(), intersection_point1.z());
        glEnd();
    }
}

void ParabolaViewer::drawMarkerPoints()
{
    glPointSize(8.0f);
    glColor3f(1.0f, 0.0f, 1.0f);
    
    glBegin(GL_POINTS);
    for(const auto& point : markerPoints)
    {
        glVertex3f(point.x(), point.y(), point.z());
    }
    glEnd();
}
} // namespace predictAndTrack