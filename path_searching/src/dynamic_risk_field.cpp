#include <path_searching/dynamic_risk_field.h>

namespace cane_planner {

double DynamicRiskField::getIndividualCost(const Eigen::Vector2d& q_pos, 
                                           const Eigen::Vector2d& obs_p, 
                                           const Eigen::Vector2d& obs_v) const {
    
    // 1. 计算预测中心 p_pred = p_k + tau * v_k
    Eigen::Vector2d p_pred = obs_p + conf_.tau * obs_v;

    // 2. 距离检查（性能优化）
    Eigen::Vector2d delta = q_pos - p_pred;
    if (delta.norm() > conf_.cutoff_dist) return 0.0;

    // 3. 处理静止或极慢的情况
    double speed = obs_v.norm();
    if (speed < 1e-4) {
        double dist_sq = delta.squaredNorm();
        return conf_.A_risk * std::exp(-0.5 * dist_sq / (conf_.sigma_y * conf_.sigma_y));
    }

    // 4. 旋转变换：将 delta 变换到障碍物局部坐标系（X轴平行于速度方向）
    // 这在数学上等价于 (q-p).T * inv(Sigma) * (q-p)
    double theta = std::atan2(obs_v.y(), obs_v.x());
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);

    double dx_local =  delta.x() * cos_th + delta.y() * sin_th;
    double dy_local = -delta.x() * sin_th + delta.y() * cos_th;

    // 5. 计算指数部分 (马氏距离的平方)
    double exponent = -0.5 * (std::pow(dx_local / conf_.sigma_x, 2) + 
                              std::pow(dy_local / conf_.sigma_y, 2));

    return conf_.A_risk * std::exp(exponent);
}

double DynamicRiskField::getIndividualCostFast(double qx, double qy,
                                                 double ox, double oy,
                                                 double vx, double vy) const
{
    double tau = conf_.tau;
    double cutoff = conf_.cutoff_dist;
    double A = conf_.A_risk;
    double sx = conf_.sigma_x;
    double sy = conf_.sigma_y;

    // 1. Predicted obstacle center
    double px = ox + tau * vx;
    double py = oy + tau * vy;

    // 2. Delta and distance
    double dx = qx - px;
    double dy = qy - py;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq > cutoff * cutoff)
        return 0.0;

    // 3. Speed check
    double speed_sq = vx * vx + vy * vy;
    if (speed_sq < 1e-8)
        return A * std::exp(-0.5 * dist_sq / (sy * sy));

    // 4. Rotate into obstacle-local frame
    double theta = std::atan2(vy, vx);
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    double dx_local =  dx * cos_t + dy * sin_t;
    double dy_local = -dx * sin_t + dy * cos_t;

    // 5. Anisotropic Mahalanobis
    double exponent = -0.5 * ((dx_local / sx) * (dx_local / sx) +
                              (dy_local / sy) * (dy_local / sy));
    return A * std::exp(exponent);
}

} // namespace