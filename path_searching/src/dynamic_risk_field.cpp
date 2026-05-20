#include <path_searching/dynamic_risk_field.h>

namespace cane_planner {

double DynamicRiskField::getIndividualCost(const Eigen::Vector2d& q_pos,
                                           const Eigen::Vector2d& obs_p,
                                           const Eigen::Vector2d& obs_v) const {

    // 1. Predicted center p_pred = p_k + tau * v_k
    Eigen::Vector2d p_pred = obs_p + conf_.tau * obs_v;

    // 2. Distance check
    Eigen::Vector2d delta = q_pos - p_pred;
    if (delta.norm() > conf_.cutoff_dist) return 0.0;

    double r_norm_sq;

    // 3. Stationary case
    double speed = obs_v.norm();
    if (speed < 1e-4) {
        r_norm_sq = delta.squaredNorm() / (conf_.sigma_y * conf_.sigma_y);
    } else {
        // 4. Rotate into obstacle-local frame
        double theta = std::atan2(obs_v.y(), obs_v.x());
        double cos_th = std::cos(theta);
        double sin_th = std::sin(theta);

        double dx_local =  delta.x() * cos_th + delta.y() * sin_th;
        double dy_local = -delta.x() * sin_th + delta.y() * cos_th;

        // 5. Normalized squared mahalanobis distance
        r_norm_sq = (dx_local / conf_.sigma_x) * (dx_local / conf_.sigma_x)
                  + (dy_local / conf_.sigma_y) * (dy_local / conf_.sigma_y);
    }

    double core_risk = conf_.A_risk * std::exp(-0.5 * r_norm_sq);

    // 6. Halo: wider low-amplitude envelope beyond the hard core.
    // When halo_scale > 0, risk = max(core, halo) so the hard boundary
    // is preserved but a soft gradient extends further out.
    if (conf_.halo_scale > 0.0 && conf_.halo_ratio > 0.0) {
        double halo_risk = conf_.A_risk * conf_.halo_ratio
                         * std::exp(-0.5 * r_norm_sq / (conf_.halo_scale * conf_.halo_scale));
        return std::max(core_risk, halo_risk);
    }

    return core_risk;
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

    double r_norm_sq;

    // 3. Speed check
    double speed_sq = vx * vx + vy * vy;
    if (speed_sq < 1e-8) {
        r_norm_sq = dist_sq / (sy * sy);
    } else {
        // 4. Rotate into obstacle-local frame
        double theta = std::atan2(vy, vx);
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        double dx_local =  dx * cos_t + dy * sin_t;
        double dy_local = -dx * sin_t + dy * cos_t;

        // 5. Anisotropic Mahalanobis
        r_norm_sq = (dx_local / sx) * (dx_local / sx)
                  + (dy_local / sy) * (dy_local / sy);
    }

    double core_risk = A * std::exp(-0.5 * r_norm_sq);

    // 6. Halo
    if (conf_.halo_scale > 0.0 && conf_.halo_ratio > 0.0) {
        double hs = conf_.halo_scale;
        double halo_risk = A * conf_.halo_ratio
                         * std::exp(-0.5 * r_norm_sq / (hs * hs));
        return std::max(core_risk, halo_risk);
    }

    return core_risk;
}

double DynamicRiskField::getHaloCostFast(double qx, double qy,
                                         double ox, double oy,
                                         double vx, double vy) const
{
    // Return halo component only. Zero when disabled.
    if (conf_.halo_scale <= 0.0 || conf_.halo_ratio <= 0.0)
        return 0.0;

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

    double r_norm_sq;

    // 3. Speed check
    double speed_sq = vx * vx + vy * vy;
    if (speed_sq < 1e-8) {
        r_norm_sq = dist_sq / (sy * sy);
    } else {
        // 4. Rotate into obstacle-local frame
        double theta = std::atan2(vy, vx);
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        double dx_local =  dx * cos_t + dy * sin_t;
        double dy_local = -dx * sin_t + dy * cos_t;

        r_norm_sq = (dx_local / sx) * (dx_local / sx)
                  + (dy_local / sy) * (dy_local / sy);
    }

    double hs = conf_.halo_scale;
    return A * conf_.halo_ratio * std::exp(-0.5 * r_norm_sq / (hs * hs));
}

} // namespace
