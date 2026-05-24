#ifndef _DYNAMIC_RISK_FIELD_H_
#define _DYNAMIC_RISK_FIELD_H_

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace cane_planner {

class DynamicRiskField {
public:
    struct Config {
        double tau = 1.0;          // 预测步长
        double A_risk = 5.0;       // 风险峰值系数
        double sigma_y = 0.22;     // 横向风险范围
        // double sigma_x = sigma_y + k*obs_vx;     //一种动态调整方案
        double sigma_x = 0.4;      // 纵向风险范围 (沿运动方向)
        double cutoff_dist = 3.0;  // 约3到4倍的sigma_x

        // Halo: wider low-amplitude envelope beyond the hard core.
        // halo_scale=0 disables it (pure Gaussian, backward compatible).
        // When enabled, risk = max(core_gaussian, halo_gaussian), where
        //   halo_gaussian = A * halo_ratio * exp(-0.5 * r² / halo_scale²)
        // This gives MPC a soft gradient to follow without forcing wide detours.
        double halo_scale = 0.0;   // width multiplier relative to core sigma
        double halo_ratio = 0.25;  // peak amplitude ratio relative to A

        // CPA/TTC: penalize predicted close encounters at the same future time.
        // This complements the obstacle-centered Gaussian field, especially for
        // perpendicular pedestrian crossings.
        bool cpa_enable = false;
        double cpa_weight = 0.6;        // peak ratio relative to A_risk
        double cpa_sigma_d = 0.65;      // distance scale for closest approach
        double cpa_tau = 1.0;           // time decay; smaller = more urgent only
        double cpa_time_horizon = 2.0;  // seconds ahead from the evaluated point
        double cpa_cutoff_dist = 3.0;   // skip if closest distance is farther
    };

    DynamicRiskField() {}
    DynamicRiskField(const Config& conf) : conf_(conf) {}

    /**
     * @brief 计算点 q 处由单个障碍物产生的动态代价
     * @param q_pos     待计算点 [x, y]
     * @param obs_p     障碍物当前位置 [x, y]
     * @param obs_v     障碍物当前速度 [vx, vy]
     */
    double getIndividualCost(const Eigen::Vector2d& q_pos,
                             const Eigen::Vector2d& obs_p,
                             const Eigen::Vector2d& obs_v) const;

    // Fast scalar version for MPPI inner loop (no Eigen allocation)
    double getIndividualCostFast(double qx, double qy,
                                 double ox, double oy,
                                 double vx, double vy) const;

    // Halo component only (for visualization). Returns 0 if halo disabled.
    double getHaloCostFast(double qx, double qy,
                           double ox, double oy,
                           double vx, double vy) const;

    // CPA/TTC conflict cost for robot point q with estimated velocity rv.
    // ox/oy must be the obstacle position at the same timestamp as q.
    double getTimeConflictCostFast(double qx, double qy,
                                   double rvx, double rvy,
                                   double ox, double oy,
                                   double vx, double vy,
                                   double safety_radius = 0.0) const;

    void setConfig(const Config& conf) { conf_ = conf; }
    const Config& getConfig() const { return conf_; }

private:
    Config conf_;
};

} // namespace 

#endif
