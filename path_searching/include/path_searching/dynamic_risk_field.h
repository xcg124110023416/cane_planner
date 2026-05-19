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

    void setConfig(const Config& conf) { conf_ = conf; }
    Config getConfig() const {return conf_; }

private:
    Config conf_;
};

} // namespace 

#endif