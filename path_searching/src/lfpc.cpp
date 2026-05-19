#include <path_searching/lfpc.h>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
    LFPC::LFPC()
    {
        // cycle init params
        x_0_ = 0.0;
        vx_0_ = 0.0;
        y_0_ = 0.0;
        vy_0_ = 0.0;
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;
        t_ = 0.0;
        // control param
        al_ = aw_ = theta_ = 0.0;
        b_ = 0.0;
        // path
        step_path_.clear();
    }
    void LFPC::initializeModel(ros::NodeHandle &nh)
    {
        nh.param("lfpc/delta_t", delta_t_, 0.1);
        nh.param("lfpc/t_sup", t_sup_, 0.35);
        nh.param("lfpc/h_", h_, 1.0);
        support_leg_ = LEFT_LEG;
        step_num_ = 0;
        // calculate
        t_c_ = sqrt(h_ / 10);//根号0.1,h_为腿长
        double CT = cosh(t_sup_ / t_c_);
        double ST = sinh(t_sup_ / t_c_);
        b_ = t_c_ * CT / ST;
        std::cout << "LFPC's first support leg is left leg" << std::endl;
        std::cout << "LFPC's contorl b is: " << b_ << std::endl;
    }
    LFPC::~LFPC()
    {
        step_path_.clear();
    }
    void LFPC::SetCtrlParams(Vector3d input)
    {
        al_ = input(0);
        aw_ = input(1);
        theta_ = theta_ + input(2);
        // std::cout << "al " << al_ << " aw_ " << aw_ << " theta_ " << theta_ << std::endl;
    }
    // param:
    // init_v_state : vx,vy,theta
    void LFPC::reset(Vector3d init_v_state, Vector3d COM_init_pos,
                     char cur_support_leg, int step_num)
    {
        COM_pos_ = COM_init_pos;
        COM_pos_(2) = -0.1;

        step_num_ = step_num;
        // change support_leg
        if (cur_support_leg == LEFT_LEG)
            support_leg_ = RIGHT_LEG;
        else if (cur_support_leg == RIGHT_LEG)
            support_leg_ = LEFT_LEG;
        // LPFC

        vx_0_ = init_v_state(0);
        vy_0_ = init_v_state(1);
        theta_ = init_v_state(2);

        // step variable
        t_ = 0;
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;

        // path clear;
        step_path_.clear();
    }
    void LFPC::updateOneStep()
    {
        // 在一个支撑相中需要计算的离散时间点数量
        // 如t_sup_ = 0.35s (支撑相持续时间), delta_t_ = 0.07s (离散步长)
        // swing_data_len = 0.35 / 0.07 = 5, 取整为 5 个时间点
        int swing_data_len = int(t_sup_ / delta_t_);
        
        // 使用当前的初始速度 vx_0_, vy_0_ 来计算下一步脚的着地位置
        // vx_0_, vy_0_ 是质心的全局速度（在本支撑相开始时的速度）
        auto state_f = calculateLFPC(vx_0_, vy_0_);//初始均为0，根据当前速度预测下一步脚的落点
        
        // 计算下一步支撑脚的全局位置
        // update step support_leg_pos
        support_leg_pos_(0) = COM_pos_(0) + state_f(0);//state_f表示当前步态下的足底目标偏移
        support_leg_pos_(1) = COM_pos_(1) + state_f(1);
        support_leg_pos_(2) = 0.0;
        
        // 计算在LIPM相对坐标系中的初始位置
        // x_0_ 和 y_0_ 是质心相对于支撑脚的初始位置（都是负值）
        // update step param;
        x_0_ = -state_f(0);
        y_0_ = -state_f(1);
        
        // 在支撑相期间，逐时间步长计算质心轨迹
        // 从COM_pos_出发，用x_0_和y_0_计算最终到support_leg_pos_的轨迹
        for (int i = 0; i < swing_data_len; i++)
        {
            updateOneDt();//按 LIPM 方程推进质心的运动
            step_path_.push_back(COM_pos_);//保存本步质心轨迹
        }
        // 更新下一个支撑相的初始速度：当前支撑相结束时的速度
        // 这样确保LIPM轨迹的连续性和动态性（速度会影响下一步的落脚位置）
        vx_0_ = vx_t_;
        vy_0_ = vy_t_;
        step_num_ += 1;
    }

    void LFPC::updateOneStepForOnce(double t)
    {     
        auto state_f = calculateLFPC(vx_0_, vy_0_);
        
        support_leg_pos_(0) = COM_pos_(0) + state_f(0);
        support_leg_pos_(1) = COM_pos_(1) + state_f(1);
        support_leg_pos_(2) = 0.0;

        x_0_ = -state_f(0);
        y_0_ = -state_f(1);

        updateOneDtForOnce(t);
        // step_path_.push_back(COM_pos_);

        vx_0_ = vx_t_;
        vy_0_ = vy_t_;
        step_num_ += 1;
    }

    // -------------------------------------API function------------------------------------//
    Vector2d LFPC::getFootPosition()
    {
        Vector2d support_leg_pos_2d_;
        support_leg_pos_2d_ << support_leg_pos_(0), support_leg_pos_(1);
        return support_leg_pos_2d_;
    }
    Vector3d LFPC::getCOMPos()
    {
        return COM_pos_;
    }
    char LFPC::getSupportFeet()
    {
        return support_leg_;
    }
    int LFPC::getStepNum()
    {
        return step_num_;
    }
    std::vector<Eigen::Vector3d> LFPC::getStepCOMPath()
    {
        return step_path_;
    }
    // retrun vx_t,vy_t,theta_
    Vector3d LFPC::getNextIterState()
    {
        Vector3d next_iter_state;
        next_iter_state(0) = vx_t_;
        next_iter_state(1) = vy_t_;
        next_iter_state(2) = theta_;
        return next_iter_state;
    }
    // -------------------------------------private function------------------------------------//
    void LFPC::updateOneDt()
    {
        t_ += delta_t_;//一直累加...直到下轮reset重置为0
        Vector4d iter_state = calculateXtVt(t_);
        x_t_ = iter_state(0);
        vx_t_ = iter_state(1);
        y_t_ = iter_state(2);
        vy_t_ = iter_state(3);

        COM_pos_(0) = x_t_ + support_leg_pos_(0);
        COM_pos_(1) = y_t_ + support_leg_pos_(1);
        COM_pos_(2) = collision_ ? collision_->getSliceHeight() : 0.0;
    }

    void LFPC::updateOneDtForOnce(double t)
    {
        t_ = t;
        Vector4d iter_state = calculateXtVt(t_);
        x_t_ = iter_state(0);
        vx_t_ = iter_state(1);
        y_t_ = iter_state(2);
        vy_t_ = iter_state(3);

        COM_pos_(0) = x_t_ + support_leg_pos_(0);
        COM_pos_(1) = y_t_ + support_leg_pos_(1);
        COM_pos_(2) = collision_ ? collision_->getSliceHeight() : 0.0;
    }

    Vector4d LFPC::calculateXtVt(double t)
    {
        // in here,  iter_state  == [x_t,vx_t,y_t,vy_t]"
        Vector4d iter_state;
        // linear inverted pendulum motion low
        double tau = t / t_c_;//t_c_ = 根号0.1
        // x
        iter_state(0) = x_0_ * cosh(tau) + t_c_ * vx_0_ * sinh(tau);//x_0_ = -state_f(0)，为负的足底目标偏移量
        iter_state(1) = x_0_ * sinh(tau) / t_c_ + vx_0_ * cosh(tau);//y_0_ = -state_f(1)
        // y
        iter_state(2) = y_0_ * cosh(tau) + t_c_ * vy_0_ * sinh(tau);
        iter_state(3) = y_0_ * sinh(tau) / t_c_ + vy_0_ * cosh(tau);
        return iter_state;
    }
    Vector4d LFPC::calculateFinalState()
    {
        Vector4d final_state;
        final_state = calculateXtVt(t_sup_);
        // std::cout << "final_state " << final_state.transpose() << std::endl;

        return final_state;
    }
    Vector2d LFPC::calculateLFPC(double vx, double vy)
    {
        Vector2d state_f;
        // Linear Foot Placement Control (LFPC)
        // 根据当前的质心速度(vx, vy)来计算下一步脚的目标落脚位置相对于当前质心的偏移量
        // 
        // 关键参数：
        //   al_: 步长参数（来自规划器的输入）
        //   aw_: 步宽参数（来自规划器的输入）
        //   theta_: 当前躯干方向（累积的偏航角）
        //   b_: LIPM稳定性系数 = t_c_ * cosh(t_sup_/t_c_) / sinh(t_sup_/t_c_)
        //       其中 t_c_ = sqrt(h/10)，h为腿长(1.0m)，t_sup_为支撑相时间(0.35s)
        //       b_ ≈ 0.3 (具体值由robots dynamics决定)
        //   vx, vy: 当前质心速度（来自上一步的末速度）
        //
        // 输出含义：
        //   state_f: 下一步脚的目标位置相对于当前质心COM_pos的偏移
        //           state_f = support_leg_pos - COM_pos
        //
        // 重要观察：即使al=0, aw=0，只要有速度输入(vx或vy非零)，就会有运动输出！
        // 这是因为 state_f 包含 b_*vx 和 b_*vy 项，反映了"为了维持平衡，脚需要提前着地"的物理原理
        
        if (support_leg_ == LEFT_LEG)
        {
            // 左腿支撑：脚在身体右侧，步宽为正时向右偏移
            state_f(0) = -al_ * cos(theta_) + aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -al_ * sin(theta_) - aw_ * cos(theta_) + b_ * vy;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            // 右腿支撑：脚在身体左侧，步宽为负时向左偏移
            state_f(0) = -al_ * cos(theta_) - aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -al_ * sin(theta_) + aw_ * cos(theta_) + b_ * vy;
        }
        // std::cout << "LFPC set:" << state_f.transpose() << std::endl;
        return state_f;
    }

    void LFPC::prepareNextStep()
    {
        // Flip support leg
        support_leg_ = (support_leg_ == LEFT_LEG) ? RIGHT_LEG : LEFT_LEG;
        // Reset step-internal LIPM state
        t_ = 0.0;
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;
        step_path_.clear();
    }

    void LFPC::copyState(const LFPC& other)
    {
        support_leg_ = other.support_leg_;
        t_sup_ = other.t_sup_;
        delta_t_ = other.delta_t_;
        h_ = other.h_;
        t_c_ = other.t_c_;
        step_num_ = other.step_num_;
        collision_ = other.collision_;
        x_0_ = other.x_0_;
        vx_0_ = other.vx_0_;
        y_0_ = other.y_0_;
        vy_0_ = other.vy_0_;
        t_ = other.t_;
        x_t_ = other.x_t_;
        vx_t_ = other.vx_t_;
        y_t_ = other.y_t_;
        vy_t_ = other.vy_t_;
        al_ = other.al_;
        aw_ = other.aw_;
        theta_ = other.theta_;
        b_ = other.b_;
        support_leg_pos_ = other.support_leg_pos_;
        COM_pos_ = other.COM_pos_;
        step_path_ = other.step_path_;
    }

    double LFPC::getTimeUpdate(){
        return this->t_sup_;
    }

    Eigen::Matrix<double, 6, 1> LFPC::getState(){
        Eigen::Matrix<double, 6, 1> state = Eigen::Matrix<double, 6, 1>::Zero();
        state(0) = vx_0_;
        state(1) = vy_0_;
        state(2) = al_;
        state(3) = aw_;
        state(4) = theta_;
        return state;
    }

} // namespace cane_planner
