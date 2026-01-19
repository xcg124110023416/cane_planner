#include <path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
    KinodynamicAstar::~KinodynamicAstar()
    {
        for (int i = 0; i < allocate_num_; i++)
        {
            delete path_node_pool_[i];
        }
    }

    bool KinodynamicAstar::search(Eigen::Vector3d start_pos, Eigen::Vector3d start_state,
                                  Eigen::Vector3d end_pos)
    {
        /* ---------- initialize --------*/
        // Bidirectional Attempt: Add both Left and Right support legs to OpenSet
        
        // 1. Left Leg Support Node
        KdNodePtr node_left = path_node_pool_[0];
        node_left->parent = NULL;
        node_left->iter_state = start_state;
        node_left->support_feet = LEFT_LEG;
        
        node_left->support_pos << start_pos(0), start_pos(1);
        start_pos(2) = 0.0;
        node_left->com_pos << start_pos;
        node_left->index = stateToIndex(start_pos);
        node_left->g_score = 0.0;
        node_left->step_num = 0;
        node_left->f_score = lambda_heu_ * getDiagHeu(start_pos, end_pos);
        node_left->kdnode_state = IN_OPEN_SET;
        
        open_set_.push(node_left);
        expanded_nodes_.insert(node_left->index, node_left);

        // 2. Right Leg Support Node
        KdNodePtr node_right = path_node_pool_[1];
        node_right->parent = NULL;
        node_right->iter_state = start_state;
        node_right->support_feet = RIGHT_LEG;
        
        node_right->support_pos << start_pos(0), start_pos(1);
        node_right->com_pos << start_pos;
        node_right->index = stateToIndex(start_pos); // Same index
        node_right->g_score = 0.0;
        node_right->step_num = 0;
        node_right->f_score = lambda_heu_ * getDiagHeu(start_pos, end_pos);
        node_right->kdnode_state = IN_OPEN_SET;
        
        open_set_.push(node_right);
        // Note: We don't insert node_right into expanded_nodes_ because it has the same index.
        
        use_node_num_ = 2; // Used 2 nodes

        // end set
        Eigen::Vector2i end_index;
        end_index = stateToIndex(end_pos);
        // TODO(1): Heuristic compute
        // cur_node->f_score = lambda_heu_ * getDiagHeu(start_pos, end_pos);
        // cur_node->kdnode_state = IN_OPEN_SET;

        // open_set_.push(cur_node);
        // use_node_num_ += 1;

        // expanded_nodes_.insert(cur_node->index, cur_node);

        KdNodePtr terminate_node = NULL;

        // num of can't find path
        // int num_feasible = 0, num_close = 0, num_collision = 0, num_outedf = 0;
        int num_feasible = 0, num_close = 0, num_collision = 0;

        /* ---------- search loop ---------- */
        while (!open_set_.empty())
        {
            KdNodePtr cur_node = open_set_.top();
            // std::cout << "Explore while is " << use_node_num_ << std::endl;
            // std::cout << "----------------------------" << std::endl;

            /* ---------- determine termination ---------- */
            // bool near_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
            //                 abs(cur_node->index(1) - end_index(1)) <= 1;
            // abs(cur_node->state_variable(2) - end_pos(2)) <= 0.1;
            // have tourble in here;
            bool near_end = abs(cur_node->com_pos(0) - end_pos(0)) <= 0.5 &&
                            abs(cur_node->com_pos(1) - end_pos(1)) <= 0.5;

            double reach_horizon = (cur_node->com_pos.head(3) - start_pos).norm();

            if (near_end)
            {
                // std::cout << "[Kin-Astar]:---------------------- " << use_node_num_ << std::endl;
                // std::cout << "use node num: " << use_node_num_ << std::endl;
                // std::cout << "iter num: " << iter_num_ << std::endl;
                // std::cout << "feasible num: " << num_feasible << std::endl;
                // std::cout << "close num: " << num_close << std::endl;
                // std::cout << "collision num: " << num_collision << std::endl;
                std::cout << use_node_num_ << "," << iter_num_ << ",";

                terminate_node = cur_node;
                retrievePath(terminate_node);
                return true;
            }
            if (reach_horizon >= horizon_)
            {
                double cur_near_end = (cur_node->com_pos.head(3) - end_pos).norm();
                double start_end = (start_pos - end_pos).norm();
                if (cur_near_end <= start_end)
                {
                    std::cout << "[Kin-Astar](horizon):---------------------- " << use_node_num_ << std::endl;
                    // std::cout << "use node num: " << use_node_num_ << std::endl;
                    // std::cout << "iter num: " << iter_num_ << std::endl;
                    // std::cout << "feasible num: " << num_feasible << std::endl;
                    // std::cout << "close num: " << num_close << std::endl;
                    // std::cout << "collision num: " << num_collision << std::endl;
                    std::cout << use_node_num_ << "," << iter_num_ << ",";
                    terminate_node = cur_node;
                    retrievePath(terminate_node);
                    return true;
                }
                else
                {
                    std::cout << "[Kin-Astar](horizon):---------------------- " << use_node_num_ << std::endl;
                    std::cout << "!---in horizion no find path--" << std::endl;
                }
            }

            /* ---------- pop node and add to close set ---------- */
            open_set_.pop();
            cur_node->kdnode_state = IN_CLOSE_SET;
            iter_num_ += 1;

            KdNode pur_state;
            vector<KdNodePtr> tmp_expand_nodes;
            vector<Eigen::Vector3d> inputs;
            Eigen::Vector3d um;

            /* ---------- param for next gait point ---------- */
            double al_res = 1 / 2.0, aw_res = 1 / 1.0, pi_res = 1 / 3.0;
            /* ----------set input list ---------- */
            for (double al = max_al_ - 0.1; al < max_al_ + 1e-3; al += al_res * 0.1)//可能表示步态点的某个线性参数（如步长）
                for (double aw = max_aw_ * aw_res; aw < max_aw_ + 1e-3; aw += max_aw_ * aw_res)//可能表示步态点的宽度或横向偏移
                    for (double api = -max_api_; api < max_api_ + 1e-2; api += max_api_ * pi_res)//可能表示步态点的角度或方向偏移
                    {
                        um << al, aw, api;
                        inputs.push_back(um);
                    }//使用 1e-3 和 1e-2 作为循环终止条件的偏移量,这样处理可以避免浮点数精度问题导致的循环边界错误

            /* ----------Explore the next gait point ---------- */
            // std::cout << "set input list" << std::endl;
            // std::cout << "new state explore,size: " << inputs.size() << std::endl;
            for (size_t i = 0; i < inputs.size(); i++)
            {
                // state transit,explore the next gait point.
                um = inputs[i];
                lfpc_model_->reset(cur_node->iter_state, cur_node->com_pos,
                                   cur_node->support_feet, cur_node->step_num);
                lfpc_model_->SetCtrlParams(um);
                lfpc_model_->updateOneStep();
                pur_state.com_pos = lfpc_model_->getCOMPos();//得到的是pur_state的com_pos_,即质心位置的轨迹点
                pur_state.com_path = lfpc_model_->getStepCOMPath();//由多个COM_pos_组成的容器
                pur_state.support_pos = lfpc_model_->getFootPosition();//得到的是pur_state的support_leg_pos_，即支撑腿位置的轨迹点

                // std::cout << "\ninput:sx,sy,yaw" << um.transpose() << std::endl;
                // std::cout << "pur_state: " << pur_state.com_pos.transpose() << std::endl;
                // std::cout << "pur_support_pos" << pur_state.support_pos.transpose() << std::endl;

                Eigen::Vector3d pro_state;
                pro_state << pur_state.com_pos;
                Eigen::Vector2i pro_id = stateToIndex(pro_state);

                // check if in feasible space
                if (pur_state.support_pos(0) <= origin_(0) || pur_state.support_pos(0) >= map_size_2d_(0) || pur_state.support_pos(1) <= origin_(1) || pur_state.support_pos(1) >= map_size_2d_(1))
                {
                    // std::cout << "outside map" << std::endl;
                    num_feasible++;
                    continue;
                }

                // Check if in close set
                KdNodePtr pro_node = expanded_nodes_.find(pro_id);
                if (pro_node != NULL && pro_node->kdnode_state == IN_CLOSE_SET)
                {
                    // std::cout << "in close,num:" << num_close << std::endl;
                    num_close++;
                    continue;
                }

                // Check com and feet safety
                // collision feet pos free
                Eigen::Vector3d pro_pos;
                bool safe_flag = true;
                // pro_pos << pur_state.support_pos(0), pur_state.support_pos(1), -0.4;
                // if (collision_->sdf_map_->getInflateOccupancy(pro_pos) == 1)
                // {
                //     safe_flag = false;
                //     break;
                // }
                // if (!safe_flag)
                // {
                //     // std::cout << "can't Traversable" << std::endl;
                //     num_collision++;
                //     continue;
                // }
                // collision com pos free
                for (size_t i = 0; i < pur_state.com_path.size(); i++)
                {
                    pro_pos << pur_state.com_path[i];
                    // if (collision_->sdf_map_->getInflateOccupancy(pro_pos) == 1)
                    if (!collision_->isTraversable(pro_pos(0), pro_pos(1)))
                    {
                        safe_flag = false;
                        break;
                    }
                }
                if (!safe_flag)
                {
                    // std::cout << "can't Traversable" << std::endl;
                    num_collision++;
                    continue;
                }
                // // out edf map by expert
                // if (collision_->getCollisionDistance(pro_pos) >= 6.0)
                // {
                //     num_outedf++;
                //     continue;
                // }

                double tmp_g_score = cur_node->g_score + estimateHeuristic(um, pur_state.com_pos, cur_node->com_pos);
                double tmp_f_score = tmp_g_score + lambda_heu_ * getDiagHeu(pur_state.com_pos, end_pos);
                
                // ==================== 添加动态障碍物代价到启发式函数 ====================
                // 动态障碍物信息通过 setDynamicObstacles() 从 planner_manager 传入
                // double dynObsCost = getDynamicObstacleCost(
                //     pur_state.com_pos,
                //     dynObstaclesPos_,      // 成员变量：动态障碍物位置向量
                //     dynObstaclesVel_,      // 成员变量：动态障碍物速度向量
                //     dynObstaclesSize_,     // 成员变量：动态障碍物大小向量
                //     predHorizon_,          // 预测地平线
                //     ts_,                   // 时间采样间隔
                //     distThreshDynamic_     // 距离阈值
                // );
                // tmp_f_score += weight_dyn_obs_ * dynObsCost;  // 加权合并到总代价
                // ===================================================================
                if (pro_node == NULL)
                {
                    // std::cout << "find new pro_node" << std::endl;
                    pro_node = path_node_pool_[use_node_num_];
                    pro_node->index = pro_id;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->parent = cur_node;
                    pro_node->kdnode_state = IN_OPEN_SET;
                    // update  state
                    pro_node->time_update_once = lfpc_model_->getTimeUpdate();
                    pro_node->input = um;
                    // pro_node->state = lfpc_model_->getState();
                    pro_node->iter_state = lfpc_model_->getNextIterState();
                    pro_node->com_pos = lfpc_model_->getCOMPos();
                    pro_node->support_feet = lfpc_model_->getSupportFeet();
                    pro_node->support_pos = lfpc_model_->getFootPosition();
                    pro_node->step_num = lfpc_model_->getStepNum();
                    pro_node->com_path = lfpc_model_->getStepCOMPath();
                    // push in set
                    open_set_.push(pro_node);
                    expanded_nodes_.insert(pro_id, pro_node);
                    // add used node num
                    use_node_num_ += 1;
                    // std::cout << "---------------" << std::endl;
                    if (use_node_num_ == allocate_num_)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return false;
                    }
                }
                else if (pro_node->kdnode_state == IN_OPEN_SET)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        // std::cout << "update new_node" << std::endl;
                        pro_node->time_update_once = lfpc_model_->getTimeUpdate();
                        // pro_node->state = lfpc_model_->getState();
                        pro_node->iter_state = lfpc_model_->getNextIterState();
                        pro_node->com_pos = lfpc_model_->getCOMPos();
                        pro_node->support_feet = lfpc_model_->getSupportFeet();
                        pro_node->support_pos = lfpc_model_->getFootPosition();
                        pro_node->step_num = lfpc_model_->getStepNum();
                        pro_node->com_path = lfpc_model_->getStepCOMPath();

                        // update score
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->parent = cur_node;
                        // std::cout << "f_score:" << tmp_f_score << " g_score:" << tmp_g_score << std::endl;
                    }
                }
                else
                {
                    std::cout << "error type in searching: " << pro_node->kdnode_state << std::endl;
                }
            }
        }

        /* ---------- open set empty, no path ---------- */
        std::cout << "[Kin-Astar]:---------------------- " << std::endl;
        std::cout << "open set empty, no path!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        std::cout << "feasible num: " << num_feasible << std::endl;
        std::cout << "close num: " << num_close << std::endl;
        std::cout << "collision num: " << num_collision << std::endl;

        return false;
    }

    void KinodynamicAstar::stateTransit(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                                        Eigen::Vector3d input, int n)
    {
        double yaw_new = state1(2) + input(2);
        if (yaw_new > M_PI)
        {
            yaw_new -= M_PI;
        }
        else if (yaw_new < -M_PI)
        {
            yaw_new += M_PI;
        }

        state2(0) = state1(0) + cos(yaw_new) * input(0) - sin(yaw_new) * input(1);
        state2(1) = state1(1) + sin(yaw_new) * input(0) - pow(-1, n) * cos(yaw_new) * input(1);
        state2(2) = yaw_new;

        // TODO using LFPC to next step location

        // std::cout << "new px:" << state2(0) << std::endl;
        // std::cout << "new py:" << state2(1) << std::endl;
        // std::cout << "new yaw:" << state2(2) << std::endl;
    }

    Eigen::Vector2i KinodynamicAstar::posToIndex(Eigen::Vector2d pt)
    {
        Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
        return idx;
    }
    Eigen::Vector2d KinodynamicAstar::stateToPos(Eigen::Vector3d state)
    {
        Vector2d pos;
        pos << state(0), state(1);
        return pos;
    }
    Eigen::Vector2i KinodynamicAstar::stateToIndex(Eigen::Vector3d state)
    {
        auto pos = stateToPos(state);
        Vector2i idx = posToIndex(pos);
        return idx;
    }

    void KinodynamicAstar::retrievePath(KdNodePtr end_node)
    {
        KdNodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);
        while (cur_node->parent != NULL)
        {
            cur_node = cur_node->parent;
            path_nodes_.push_back(cur_node);
        }
        // reverse path form begin to end;
        reverse(path_nodes_.begin(), path_nodes_.end());
    }

    std::vector<Eigen::Vector3d> KinodynamicAstar::getPath()
    {
        vector<Eigen::Vector3d> path;
        for (size_t i = 0; i < path_nodes_.size(); i++)
        {
            for (size_t j = 0; j < path_nodes_[i]->com_path.size(); j++)
            {
                path.push_back(path_nodes_[i]->com_path[j]);
            }
        }
        return path;
    }

    std::vector<Eigen::Vector3d> KinodynamicAstar::getComPos()
    {
        vector<Eigen::Vector3d> path;
        for (size_t i = 0; i < path_nodes_.size(); i++)
        {
            Eigen::Vector3d pos;
            pos << path_nodes_[i]->com_pos;
            path.push_back(pos);
        }
        return path;
    }
    std::vector<Eigen::Vector3d> KinodynamicAstar::getFeetPos()
    {
        vector<Eigen::Vector3d> path;
        for (size_t i = 0; i < path_nodes_.size(); i++)
        {
            Eigen::Vector3d pos;
            pos << path_nodes_[i]->support_pos, 0.0;
            path.push_back(pos);
        }
        return path;
    }

    void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
    {
        /* ---------- path duration ---------- */
        double T_sum = 0.0;
        KdNodePtr node = path_nodes_.back();
        while (node->parent != NULL)
        {
            T_sum += node->time_update_once;
            node = node->parent;
        }
        cout << "T_sum:" << T_sum << endl;

        // Calculate boundary vel and acc
        Eigen::Vector3d end_vel, end_acc;
        double t;
        
        t = path_nodes_.back()->time_update_once;
        // cout << "initial t: " << t << endl;
        end_vel = node->state.head(3);
        end_vel(3) = 0;
        // end_acc = path_nodes_.back()->input;

        // Get point samples
        int seg_num = floor(T_sum / ts);
        seg_num = max(12, seg_num);
        ts = T_sum / double(seg_num);
        // cout << "revised ts: " << ts << ", seg num: " << seg_num << endl;

        node = path_nodes_.back();

        for (double ti = T_sum; ti > -1e-5; ti -= ts)
        {
            // samples on searched traj
            Eigen::Vector3d um = node->input;
            Eigen::Vector3d iter_state = node->parent->iter_state;
            Eigen::Vector3d com_pos = node->parent->com_pos;
            char support_feet = node->parent->support_feet;
            int step_num = node->parent->step_num;

            lfpc_model_->reset(iter_state, com_pos, support_feet, step_num);
            lfpc_model_->SetCtrlParams(um);
            lfpc_model_->updateOneStepForOnce(t);
            
            point_set.push_back(lfpc_model_->getCOMPos());
            // cout<< "ti: " << ti << ", t: " << t << ", com_pos: " << lfpc_model_->getCOMPos().transpose() << endl;
            t -= ts;

            lfpc_model_->setThetaZero();
            // cout << "t: " << t << endl;
            if (t < -1e-5)  // ← 改为无条件检查
            {
                if (node->parent->parent != NULL)
                {
                    node = node->parent;
                    t += node->time_update_once;
                    // cout<< "parent node, new t: " << node->time_update_once << endl;
                }
                else
                {
                    ROS_INFO("Kinodynamic Astar get samples finished.");
                    break;  
                }
            }
        }
        reverse(point_set.begin(), point_set.end());


        start_end_derivatives.push_back({0,0,0});
        start_end_derivatives.push_back(end_vel);
        start_end_derivatives.push_back({0,0,0});
        start_end_derivatives.push_back({0,0,0});
    }

    void KinodynamicAstar::setParam(ros::NodeHandle &nh)
    {
        // 用于放大f_score的一个倍速
        nh.param("kinastar/lambda_heu", lambda_heu_, -1.0);
        // 这里的加上时间维度
        nh.param("kinastar/resolution_astar", resolution_, -1.0);
        inv_resolution_ = ceil(1 / resolution_);
        // 分配的最大可以搜索的数量；
        nh.param("kinastar/allocate_num", allocate_num_, -1);
        // 分配规划的区域限制
        nh.param("kinastar/horizon", horizon_, -1.0);
        // 人体动力学限制参数
        nh.param("kinastar/max_al", max_al_, -1.0);//步长
        nh.param("kinastar/max_aw", max_aw_, -1.0);//步宽
        nh.param("kinastar/max_theta", max_api_, -1.0);//最大旋转弧度制
        // 动态障碍物相关参数
        nh.param("kinastar/weight_dyn_obs", weight_dyn_obs_, -1.0);
        nh.param("kinastar/pred_horizon", predHorizon_, -3.0);
        nh.param("kinastar/ts", ts_, -0.5);
        nh.param("kinastar/distThresh_Dynamic", distThreshDynamic_, -1.0);

        nh.param("kinastar/ts_sample", tsSample_, -0.1);

        tie_breaker_ = 1.0 + 1.0 / 10000;

        kin_SamplePath_pub_ = nh.advertise<nav_msgs::Path>("/kin_astar/SamplePath", 20);
    }
    void KinodynamicAstar::init()
    {
        /* ---------- pre-allocated node ---------- */
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++)
        {
            path_node_pool_[i] = new KdNode;
        }

        use_node_num_ = 0;
        iter_num_ = 0;

        /* ---------- map params ---------- */
        this->inv_resolution_ = 1.0 / resolution_;
        Eigen::Vector3d ori, map_size_3d;
        collision_->getMapRegion(ori, map_size_3d);
        origin_ << ori(0), ori(1);
        map_size_2d_ << map_size_3d(0), map_size_3d(1);

        std::cout << "origin_: " << origin_.transpose() << std::endl;
        std::cout << "map size: " << map_size_2d_.transpose() << std::endl;

        /* ----------lfpc model params ---------- */
        // lfpc_model_->
    }
    void KinodynamicAstar::reset()
    {
        expanded_nodes_.clear();
        path_nodes_.clear();
        std::priority_queue<KdNodePtr, std::vector<KdNodePtr>, KdNodeComparator> empty_queue;
        open_set_.swap(empty_queue);

        for (int i = 0; i < use_node_num_; i++)
        {
            KdNodePtr node = path_node_pool_[i];
            node->parent = NULL;
            node->kdnode_state = NOT_EXPAND;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
    }

    void KinodynamicAstar::setCollision(const CollisionDetection::Ptr &col)
    {
        this->collision_ = col;
    }

    void KinodynamicAstar::setModel(const LFPC::Ptr &col)
    {
        this->lfpc_model_ = col;
    }

    void KinodynamicAstar::setDynamicObstacles(const std::vector<Eigen::Vector3d>& pos,
                                               const std::vector<Eigen::Vector3d>& vel,
                                               const std::vector<Eigen::Vector3d>& size)
    {
        this->dynObstaclesPos_ = pos;
        this->dynObstaclesVel_ = vel;
        this->dynObstaclesSize_ = size;
    }

    double KinodynamicAstar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        double h = (dx + dy) + (sqrt(2.0) - 1) * min(dx, dy);

        // std::cout << "f heu is : " << tie_breaker_ * h << std::endl;

        return tie_breaker_ * h;
    }

    double KinodynamicAstar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        // double dz = fabs(x1(2) - x2(2));

        return tie_breaker_ * (dx + dy);
    }

    double KinodynamicAstar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
    {
        auto pos1 = stateToPos(x1);
        auto pos2 = stateToPos(x2);
        return tie_breaker_ * (pos2 - pos1).norm();
    }

    double KinodynamicAstar::estimateHeuristic(Eigen::Vector3d input)
    {
        Eigen::Vector2d acc_x_y;
        acc_x_y << input(0), input(1);
        // double heu = acc_x_y.norm() + input(2);
        double heu = 0.1 * abs(input(2));
        // std::cout << "this heu is " << heu << std::endl;
        return heu;
    }

    double KinodynamicAstar::estimateHeuristic(Eigen::Vector3d input, Eigen::Vector3d state1, Eigen::Vector3d state2)
    {
        // Eigen::Vector2d acc_x_y;
        // acc_x_y << input(0), input(1);
        // double dx = fabs(state1(0) - state2(0));
        // double dy = fabs(state1(1) - state2(1));
        // double heu = dx + dy + 0.5 * abs(input(2));
        double heu = (state1 - state2).norm() + 0.5 * abs(input(2));
        // std::cout << "this heu is " << heu << std::endl;

        return heu;
    }

    /* ==================== Dynamic Obstacle Cost Function for Front-End ====================
     * 这是从后端优化（B样条）移植到前端规划的动态障碍物代价函数
     * 用于在A*搜索中评估节点与动态障碍物的冲突风险
     * 
     * 输入参数：
     *   pos: 当前节点位置 (3D)
     *   dynObsPos: 所有动态障碍物的当前位置
     *   dynObsVel: 所有动态障碍物的速度
     *   dynObsSize: 所有动态障碍物的大小 (宽, 高, 深)
     *   predHorizon: 预测地平线（秒）
     *   ts: 时间采样间隔（秒）
     *   distThresh: 距离阈值（米），用于定义"危险区域"
     * 
     * 返回值：
     *   cost: 标量代价值（越小越安全）
     * ================================================================================== */
    double KinodynamicAstar::getDynamicObstacleCost(
        const Eigen::Vector3d& pos,
        const std::vector<Eigen::Vector3d>& dynObsPos,
        const std::vector<Eigen::Vector3d>& dynObsVel,
        const std::vector<Eigen::Vector3d>& dynObsSize,
        double predHorizon,
        double ts,
        double distThreshDynamic)
    {
        double cost = 0.0;
        
        // 如果没有动态障碍物，直接返回零代价
        if (dynObsPos.size() == 0) return cost;
        
        // 确保输入数据大小一致
        if (dynObsPos.size() != dynObsVel.size() || dynObsPos.size() != dynObsSize.size()) {
            ROS_WARN("[KinodynamicAstar] Dynamic obstacle data size mismatch!");
            return cost;
        }
        
        int predictionNum = static_cast<int>(predHorizon / ts);
        const int skipFactor = 1;  // 可调整：采样密度（1=每个时间步，2=每两个时间步...）
        
        // 遍历所有动态障碍物
        for (size_t j = 0; j < dynObsPos.size(); ++j) {
            // 计算障碍物的有效半径（投影到XY平面）
            double obsRadius = std::sqrt(
                std::pow(dynObsSize[j](0) / 2.0, 2) + 
                std::pow(dynObsSize[j](1) / 2.0, 2)
            );
            
            Eigen::Vector3d obstacleVel = dynObsVel[j];
            
            // 在预测地平线内，按时间间隔采样，评估节点与障碍物的接近程度
            for (int n = 0; n <= predictionNum; n += skipFactor) {
                // 预测障碍物在时刻 t = n*ts 的位置
                Eigen::Vector3d predictedObsPos = dynObsPos[j] + static_cast<double>(n * ts) * obstacleVel;
                // linearly decrease to half
                double distThresh = (1 - double(n/predictionNum) * 0.2) * distThreshDynamic; 
                // 计算节点与预测障碍物位置的差异（忽略Z轴）
                Eigen::Vector3d diff = pos - predictedObsPos;
                diff(2) = 0.0;  // 忽略Z轴差异
                
                // 计算实际距离（减去障碍物本身大小）
                double dist = diff.norm() - obsRadius;
                
                // 距离偏差：如果dist < distThresh，说明危险
                double distErr = distThresh - dist;
                
                // 根据接近程度分段计算代价
                // 这个设计来自于B样条的后端优化，用三次多项式进行平滑过度
                if (distErr <= 0) {
                    // 距离足够远，无惩罚
                    // cost += 0;
                } 
                else if (distErr > 0 && distErr <= distThresh) {
                    // 在危险区域内，使用三次惩罚
                    // 这样距离越近，惩罚越大（三次增长）
                    cost += std::pow(distErr, 3);
                } 
                else if (distErr >= distThresh) {
                    // 非常接近，使用更强的二次惩罚
                    // a, b, c 是为了保证平滑过渡的系数
                    double a = 3.0 * distThreshDynamic;
                    double b = -3.0 * std::pow(distThreshDynamic, 2);
                    double c = std::pow(distThreshDynamic, 3);
                    cost += (a * std::pow(distErr, 2) + b * distErr + c);
                }
            }
        }
        
        return cost;
    }

    void KinodynamicAstar::publishKinodynamicAstarPath(const vector<Eigen::Vector3d>& path)
    {
        vector<Eigen::Vector3d> list;
        list = path;
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();
        for (size_t i = 0; i < list.size(); i++)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = list[i](0);
            this_pose_stamped.pose.position.y = list[i](1);
            this_pose_stamped.pose.position.z = collision_->getSliceHeight();
            this_pose_stamped.pose.orientation.x = 0.0;
            this_pose_stamped.pose.orientation.y = 0.0;
            this_pose_stamped.pose.orientation.z = 0.0;
            this_pose_stamped.pose.orientation.w = 1.0;
            this_pose_stamped.header.frame_id = "world";
            this_pose_stamped.header.stamp = ros::Time::now();
            path_msg.poses.push_back(this_pose_stamped);
        }
        kin_SamplePath_pub_.publish(path_msg);
    }

} // namespace cane_planner
