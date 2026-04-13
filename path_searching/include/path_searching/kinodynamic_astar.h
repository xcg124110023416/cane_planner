#ifndef _KINODYNAMIC_ASTAR_H_
#define _KINODYNAMIC_ASTAR_H_

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include <ros/console.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <path_searching/matrix_hash.h>
#include <plan_env/collision_detection.h>
#include <path_searching/lfpc.h>
#include <path_searching/dynamic_risk_field.h>
// #include <plan_env/edt_environment.h>

using namespace std;

namespace cane_planner
{

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

  // KdKdNode 中需要保存的数据
  class KdNode
  {
  public:
    /* -------------------- */
    // node's index(from px,py)
    Eigen::Vector3i index;
    
    // lfpc iter param
    Eigen::Vector3d com_pos;
    //  iter_state variable: vx0，vy0,theta
    double time_update_once;
    Eigen::Matrix<double, 6, 1> state;
    Eigen::Vector3d input;

    Eigen::Vector3d  iter_state;
    char support_feet;
    Eigen::Vector2d support_pos;
    std::vector<Eigen::Vector3d> com_path;
    int step_num;
    // astar param
    double g_score, f_score;
    KdNode *parent;
    char kdnode_state;

    /* -------------------- */
    KdNode()
    {
      parent = NULL;
      kdnode_state = NOT_EXPAND;
    }
    ~KdNode(){};
  };
  typedef KdNode *KdNodePtr;

  class KdNodeComparator
  {
  public:
    bool operator()(KdNodePtr KdNode1, KdNodePtr KdNode2)
    {
      return KdNode1->f_score > KdNode2->f_score;
    }
  };

  class KdNodeHashTable
  {
  private:
      /* data */
      // data_2d_ 可以保留，用于兼容普通的 2D 搜索
      std::unordered_map<Eigen::Vector2i, KdNodePtr, matrix_hash<Eigen::Vector2i>> data_2d_;
      
      // data_3d_ 现在用于存储 [x_idx, y_idx, leg_idx] 的完整状态
      std::unordered_map<Eigen::Vector3i, KdNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;

  public:
      KdNodeHashTable() {}
      ~KdNodeHashTable() { clear(); }

      // --- 2D 接口 (保持兼容) ---
      void insert(Eigen::Vector2i idx, KdNodePtr KdNode) {
          data_2d_.insert(std::make_pair(idx, KdNode));
      }

      KdNodePtr find(Eigen::Vector2i idx) {
          auto iter = data_2d_.find(idx);
          return iter == data_2d_.end() ? NULL : iter->second;
      }

      // --- 3D 接口 (核心修改：增加直接接收 Vector3i 的函数) ---
      
      // 增加：直接插入 Vector3i 索引
      void insert(Eigen::Vector3i idx, KdNodePtr KdNode) {
          data_3d_.insert(std::make_pair(idx, KdNode));
      }

      // 增加：直接查找 Vector3i 索引 (解决你 search 函数中编译报错的关键)
      KdNodePtr find(Eigen::Vector3i idx) {
          auto iter = data_3d_.find(idx);
          return iter == data_3d_.end() ? NULL : iter->second;
      }

      // 保留：原来的三参数插入 (内部会自动组合成 Vector3i)
      void insert(Eigen::Vector2i idx, int leg_idx, KdNodePtr KdNode) {
          data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), leg_idx), KdNode));
      }

      // 保留：原来的双参数查找
      KdNodePtr find(Eigen::Vector2i idx, int leg_idx) {
          auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), leg_idx));
          return iter == data_3d_.end() ? NULL : iter->second;
      }

      void clear() {
          data_2d_.clear();
          data_3d_.clear();
      }
  };

  class KinodynamicAstar
  {
  private:
    /* ---------- main data structure----------  */
    vector<KdNodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    KdNodeHashTable expanded_nodes_;
    std::priority_queue<KdNodePtr, std::vector<KdNodePtr>, KdNodeComparator> open_set_;
    std::vector<KdNodePtr> path_nodes_;
    DynamicRiskField risk_field;

    /*----------  paramter  ---------- */
    int allocate_num_;
    double lambda_heu_;
    double tie_breaker_;
    double horizon_;
    double resolution_, inv_resolution_;
    double max_al_, max_aw_, max_api_;
    double weight_dyn_obs_, weight_steering_, predHorizon_, ts_, distThreshDynamic_;
    double tsSample_;
    Eigen::Vector2d origin_, map_size_2d_;
    ros::Publisher kin_SamplePath_pub_;

    // 动态障碍物信息存储
    std::vector<Eigen::Vector3d> dynObstaclesPos_;   // 动态障碍物当前位置
    std::vector<Eigen::Vector3d> dynObstaclesVel_;   // 动态障碍物速度
    std::vector<Eigen::Vector3d> dynObstaclesSize_;  // 动态障碍物尺寸

    CollisionDetection::Ptr collision_;
    LFPC::Ptr lfpc_model_;

    /* helper */
    Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    Eigen::Vector2d stateToPos(Eigen::Vector3d state);
    Eigen::Vector2i stateToIndex(Eigen::Vector3d state);

    void retrievePath(KdNodePtr end_node);

    /* shot trajectory */

    /*Compute Heuristic*/
    double estimateHeuristic(Eigen::Vector3d input);
    double estimateHeuristic(Eigen::Vector3d input,Eigen::Vector3d state1, Eigen::Vector3d state2);
    /* heuristic function */
    double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

    /* dynamic obstacle cost function for front-end planning */
    double getDynamicObstacleCost(const Eigen::Vector3d& pos, 
                                   const std::vector<Eigen::Vector3d>& dynObsPos,
                                   const std::vector<Eigen::Vector3d>& dynObsVel,
                                   const std::vector<Eigen::Vector3d>& dynObsSize,
                                   double predHorizon,
                                   double ts,
                                   double distThreshDynamic);

    /* state propagation */
    void stateTransit(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                      Eigen::Vector3d input, int n);

  public:
    KinodynamicAstar(){};
    ~KinodynamicAstar();

    enum
    {
      REACH_END = 1,
      NO_PATH = 2,
      NEAR_END = 3
    };

    /* main API */
    bool search(Eigen::Vector3d start_pos, Eigen::Vector3d start_state,
               Eigen::Vector3d end_pos);
    std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getComPos();
    std::vector<Eigen::Vector3d> getFeetPos();
    int getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives);


    void setParam(ros::NodeHandle &nh);
    void init();
    void reset();

    void setCollision(const CollisionDetection::Ptr &col);
    void setModel(const LFPC::Ptr &col);
    
    // 设置动态障碍物信息（从dynamicDetector获取）
    void setDynamicObstacles(const std::vector<Eigen::Vector3d>& pos,
                             const std::vector<Eigen::Vector3d>& vel,
                             const std::vector<Eigen::Vector3d>& size);

    void publishKinodynamicAstarPath(const vector<Eigen::Vector3d>& path);

    typedef shared_ptr<KinodynamicAstar> Ptr;
  };

} // namespace cane_planner

#endif