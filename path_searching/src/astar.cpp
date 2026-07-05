#include <path_searching/astar.h>
#include <limits>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
  Astar::~Astar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  bool Astar::search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool dynamic, double time_start)
  {
    const bool use_static = use_static_global_map_ && collision_->hasStaticGlobalMap();
    auto in_astar_bounds = [this](const Eigen::Vector2d &pt) {
      return pt(0) > origin_(0) && pt(0) < map_size_2d_(0) &&
             pt(1) > origin_(1) && pt(1) < map_size_2d_(1);
    };
    if (use_static)
    {
      bool start_free = collision_->isStaticTraversable(start_pt(0), start_pt(1));
      bool end_free = collision_->isStaticTraversable(end_pt(0), end_pt(1));
      const int start_near_free = collision_->countStaticTraversableAround(start_pt, 0.5);
      const int end_near_free = collision_->countStaticTraversableAround(end_pt, 0.5);
      ROS_WARN_THROTTLE(1.0,
                        "[Astar static] bounds=[%.2f %.2f]x[%.2f %.2f], start=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f, goal=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f",
                        origin_(0), map_size_2d_(0), origin_(1), map_size_2d_(1),
                        start_pt(0), start_pt(1), in_astar_bounds(start_pt), start_free, start_near_free,
                        collision_->getStaticCollisionDistance(start_pt),
                        end_pt(0), end_pt(1), in_astar_bounds(end_pt), end_free, end_near_free,
                        collision_->getStaticCollisionDistance(end_pt));
    }
    else
    {
      auto count_near_traversable = [this](const Eigen::Vector2d &pt, double radius) {
        const int step = std::max(1, static_cast<int>(std::ceil(radius * inv_resolution_)));
        int count = 0;
        for (int dx = -step; dx <= step; ++dx)
        {
          for (int dy = -step; dy <= step; ++dy)
          {
            if (std::hypot(dx, dy) * resolution_ > radius)
              continue;
            const double x = pt(0) + dx * resolution_;
            const double y = pt(1) + dy * resolution_;
            if (x <= origin_(0) || x >= map_size_2d_(0) ||
                y <= origin_(1) || y >= map_size_2d_(1))
              continue;
            if (collision_->isTraversable(x, y))
              count++;
          }
        }
        return count;
      };
      const bool start_free = collision_->isTraversable(start_pt(0), start_pt(1));
      const bool end_free = collision_->isTraversable(end_pt(0), end_pt(1));
      ROS_WARN_THROTTLE(1.0,
                        "[Astar sdf] bounds=[%.2f %.2f]x[%.2f %.2f], start=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f, goal=(%.2f, %.2f) in=%d free=%d near_free=%d dist=%.2f",
                        origin_(0), map_size_2d_(0), origin_(1), map_size_2d_(1),
                        start_pt(0), start_pt(1), in_astar_bounds(start_pt), start_free,
                        count_near_traversable(start_pt, 0.5),
                        collision_->getCollisionDistance(start_pt),
                        end_pt(0), end_pt(1), in_astar_bounds(end_pt), end_free,
                        count_near_traversable(end_pt, 0.5),
                        collision_->getCollisionDistance(end_pt));
    }

    /* ---------- initialize ---------- */
    NodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->position = start_pt;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    // Eigen::Vector2d end_state(6);
    Eigen::Vector2i end_index;
    // double time_to_goal;

    end_index = posToIndex(end_pt);
    cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
    cur_node->node_state = IN_OPEN_SET;

    open_set_.push(cur_node);
    use_node_num_ += 1;

    if (dynamic)
    {
      time_origin_ = time_start;
      cur_node->time = time_start;
      cur_node->time_idx = timeToIndex(time_start);
      expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
      // cout << "time start: " << time_start << endl;
    }
    else
      expanded_nodes_.insert(cur_node->index, cur_node);

    // NodePtr neighbor = NULL;
    NodePtr terminate_node = NULL;
    Eigen::Vector2d expanded_min = start_pt;
    Eigen::Vector2d expanded_max = start_pt;
    Eigen::Vector2d closest_to_goal = start_pt;
    double closest_goal_dist = (start_pt - end_pt).norm();
    int reject_bounds = 0;
    int reject_closed = 0;
    int reject_collision = 0;

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      /* ---------- get lowest f_score node ---------- */
      cur_node = open_set_.top();
      // cout << "pos: " << cur_node->position.transpose() << endl;
      // cout << "time: " << cur_node->time << endl;
      // cout << "dist: " <<
      // edt_environment_->evaluateCoarseEDT(cur_node->state.head(3),
      // cur_node->time) <<
      // endl;

      /* ---------- determine termination ---------- */

      bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                       abs(cur_node->index(1) - end_index(1)) <= 1;
      double reach_horizon = (cur_node->position - start_pt).norm();

      if (reach_end)
      {
        // cout << "[Astar]:---------------------- " << use_node_num_ << endl;
        // cout << "use node num: " << use_node_num_ << endl;
        // cout << "iter num: " << iter_num_ << endl;
        cout << use_node_num_ << "," << iter_num_ << ",";
        terminate_node = cur_node;
        retrievePath(terminate_node);
        has_path_ = true;

        return true;
      }
      // horizon 为最大搜索距离, 如果到达了最大搜索距离, 但是还是没有找到路径, 则认为没有路径
      if (reach_horizon >= horizon_)
      {
        double cur_near_end = (cur_node->position - end_pt).norm();
        double start_near_end = (start_pt - end_pt).norm();
        if (cur_near_end <= start_near_end)
        {
          std::cout << "[Astar](horizon):---------------------- " << use_node_num_ << std::endl;
          std::cout << use_node_num_ << "," << iter_num_ << ",";
          terminate_node = cur_node;

          retrievePath(terminate_node);
          has_path_ = true;

          return true;
        }
        else
        {
          std::cout << "[Astar](horizon):---------------------- " << use_node_num_ << std::endl;
          std::cout << "!---in horizion no find path--" << std::endl;
        }
      }
      /* ---------- pop node and add to close set ---------- */
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;
      expanded_min = expanded_min.cwiseMin(cur_node->position);
      expanded_max = expanded_max.cwiseMax(cur_node->position);
      const double cur_goal_dist = (cur_node->position - end_pt).norm();
      if (cur_goal_dist < closest_goal_dist)
      {
        closest_goal_dist = cur_goal_dist;
        closest_to_goal = cur_node->position;
      }

      /* ---------- init neighbor expansion ---------- */

      Eigen::Vector2d cur_pos = cur_node->position;
      Eigen::Vector2d pro_pos;
      double pro_t;

      vector<Eigen::Vector2d> inputs;
      Eigen::Vector2d d_pos;

      /* ---------- expansion loop ---------- */
      for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        {
          d_pos << dx, dy;

          if (d_pos.norm() < 1e-3)
            continue;

          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_2d_(0) || pro_pos(1) <= origin_(1) || pro_pos(1) >= map_size_2d_(1))
          {
            // cout << "outside map" << endl;
            reject_bounds++;
            continue;
          }

          /* not in close set */
          Eigen::Vector2i pro_id = posToIndex(pro_pos);
          pro_t = dynamic ? cur_node->time + 1.0 : 0.0;
          int pro_t_id = dynamic ? timeToIndex(pro_t) : 0;
          NodePtr pro_node =
              dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
          {
            // cout << "in closeset" << endl;
            reject_closed++;
            continue;
          }
          Eigen::Vector3d pro_pos_3d;
          pro_pos_3d << pro_pos(0), pro_pos(1), collision_->getSliceHeight(); // 设置路径高度
          const bool near_start = (pro_pos - start_pt).norm() <= static_endpoint_clear_radius_;
          const bool near_goal = (pro_pos - end_pt).norm() <= static_endpoint_clear_radius_;
          const bool near_endpoint = near_start || near_goal;

          /* collision free */
          bool traversable = use_static ?
                                 collision_->isStaticTraversable(pro_pos(0), pro_pos(1)) :
                                 collision_->isTraversable(pro_pos(0), pro_pos(1));
          if (use_static && !traversable)
          {
            traversable = near_endpoint;
          }
          if (!traversable)
          // if (!collision_->isTraversable(pro_pos_3d))
          {
            // cout << "Can't Traversable" << endl;
            reject_collision++;
            continue;
          }

          if (traversable_radius_ > 1e-6 && !near_endpoint)
          {
            const double sample_step = std::max(0.05, resolution_);
            bool disk_traversable = true;
            for (double sx = -traversable_radius_; sx <= traversable_radius_ + 1e-6 && disk_traversable; sx += sample_step)
            {
              for (double sy = -traversable_radius_; sy <= traversable_radius_ + 1e-6; sy += sample_step)
              {
                if (std::hypot(sx, sy) > traversable_radius_ + 1e-6)
                  continue;
                const Eigen::Vector2d sample = pro_pos + Eigen::Vector2d(sx, sy);
                if (sample(0) <= origin_(0) || sample(0) >= map_size_2d_(0) ||
                    sample(1) <= origin_(1) || sample(1) >= map_size_2d_(1))
                {
                  disk_traversable = false;
                  break;
                }
                const bool sample_free = use_static ?
                    collision_->isStaticTraversable(sample(0), sample(1)) :
                    collision_->isTraversable(sample(0), sample(1));
                if (!sample_free)
                {
                  disk_traversable = false;
                  break;
                }
              }
            }
            if (!disk_traversable)
            {
              reject_collision++;
              continue;
            }
          }

          double sdf_dist = use_static ?
              collision_->getStaticCollisionDistance(pro_pos) :
              collision_->getCollisionDistance(pro_pos);
          if (min_clearance_ > 1e-6 && !near_endpoint && sdf_dist < min_clearance_)
          {
            reject_collision++;
            continue;
          }

          /* ---------- compute cost ---------- */
          double tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;

          // Soft clearance bias: nudge the guide path toward the middle without
          // overpowering path length enough to switch corridor topology.
          if (w_clearance_ > 1e-6)
          {
            if (sdf_dist < clearance_sigma_)
            {
              double t = 1.0 - sdf_dist / clearance_sigma_;
              tmp_g_score += w_clearance_ * d_pos.norm() * t * t;
            }
          }
          tmp_f_score = tmp_g_score + lambda_heu_ * getDiagHeu(pro_pos, end_pt);

          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = pro_t;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time_idx, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return false;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + 1.0;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }

          /* ----------  ---------- */
        }
    }

    /* ---------- open set empty, no path ---------- */
    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num_ << endl;
    cout << "iter num: " << iter_num_ << endl;
    ROS_WARN("[Astar debug] %s expanded_bbox=[%.2f %.2f]x[%.2f %.2f], closest_to_goal=(%.2f, %.2f), closest_dist=%.2f, goal=(%.2f, %.2f), rejects bounds=%d closed=%d collision=%d",
             use_static ? "static" : "sdf",
             expanded_min(0), expanded_max(0), expanded_min(1), expanded_max(1),
             closest_to_goal(0), closest_to_goal(1), closest_goal_dist,
             end_pt(0), end_pt(1), reject_bounds, reject_closed, reject_collision);
    return false;
  }

  void Astar::setParam(ros::NodeHandle &nh)
  {

    // resolution 可以理解成最小分辨率
    nh.param("astar/resolution_astar", resolution_, -1.0);
    // 这里的astar可以加上时间维度
    nh.param("astar/time_resolution", time_resolution_, -1.0);
    // 用于放大f_score的一个倍速；
    nh.param("astar/lambda_heu", lambda_heu_, -1.0);
    // 分配的最大可以搜索的数量；
    nh.param("astar/allocate_num", allocate_num_, 1);
    // 搜索允许的最大范围（例如 10 米内）
    nh.param("astar/horizon", horizon_, -1.0);
    // 靠近障碍物的软代价权重（0=不惩罚，建议 0.05~0.3）
    nh.param("astar/w_clearance", w_clearance_, 0.0);
    // 安全距离尺度（m），小于此值代价显著上升
    nh.param("astar/clearance_sigma", clearance_sigma_, 0.5);
    nh.param("astar/min_clearance", min_clearance_, 0.0);
    nh.param("astar/traversable_radius", traversable_radius_, 0.0);
    nh.param("astar/use_static_global_map", use_static_global_map_, false);
    nh.param("astar/static_endpoint_clear_radius", static_endpoint_clear_radius_, 0.25);
    // tie_breaker 见路径规划课程
    tie_breaker_ = 1.0 + 1.0 / 10000;
  }

  void Astar::retrievePath(NodePtr end_node)
  {
    NodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  std::vector<Eigen::Vector2d> Astar::getPath()
  {
    vector<Eigen::Vector2d> path;
    for (size_t i = 0; i < path_nodes_.size(); ++i)
    {
      path.push_back(path_nodes_[i]->position);
    }
    return path;
  }

  double Astar::getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));

    double h = (dx + dy) + (sqrt(2.0) - 2) * min(dx, dy);

    return tie_breaker_ * h;
  }

  double Astar::getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    // double dz = fabs(x1(2) - x2(2));

    // return tie_breaker_ * (dx + dy + dz);
    return tie_breaker_ * (dx + dy);
  }

  double Astar::getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
  {
    return tie_breaker_ * (x2 - x1).norm();
  }

  void Astar::init()
  {
    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = 1.0 / time_resolution_;
    Eigen::Vector3d ori, map_size_3d;
    collision_->getMapRegion(ori, map_size_3d);
    origin_ << ori(0), ori(1);
    map_size_2d_ << map_size_3d(0), map_size_3d(1);

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_2d_.transpose() << endl;

    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new Node;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  // void Astar::setEnvironment(const EDTEnvironment::Ptr &env)
  // {
  //   this->edt_environment_ = env;
  // }

  void Astar::setCollision(const CollisionDetection::Ptr &col)
  {
    this->collision_ = col;
  }

  void Astar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      NodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
  }

  std::vector<NodePtr> Astar::getVisitedNodes()
  {
    vector<NodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  Eigen::Vector2i Astar::posToIndex(Eigen::Vector2d pt)
  {
    Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

    // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
    // origin_(1)) * inv_resolution_),
    //     floor((pt(2) - origin_(2)) * inv_resolution_);

    return idx;
  }

  int Astar::timeToIndex(double time)
  {
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
  }
} // namespace fast_planner
