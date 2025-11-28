#include <plan_env/collision_detection.h>
#include <plan_env/sdf_map.h>

using namespace fast_planner;

namespace cane_planner
{
    CollisionDetection::~CollisionDetection()
    {
    }

    void CollisionDetection::init(ros::NodeHandle &nh)
    {
        node_ = nh;
        node_.param("Collision/margin", margin_, -1.0);//当机器人与障碍物之间的距离小于 margin_ 时，认为有碰撞风险。
        node_.param("Collision/SliceHeight", slice_height_, -1.0);//它指定了要在哪个高度平面上进行碰撞检测。
        
        cout << "Collision Detection[18]:margin:" << margin_ << endl;
        cout << "Collision Detection[18]:SliceHeight:" << slice_height_ << endl;
    }
    void CollisionDetection::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
    }

    // 修改后的 3D 碰撞检测函数
bool CollisionDetection::isTraversable(double x, double y) {
    // foot_z: 脚部高度，稍微抬高一点，容忍地面的微小不平整
    double foot_z = slice_height_ - 0.6; 
    // head_z: 头部高度，导盲杖系统的关键保护区域
    double head_z = slice_height_ + 1.4; 
    // check_step: 检查步长，建议设为分辨率的 2-3 倍 (0.2 - 0.3m)
    double check_step = 0.3;        
    // ===  身体圆柱体碰撞检测 (防止撞头/撞腰) ===
    for (double z = foot_z; z <= head_z; z += check_step) {
        Eigen::Vector3d check_pt(x, y, z);
        
        double dist = sdf_map_->getDistance(check_pt);
        
        if (dist < margin_) { 
            return false; 
        }
    }
    // 额外检查一下头顶 
    Eigen::Vector3d head_pt(x, y, head_z);
    if (sdf_map_->getDistance(head_pt) < margin_) {
        return false;
    }

    return true; // 所有检查都通过
}

    bool CollisionDetection::isTraversable(Eigen::Vector3d pos)
    {
        double dis = sdf_map_->getDistance(pos);
        if (dis <= margin_)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    double CollisionDetection::getCollisionDistance(Eigen::Vector2d pos)
    {
        Eigen::Vector3d index;
        index(0) = pos(0);
        index(1) = pos(1);
        index(2) = slice_height_;
        double dis = sdf_map_->getDistance(index);
        return dis;
    }

    void CollisionDetection::getSurroundDistance(Eigen::Vector2d pts[2][2][2], double dists[2][2][2])
    {
        Eigen::Vector3d pts_temp[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    pts_temp[x][y][z](0) = pts[x][y][z](0);
                    pts_temp[x][y][z](1) = pts[x][y][z](1);
                    // TODO:this should changle to slice_height_list_;
                    pts_temp[x][y][z](2) = 0.6;

                    dists[x][y][z] = sdf_map_->getDistance(pts_temp[x][y][z]);
                }
    }

} // namespace cane_planner
