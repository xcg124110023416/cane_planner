#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner
{
  MapROS::MapROS()
  {
  }

  MapROS::~MapROS()
  {
  }

  void MapROS::setMap(SDFMap *map)
  {
    this->map_ = map;
  }

  void MapROS::init()
  {
    node_.param("map_ros/fx", fx_, -1.0);
    node_.param("map_ros/fy", fy_, -1.0);
    node_.param("map_ros/cx", cx_, -1.0);
    node_.param("map_ros/cy", cy_, -1.0);

    node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
    node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
    node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
    node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
    node_.param("map_ros/skip_pixel", skip_pixel_, -1);

    node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
    node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
    node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
    node_.param("map_ros/show_occ_time", show_occ_time_, false);
    node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
    node_.param("map_ros/show_all_map", show_all_map_, false);
    node_.param("map_ros/frame_id", frame_id_, std::string("world"));

    proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
    point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
    // proj_points_.reserve(640 * 480 / map_->mp_->skip_pixel_ / map_->mp_->skip_pixel_);
    proj_points_cnt = 0;

    local_updated_ = false;
    esdf_need_update_ = false;
    fuse_time_ = 0.0;
    esdf_time_ = 0.0;
    max_fuse_time_ = 0.0;
    max_esdf_time_ = 0.0;
    fuse_num_ = 0;
    esdf_num_ = 0;
    depth_image_.reset(new cv::Mat);

    rand_noise_ = normal_distribution<double>(0, 0.1);
    random_device rd;
    eng_ = default_random_engine(rd());

    esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);//计算每个体素到最近障碍物的距离
    vis_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

    // publish init
    map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
    map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
    map_local_inflate_pub_ =
        node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
    unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
    esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
    update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
    depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

    // sub init
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 50));
    cloud_sub_.reset(
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/pose", 25));
    odom_sub_.reset(
        new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/map_ros/odom", 25));

    // register callback function
    // depth+pose
    sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
        MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
    // depth+odom
    sync_image_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImageOdom>(
        MapROS::SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&MapROS::depthOdomCallback, this, _1, _2));
    // cloud+pose
    sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
        MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));
    // // cloud+odom
    // sync_cloud_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudOdom>(
    //     MapROS::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    // sync_cloud_odom_->registerCallback(boost::bind(&MapROS::cloudOdomCallback, this, _1, _2));



    // use odometry and point cloud
    indep_cloud_sub_ =
        node_.subscribe<sensor_msgs::PointCloud2>("/map_ros/cloud", 10, &MapROS::cloudCallback, this);
    indep_odom_sub_ =
        node_.subscribe<nav_msgs::Odometry>("/map_ros/odom", 10, &MapROS::odomCallback, this);


    map_start_time_ = ros::Time::now();
  }

  void MapROS::visCallback(const ros::TimerEvent &e)
  {
    std::lock_guard<std::mutex> lock(map_mutex_); // 【等待锁释放后，再上锁】
    publishMapLocal();
    if (show_all_map_)
    {
      // Limit the frequency of all map
      static double tpass = 0.0;
      tpass += (e.current_real - e.last_real).toSec();
      if (tpass > 0.1)
      {
        publishMapAll();
        publishUnknown();
        publishESDF();
        publishUpdateRange();
        tpass = 0.0;
      }
    }

    // publishDepth();
  }

  void MapROS::updateESDFCallback(const ros::TimerEvent & /*event*/)
  {
    std::lock_guard<std::mutex> lock(map_mutex_); // 【自动上锁】
    if (!esdf_need_update_)  return;
    auto t1 = ros::Time::now();

    map_->updateESDF3d();//计算每个体素到最近障碍物的距离，结果保存在distance_buffer_中

    auto t2 = ros::Time::now();
    esdf_time_ += (t2 - t1).toSec();
    max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
    esdf_num_++;
    if (show_esdf_time_)
      ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
               max_esdf_time_);
    esdf_need_update_ = false;
  }

  void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                 const geometry_msgs::PoseStampedConstPtr &pose)
  {
    camera_pos_(0) = pose->pose.position.x;
    camera_pos_(1) = pose->pose.position.y;
    camera_pos_(2) = pose->pose.position.z;
    camera_pos_(2) = 0.0;

    if (!map_->isInMap(camera_pos_)) // exceed mapped region
    {
      ROS_WARN("camera_pos exceed mapped region");
      return;
    }

    camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                   pose->pose.orientation.y, pose->pose.orientation.z);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
    cv_ptr->image.copyTo(*depth_image_);

    auto t1 = ros::Time::now();

    // generate point cloud, update map
    proessDepthImage();
    map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }

    auto t2 = ros::Time::now();
    fuse_time_ += (t2 - t1).toSec();
    max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
    fuse_num_ += 1;
    if (show_occ_time_)
      ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
               max_fuse_time_);
  }

  void MapROS::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                 const nav_msgs::OdometryConstPtr &odom)
  {
    camera_pos_(0) = odom->pose.pose.position.x;
    camera_pos_(1) = odom->pose.pose.position.y;
    // camera_pos_(2) = odom->pose.pose.position.z;
    camera_pos_(2) = 0.0;
    if (!map_->isInMap(camera_pos_)) // exceed mapped region
      return;
    camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                   odom->pose.pose.orientation.x,
                                   odom->pose.pose.orientation.y,
                                   odom->pose.pose.orientation.z);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
    cv_ptr->image.copyTo(*depth_image_);

    auto t1 = ros::Time::now();

    // generate point cloud, update map
    proessDepthImage();
    map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }

    auto t2 = ros::Time::now();
    fuse_time_ += (t2 - t1).toSec();
    max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
    fuse_num_ += 1;
    if (show_occ_time_)
      ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
               max_fuse_time_);
  }

  void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                                 const geometry_msgs::PoseStampedConstPtr &pose)
  {
    camera_pos_(0) = pose->pose.position.x;
    camera_pos_(1) = pose->pose.position.y;
    camera_pos_(2) = pose->pose.position.z;
    camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                   pose->pose.orientation.y, pose->pose.orientation.z);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    int num = cloud.points.size();

    // 使用独立的一个方法来解决建图中过程点云的问题；
    map_->BuildsimulationMap(cloud, num, camera_pos_);

    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }
  }


  void MapROS::odomCallback(const nav_msgs::OdometryConstPtr& odom) {

    // tf from cam to odom
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header = odom->header;//camera_init
    pose_cam.pose = odom->pose.pose;
    geometry_msgs::PoseStamped pose_world;
    tf_listener_.transformPose("world", pose_cam, pose_world);//camera_init to world，实际没有变化
    // if (has_first_depth_) return;

    camera_pos_(0) = pose_world.pose.position.x;
    camera_pos_(1) = pose_world.pose.position.y;
    camera_pos_(2) = pose_world.pose.position.z;

    camera_q_ = Eigen::Quaterniond( pose_cam.pose.orientation.w,
                                pose_cam.pose.orientation.x,
                                pose_cam.pose.orientation.y,
                                pose_cam.pose.orientation.z);

    // if (!map_->isInMap(camera_pos_)) // exceed mapped region
    // return;

    has_odom_ = true;
  }


void MapROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  // has_cloud_ = true;

  if (!has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0) return;

  if (isnan(camera_pos_(0)) || isnan(camera_pos_(1)) || isnan(camera_pos_(2))) return;

  map_->resetBuffer(camera_pos_ - map_ ->mp_->local_update_range_,
                    camera_pos_ + map_ ->mp_->local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(map_ ->mp_->obstacles_inflation_ / map_ ->mp_->resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = map_ ->mp_->map_max_boundary_(0);
  min_y = map_ ->mp_->map_max_boundary_(1);
  min_z = map_ ->mp_->map_max_boundary_(2);

  max_x = map_ ->mp_->map_min_boundary_(0);
  max_y = map_ ->mp_->map_min_boundary_(1);
  max_z = map_ ->mp_->map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < map_ ->mp_->local_update_range_(0) && fabs(devi(1)) < map_ ->mp_->local_update_range_(1) &&
        fabs(devi(2)) < map_ ->mp_->local_update_range_(2)) {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {

            p3d_inf(0) = pt.x + x * map_ ->mp_->resolution_;
            p3d_inf(1) = pt.y + y * map_ ->mp_->resolution_;
            p3d_inf(2) = pt.z + z * map_ ->mp_->resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            map_ ->posToIndex(p3d_inf, inf_pt);

            if (!map_ ->isInMap(inf_pt)) continue;

            int idx_inf = map_ ->toAddress(inf_pt);

            map_ ->md_->occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, camera_pos_(0));
  min_y = min(min_y, camera_pos_(1));
  min_z = min(min_z, camera_pos_(2));

  max_x = max(max_x, camera_pos_(0));
  max_y = max(max_y, camera_pos_(1));
  max_z = max(max_z, camera_pos_(2));

  max_z = max(max_z, map_ ->mp_->ground_height_);

  map_ ->posToIndex(Eigen::Vector3d(max_x, max_y, max_z), map_ ->md_->local_bound_max_);
  map_ ->posToIndex(Eigen::Vector3d(min_x, min_y, min_z), map_ ->md_->local_bound_min_);

  map_ ->boundIndex(map_ ->md_->local_bound_min_);
  map_ ->boundIndex(map_ ->md_->local_bound_max_);

  esdf_need_update_ = true;
}

  void MapROS::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                                 const nav_msgs::OdometryConstPtr &odom)
  {
    // tf from cam to odom
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header = odom->header;//camera_init
    pose_cam.pose = odom->pose.pose;
    geometry_msgs::PoseStamped pose_world;
    tf_listener_.transformPose("world", pose_cam, pose_world);//camera_init to world，实际没有变化
    camera_pos_(0) = pose_world.pose.position.x;
    camera_pos_(1) = pose_world.pose.position.y;
    camera_pos_(2) = pose_world.pose.position.z;
    if (!map_->isInMap(camera_pos_)) // exceed mapped region
      return;
    camera_q_ = Eigen::Quaterniond( pose_cam.pose.orientation.w,
                                    pose_cam.pose.orientation.x,
                                    pose_cam.pose.orientation.y,
                                    pose_cam.pose.orientation.z);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);//转换为pcl类型cloud，点云意味着障碍物
    int num = cloud.points.size();
    //将输入的点云数据融合到三维栅格地图（SDFMap）中,把相机看到的点云变成一张“概率地图”occupancy_buffer_，更新出哪些地方是障碍、哪些地方是空的，并维护更新范围和缓存，方便下一步地图计算。
    map_->inputPointCloud(cloud, num, camera_pos_);

    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();//主要用于实现三维栅格地图中障碍物的膨胀过程，occupancy_buffer_inflate_为膨胀障碍栅格
      esdf_need_update_ = true;
      local_updated_ = false;
    }
  }

  void MapROS::proessDepthImage()
  {
    proj_points_cnt = 0;

    uint16_t *row_ptr;
    int cols = depth_image_->cols;
    int rows = depth_image_->rows;
    double depth;
    Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
    Eigen::Vector3d pt_cur, pt_world;
    const double inv_factor = 1.0 / k_depth_scaling_factor_;

    for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
    {
      row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
      for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
      {
        depth = (*row_ptr) * inv_factor;
        row_ptr = row_ptr + skip_pixel_;

        // // filter depth
        // if (depth > 0.01)
        //   depth += rand_noise_(eng_);

        // TODO: simplify the logic here
        if (*row_ptr == 0 || depth > depth_filter_maxdist_)
          depth = depth_filter_maxdist_;
        else if (depth < depth_filter_mindist_)
          continue;

        pt_cur(0) = (u - cx_) * depth / fx_;
        pt_cur(1) = (v - cy_) * depth / fy_;
        pt_cur(2) = depth;
        pt_world = camera_r * pt_cur + camera_pos_;
        auto &pt = point_cloud_.points[proj_points_cnt++];
        pt.x = pt_world[0];
        pt.y = pt_world[1];
        pt.z = pt_world[2];
      }
    }

    // publishDepth();
  }

  void MapROS::publishMapAll()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
      for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
        for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
          {
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud1.push_back(pt);
          }
        }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    map_all_pub_.publish(cloud_msg);

    // Output time and known volumn
    double time_now = (ros::Time::now() - map_start_time_).toSec();
    double known_volumn = 0;

    for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
      for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
        for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
            known_volumn += 0.1 * 0.1 * 0.1;
        }

    ofstream file("/home/boboyu/workspaces/plan_ws/src/fast_planner/exploration_manager/resource/"
                  "curve1.txt",
                  ios::app);
    file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
  }

  void MapROS::publishMapLocal()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    Eigen::Vector3i min_cut = map_->md_->local_bound_min_;//全局变量
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    // for (int z = min_cut(2); z <= max_cut(2); ++z)
    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
        {
          // if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)//该点occupancy_buffer_大于可能是障碍物的阈值
          // {
          //   // Occupied cells
          //   Eigen::Vector3d pos;
          //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          //   if (pos(2) > visualization_truncate_height_)
          //     continue;
          //   if (pos(2) < visualization_truncate_low_)
          //     continue;

          //   pt.x = pos(0);
          //   pt.y = pos(1);
          //   pt.z = pos(2);
          //   cloud.push_back(pt);
          // }
          if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)//或者该体素本身被视为膨胀单位
          {
            // Inflated occupied cells
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud2.push_back(pt);
          }
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    cloud2.width = cloud2.points.size();
    cloud2.height = 1;
    cloud2.is_dense = true;
    cloud2.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud2, cloud_msg);
    map_local_pub_.publish(cloud_msg);
    // pcl::toROSMsg(cloud2, cloud_msg);
    map_local_inflate_pub_.publish(cloud_msg);
  }

  void MapROS::publishUnknown()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
    map_->boundIndex(max_cut);
    map_->boundIndex(min_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3)
          {
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    unknown_pub_.publish(cloud_msg);
  }

  void MapROS::publishDepth()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < proj_points_cnt; ++i)
    {
      cloud.push_back(point_cloud_.points[i]);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    depth_pub_.publish(cloud_msg);
  }

  void MapROS::publishUpdateRange()
  {
    Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
    visualization_msgs::Marker mk;
    map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
    map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

    cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
    cube_scale = esdf_max_pos - esdf_min_pos;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;
    mk.pose.position.x = cube_pos(0);
    mk.pose.position.y = cube_pos(1);
    mk.pose.position.z = cube_pos(2);
    mk.scale.x = cube_scale(0);
    mk.scale.y = cube_scale(1);
    mk.scale.z = cube_scale(2);
    mk.color.a = 0.3;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    update_range_pub_.publish(mk);
  }

  void MapROS::publishESDF()
  {
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    // 获取当前局部地图的边界索引
    Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_);
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_);
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    // 遍历 X 和 Y 轴（水平面）
    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
      {
        // 【核心修改】：投影逻辑
        // 对于每个 (x, y)，我们要找到垂直方向 (z) 上最小的距离值
        double min_dist_in_column = max_dist; // 初始化为最大值

        // 遍历 Z 轴范围 (从 min_z 到 max_z)
        for (int z = min_cut(2); z <= max_cut(2); ++z) {
            Eigen::Vector3d pos_check;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos_check);

            if (pos_check(2) > visualization_truncate_height_)
              continue;
            if (pos_check(2) < visualization_truncate_low_)
              continue;
            
            // 获取该 3D 点的距离
            double d = map_->getDistance(pos_check);
            
            // 更新最小值
            if (d < min_dist_in_column) {
                min_dist_in_column = d;
            }
        }

        // 将找到的最小距离作为该 (x,y) 点的显示值
        dist = min_dist_in_column;
        dist = min(dist, max_dist);
        dist = max(dist, min_dist);

        // 计算显示位置
        Eigen::Vector3d pos;
        // 这里 Z 轴依然固定在切片高度，或者设为 0，方便在平面上查看
        map_->indexToPos(Eigen::Vector3i(x, y, 1), pos); 
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = esdf_slice_height_; // 投影到一个固定高度平面上显示

        // 颜色强度映射：距离越近越黑/红，越远越白/绿 (取决于 Rviz 配色)
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
      }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    esdf_pub_.publish(cloud_msg);
  }
} // namespace fast_planner