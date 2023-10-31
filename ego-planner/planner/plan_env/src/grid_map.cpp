#include "plan_env/grid_map.h"
#include "time.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

int my_count=0;

clock_t start_clock,end_clock;

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("grid_map/obstacles_inflation2d", mp_.obstacles_inflation2d_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, 999.0);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("grid_map/map_type", mp_.map_type_, 3);
  ROS_INFO("map usage : %d",mp_.map_type_);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("map"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  int buffer_size_2d = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_segmentation_ = vector<char>(buffer_size, 0);
  md_.terrain_height_ = vector<double>(buffer_size_2d,0);
  if(mp_.map_type_==EXPLORATION)
  {
      md_.occupancy_buffer_inflate_2d_ = vector<char>(buffer_size_2d,1);
  }else if(mp_.map_type_==PLANNING)
  {
      md_.occupancy_buffer_inflate_2d_ = vector<char>(buffer_size_2d,0);
  }
  else
  {
      ROS_ERROR("Map usage error!");
  }

  md_.gradient_x_ = vector<double>(buffer_size_2d,0);
  md_.gradient_y_ = vector<double>(buffer_size_2d,0);
  md_.gradient_general_ = vector<double>(buffer_size_2d,0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;
  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, -0.02,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/grid_map/depth", 50));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud", 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/grid_map/odom", 10, &GridMap::odomCallback, this);
  if(mp_.map_type_==PLANNING)
  {
      indep_cloud_segmentation_sub_ =
              node_.subscribe<sensor_msgs::PointCloud2>("/benchmark/N", 10, &GridMap::cloudSegmentationCallback_PLANNING, this);

  }else if(mp_.map_type_==EXPLORATION)
  {
      indep_cloud_segmentation_sub_ =
              node_.subscribe<sensor_msgs::PointCloud2>("/benchmark/N", 10, &GridMap::cloudSegmentationCallback_EXPLORATION, this);

  }

  if(mp_.map_type_==EXPLORATION)
  {
      occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  }

  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

  map_2d_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/map_2d",10);
  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 10);

  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/unknown", 10);
  terrain_height_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/terrain_height", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

//记录grid_map中每一个栅格被ray穿过的次数，并记录第一个次被穿过的元素到cache_voxel_中
int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v++)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
    // if (md_.proj_points_.size() == 0)
    if (!md_.is_pcl_update)
        return;

    ros::Time t1, t2;

    md_.raycast_num_ += 1;

    int vox_idx;
    double length;

    // bounding box of updated region
    double min_x = mp_.map_max_boundary_(0);
    double min_y = mp_.map_max_boundary_(1);
    double min_z = mp_.map_max_boundary_(2);

    double max_x = mp_.map_min_boundary_(0);
    double max_y = mp_.map_min_boundary_(1);
    double max_z = mp_.map_min_boundary_(2);

    RayCaster raycaster;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt, pt_w;

    for (int i = 0; i < md_.lidar_proj_points_.points.size(); ++i)
    {
        pt_w  <<md_.lidar_proj_points_.points[i].x,md_.lidar_proj_points_.points[i].y,md_.lidar_proj_points_.points[i].z;

        // set flag for projected point

        if (!isInMap(pt_w))
        {
            pt_w = closetPointInMap(pt_w, md_.camera_pos_);

            length = (pt_w - md_.camera_pos_).norm();
            if (length > mp_.max_ray_length_)
            {
                pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
            }
            vox_idx = setCacheOccupancy(pt_w, 0);
        }
        else
        {
            length = (pt_w - md_.camera_pos_).norm();

            if (length > mp_.max_ray_length_)
            {
                pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
                vox_idx = setCacheOccupancy(pt_w, 0);
            }
            else
            {
                vox_idx = setCacheOccupancy(pt_w, 1);
            }
        }

        max_x = max(max_x, pt_w(0));
        max_y = max(max_y, pt_w(1));
        max_z = max(max_z, pt_w(2));

        min_x = min(min_x, pt_w(0));
        min_y = min(min_y, pt_w(1));
        min_z = min(min_z, pt_w(2));

        // raycasting between camera center and point

        if (vox_idx != INVALID_IDX)
        {
            if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
            {

                continue;
            }
            else
            {
                md_.flag_rayend_[vox_idx] = md_.raycast_num_;
            }
        }

        raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

        while (raycaster.step(ray_pt))
        {
            Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
            length = (tmp - md_.camera_pos_).norm();

             if (length < mp_.min_ray_length_) break;

            vox_idx = setCacheOccupancy(tmp, 0);

            if (vox_idx != INVALID_IDX)
            {
                if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
                {
                    //ROS_INFO("Same ray .skip!");
                    break;
                }
                else
                {
                    md_.flag_traverse_[vox_idx] = md_.raycast_num_;
                }
            }
        }
    }

    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));

    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));
    max_z = max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.local_updated_ = true;

    // update occupancy cached in queue
    Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
    Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;
    while (!md_.cache_voxel_.empty())
    {

        Eigen::Vector3i idx = md_.cache_voxel_.front();
        int idx_ctns = toAddress(idx);
        md_.cache_voxel_.pop();

        double log_odds_update =
                md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

        md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;


        if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
        {
            continue;
        }
        else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
        {
            md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
            continue;
        }

        bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
        if (!in_local)
        {
            md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
        }

        md_.occupancy_buffer_[idx_ctns] =
                std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                         mp_.clamp_max_log_);
    }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::projectMapTo2d() {

    Eigen::Vector3d temp_pos;
    Eigen::Vector3i temp_idx;
    //calculate height
    for(int x=md_.local_bound_min_(0);x<md_.local_bound_max_(0);x++)
    {
        for(int y=md_.local_bound_min_(1);y<md_.local_bound_max_(1);y++)
        {
            for(int z = 0;z<md_.local_bound_max_(2);z++)
            {
                temp_idx = Eigen::Vector3i(x,y,z);
               if(md_.occupancy_buffer_inflate_[toAddress(temp_idx)]==0||
                  (md_.occupancy_buffer_inflate_[toAddress(temp_idx)]==1&&z==(md_.local_bound_max_(2)-1)))
               {
                   indexToPos(temp_idx,temp_pos);
                   md_.terrain_height_[toAddress2d(temp_idx(0),temp_idx(1))] = temp_pos(2);
                   break;
               }

            }
        }
    }
//    Eigen::Matrix<double,3,3> kernel_x,kernel_y;
//    kernel_x<<-1,0,1,
//              -2,0,2,
//              -1,0,1;
//    kernel_y<<-1,-2,-1,
//               0,0,0,
//               -1,2,1;
//    for(int x=md_.local_bound_min_(0);x<md_.local_bound_max_(0);x++)
//    {
//        for(int y=md_.local_bound_min_(1);y<md_.local_bound_max_(1);y++)
//        {
//            md_.gradient_x_[toAddress2d(x,y)] = 0;
//            md_.gradient_y_[toAddress2d(x,y)] = 0;
//            for(int dx=-1;dx<=1;dx++)
//            {
//                if(x+dx<0 || x+dx >md_.local_bound_max_(0)) continue;
//                for(int dy=-1;dy<=1;dy++)
//                {
//                    if(y+dy<0 || y+dy >md_.local_bound_max_(0)) continue;
//                    md_.gradient_x_[toAddress2d(x,y)] += md_.terrain_height_[toAddress2d(x+dx,y+dy)]*kernel_x(dx+1,dy+1);
//                    md_.gradient_y_[toAddress2d(x,y)] += md_.terrain_height_[toAddress2d(x+dx,y+dy)]*kernel_y(dx+1,dy+1);
//                }
//            }
//
//        }
//    }
//    int buffer_size_2d = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);
//    for(int i=0;i<buffer_size_2d;i++)
//    {
//        md_.gradient_general_[i] = md_.gradient_x_[i]*0.5+md_.gradient_y_[i]*0.5;
//        if(md_.gradient_general_[i] >0.5)
//        {
//            md_.occupancy_buffer_inflate_2d_[i] = 1;
//        }
//    }



}

void GridMap::fliterLocalMap() {
    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;
    Eigen::Vector3d min,max;

    static int count=0;
    boundIndex(max_cut);
    boundIndex(min_cut);
    indexToPos(min_cut,min);
    indexToPos(max_cut,max);

//    if(count%40==0)
//    {
//        cout<<"Update min idx : "<<min_cut(0)<<","<<min_cut(1)<<","<<min_cut(2)<<endl;
//        cout<<"Update max idx : "<<max_cut(0)<<","<<max_cut(1)<<","<<max_cut(2)<<endl;
//        cout<<"Update min pos : "<<min(0)<<","<<min(1)<<","<<min(2)<<endl;
//        cout<<"Update max pos : "<<max(0)<<","<<max(1)<<","<<max(2)<<endl;
//    }
//    count+=1;

    Eigen::Vector3i idx,idx_up,idx_down;
    Eigen::Vector3d pos_down;
    for(int i=min_cut(0);i<max_cut(0);i++)
    {
        for(int j=min_cut(1);j<max_cut(1);j++)
        {
            for(int k=min_cut(2)+1;k<max_cut(2)-1;k++)
            {
                idx = Eigen::Vector3i(i,j,k);
                idx_up = Eigen::Vector3i(i,j,k+1);
                idx_down = Eigen::Vector3i(i,j,k-1);
                indexToPos(idx_down,pos_down);
                if(isUnknown(idx)&&isKnownFree(idx_up)&&
                   (isKnownFree(idx_down)||pos_down(2)<md_.camera_pos_(2)))
                {
                    md_.occupancy_buffer_[toAddress(idx)] = 0;
                }
            }
        }
    }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  //publishMap();
  if(mp_.map_type_==EXPLORATION)
  {
      publishUnknown();
  }
//  if(mp_.map_type_==PLANNING)
//  {
      publishMapInflate(true);
//  }
    //publishMap2D();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{

  if (!md_.occ_need_update_)
    return;
  start_clock = clock();
  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  //projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
    double duration;
    end_clock = clock();
    duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
    static int count =0;
    count+=1;
    if(count==10)
    {
        cout<<"point size :"<<md_.lidar_proj_points_.points.size()<<endl;
        ROS_INFO("Control times : %f ms",duration);
        count=0;
    }
  // t3 = ros::Time::now();
  fliterLocalMap();

  //if (md_.local_updated_)
    //clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.is_pcl_update = false;



}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}
void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  my_count+=1;
  ros::Time time_begin = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);
  pcl::fromROSMsg(*img, md_.lidar_proj_points_);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  md_.lidar_proj_points_ = latest_cloud;
  md_.is_pcl_update = true;
  md_.occ_need_update_ = true;


//  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
//                    md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);
  //if(mp_.map_type_==PLANNING)
  //{
      //ROS_INFO("Point cloud num : %d",latest_cloud.points.size());
      for (size_t i = 0; i < latest_cloud.points.size(); ++i)
      {
          pt = latest_cloud.points[i];
          p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

          /* point inside update range */
          Eigen::Vector3d devi = p3d - md_.camera_pos_;
          Eigen::Vector3i inf_pt;

          if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
              fabs(devi(2)) < mp_.local_update_range_(2))
          {

              /* inflate the point */
              for (int x = -inf_step; x <= inf_step; ++x)
                  for (int y = -inf_step; y <= inf_step; ++y)
                      for (int z = -inf_step_z; z <= inf_step_z; ++z)
                      {

                          p3d_inf(0) = pt.x + x * mp_.resolution_;
                          p3d_inf(1) = pt.y + y * mp_.resolution_;
                          p3d_inf(2) = pt.z + z * mp_.resolution_;

                          max_x = max(max_x, p3d_inf(0));
                          max_y = max(max_y, p3d_inf(1));
                          max_z = max(max_z, p3d_inf(2));

                          min_x = min(min_x, p3d_inf(0));
                          min_y = min(min_y, p3d_inf(1));
                          min_z = min(min_z, p3d_inf(2));

                          posToIndex(p3d_inf, inf_pt);

                          if (!isInMap(inf_pt))
                              continue;

                          int idx_inf = toAddress(inf_pt);

                          md_.occupancy_buffer_inflate_[idx_inf] = 1;
                      }
          }
      }

      min_x = min(min_x, md_.camera_pos_(0));
      min_y = min(min_y, md_.camera_pos_(1));
      min_z = min(min_z, md_.camera_pos_(2));

      max_x = max(max_x, md_.camera_pos_(0));
      max_y = max(max_y, md_.camera_pos_(1));
      max_z = max(max_z, md_.camera_pos_(2));

      max_z = max(max_z, mp_.ground_height_);

      posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
      posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

      boundIndex(md_.local_bound_min_);
      boundIndex(md_.local_bound_max_);
  //}
    //projectMapTo2d();
    ros::Time time_end = ros::Time::now();
    if(my_count%20==0)
    {
      //ROS_INFO("Update map time : %f ms",(time_end-time_begin).toSec()*1000);
      my_count=0;
    }
    
}

void GridMap::cloudSegmentationCallback_EXPLORATION(const sensor_msgs::PointCloud2ConstPtr &cloud_segmentation) {
    my_count+=1;
    ros::Time time_begin = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*cloud_segmentation, latest_cloud);

    if (!md_.has_odom_)
    {
        std::cout << "no odom!" << std::endl;
        return;
    }

    if (latest_cloud.points.size() == 0)
        return;

    if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
        return;

    //md_.lidar_proj_points_ = latest_cloud;

//  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
//                    md_.camera_pos_ + mp_.local_update_range_);

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    int inf_step = ceil(mp_.obstacles_inflation2d_ / mp_.resolution_);
    int inf_step_z = 1;
    //cout<<"Inflate size : "<<inf_step<<endl;
    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    //if(mp_.map_type_==PLANNING)
    //{
    //ROS_INFO("Point cloud num : %d",latest_cloud.points.size());
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        /* point inside update range */
        Eigen::Vector3d devi = p3d - md_.camera_pos_;
        Eigen::Vector3i inf_pt;

        if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
            fabs(devi(2)) < mp_.local_update_range_(2))
        {

            /* inflate the point */
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z)
                    {
                        p3d_inf(0) = pt.x + x * mp_.resolution_;
                        p3d_inf(1) = pt.y + y * mp_.resolution_;
                        p3d_inf(2) = pt.z + z * mp_.resolution_;

                        max_x = max(max_x, p3d_inf(0));
                        max_y = max(max_y, p3d_inf(1));
                        max_z = max(max_z, p3d_inf(2));

                        min_x = min(min_x, p3d_inf(0));
                        min_y = min(min_y, p3d_inf(1));
                        min_z = min(min_z, p3d_inf(2));

                        posToIndex(p3d_inf, inf_pt);
                       // ROS_INFO("Run here! 2");
                        if (!isInMap(inf_pt))
                            continue;

                        int idx_inf = toAddress(inf_pt);

                        md_.occupancy_buffer_inflate_segmentation_[idx_inf] = 1;
                    }
        }
    }

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    Eigen::Vector3i temp_idx;
    Eigen::Vector3i temp_pos;
    int count=0,count_free=0;
    for(int x=md_.local_bound_min_(0);x<md_.local_bound_max_(0);x++)
        for(int y=md_.local_bound_min_(1);y<md_.local_bound_max_(1);y++)
            for(int z=md_.local_bound_min_(2);z<md_.local_bound_max_(2);z++)
            {
                temp_idx<<x,y,z;
                if(md_.occupancy_buffer_inflate_segmentation_[toAddress(temp_idx)]==1&&z<=md_.camera_pos_(2))
                {
                    count+=1;
                }
                if(isKnownFree(temp_idx))
                {
                    count_free+=1;
                }

                if(z==(md_.local_bound_max_(2)-1))
                {
                    if(count>=1)
                    {
                       md_.occupancy_buffer_inflate_2d_[toAddress2d(x,y)]=1;
                    }
                    else
                    {
                        //ROS_INFO("Run here! 2");
                        if(count==0&&count_free>1) md_.occupancy_buffer_inflate_2d_[toAddress2d(x,y)]=0;
                    }
                    count =0;
                    count_free=0;
                }
            }

    publishMap2D();
    //ROS_INFO("Run here 3");

}

void GridMap::cloudSegmentationCallback_PLANNING(const sensor_msgs::PointCloud2ConstPtr &cloud_segmentation) {
    my_count+=1;
    ros::Time time_begin = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*cloud_segmentation, latest_cloud);

    if (!md_.has_odom_)
    {
        std::cout << "no odom!" << std::endl;
        return;
    }

    if (latest_cloud.points.size() == 0)
        return;

    if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
        return;

    md_.lidar_proj_points_ = latest_cloud;

//  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
//                    md_.camera_pos_ + mp_.local_update_range_);

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    int inf_step_z = 1;
    //cout<<"Inflate size : "<<inf_step<<endl;
    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    //if(mp_.map_type_==PLANNING)
    //{
    //ROS_INFO("Point cloud num : %d",latest_cloud.points.size());
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        /* point inside update range */
        Eigen::Vector3d devi = p3d - md_.camera_pos_;
        Eigen::Vector3i inf_pt;

        if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
            fabs(devi(2)) < mp_.local_update_range_(2))
        {

            /* inflate the point */
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z)
                    {
                        p3d_inf(0) = pt.x + x * mp_.resolution_;
                        p3d_inf(1) = pt.y + y * mp_.resolution_;
                        p3d_inf(2) = pt.z + z * mp_.resolution_;

                        max_x = max(max_x, p3d_inf(0));
                        max_y = max(max_y, p3d_inf(1));
                        max_z = max(max_z, p3d_inf(2));

                        min_x = min(min_x, p3d_inf(0));
                        min_y = min(min_y, p3d_inf(1));
                        min_z = min(min_z, p3d_inf(2));

                        posToIndex(p3d_inf, inf_pt);
                        // ROS_INFO("Run here! 2");
                        if (!isInMap(inf_pt))
                            continue;

                        int idx_inf = toAddress(inf_pt);

                        md_.occupancy_buffer_inflate_segmentation_[idx_inf] = 1;
                    }
        }
    }

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    Eigen::Vector3i temp_idx;
    Eigen::Vector3i temp_pos;
    int count=0;
    for(int x=md_.local_bound_min_(0);x<md_.local_bound_max_(0);x++)
        for(int y=md_.local_bound_min_(1);y<md_.local_bound_max_(1);y++)
            for(int z=md_.local_bound_min_(2);z<md_.local_bound_max_(2);z++)
            {
                temp_idx<<x,y,z;
                if(md_.occupancy_buffer_inflate_segmentation_[toAddress(temp_idx)]==1&&z<=(md_.camera_pos_(2)+3))
                {
                    count+=1;
                }

                if(z==(md_.local_bound_max_(2)-1))
                {
                    if(count>=1)
                    {
                        md_.occupancy_buffer_inflate_2d_[toAddress2d(x,y)]=1;
                    }
                    count =0;
                }
            }

    publishMap2D();

}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
          //cout<<"md_.occupancy_buffer_ : "<<md_.occupancy_buffer_[toAddress(x, y, z)]<<endl;
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);

}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_||pos(2)<0.1)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void GridMap::publTerrainHeight() {
    pcl::PointXYZ pt;
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector3i min_cut = md_.local_bound_min_-Eigen::Vector3i(0,0,0);
    Eigen::Vector3i max_cut = md_.local_bound_max_+Eigen::Vector3i(0,0,0);
    static int count=0;
//    if(count%100==0)
//    {
//        cout<<"local bound min : "<<md_.local_bound_min_(0)<<","<<md_.local_bound_min_(1)<<","<<md_.local_bound_min_(2)<<endl;
//        cout<<"local bound max : "<<md_.local_bound_max_(0)<<","<<md_.local_bound_max_(1)<<","<<md_.local_bound_max_(2)<<endl;
//    }
    count+=1;
    boundIndex(max_cut);
    boundIndex(min_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            {
                //cout<<"md_.occupancy_buffer_ : "<<md_.occupancy_buffer_[toAddress(x, y, z)]<<endl;
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, 0), pos);
                pos(2) = md_.terrain_height_[toAddress2d(x,y)];
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                //cloud.push_back(pt);
                cloud->push_back(pt);
            }


    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->header.frame_id = mp_.frame_id_;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    terrain_height_pub_.publish(cloud_msg);
}

void GridMap::publishMap2D() {
    nav_msgs::OccupancyGrid map_2d;
    map_2d.header.frame_id = "map";
    map_2d.info.resolution = mp_.resolution_;
    map_2d.info.width = md_.local_bound_max_(0)-md_.local_bound_min_(0);
    map_2d.info.height = md_.local_bound_max_(1)-md_.local_bound_min_(1);
    map_2d.data = std::vector<int8_t>(map_2d.info.width*map_2d.info.height,0);
    Eigen::Vector3d origin;
    indexToPos(md_.local_bound_min_,origin);
    map_2d.info.origin.position.x = origin(0);
    map_2d.info.origin.position.y = origin(1);
    for(int x = md_.local_bound_min_(0);x<md_.local_bound_max_(0);x++)
    {
        for(int y = md_.local_bound_min_(1);y<md_.local_bound_max_(1);y++)
        {
            if(md_.occupancy_buffer_inflate_2d_[toAddress2d(x,y)]==1)
            {
                map_2d.data[(y-md_.local_bound_min_(1))*map_2d.info.width+(x-md_.local_bound_min_(0))]=100;
            }

        }
    }
    map_2d_pub_.publish(map_2d);
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_-Eigen::Vector3i(0,0,0);
    Eigen::Vector3i max_cut = md_.local_bound_max_+Eigen::Vector3i(0,0,0);
    static int count=0;
//    if(count%100==0)
//    {
//        cout<<"local bound min : "<<md_.local_bound_min_(0)<<","<<md_.local_bound_min_(1)<<","<<md_.local_bound_min_(2)<<endl;
//        cout<<"local bound max : "<<md_.local_bound_max_(0)<<","<<md_.local_bound_max_(1)<<","<<md_.local_bound_max_(2)<<endl;
//    }
    count+=1;
    boundIndex(max_cut);
    boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        //cout<<"md_.occupancy_buffer_ : "<<md_.occupancy_buffer_[toAddress(x, y, z)]<<endl;
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_)
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
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

 int GridMap::getVoxelNum() {
   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
 }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);    
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();   
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;
  
  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
}

// GridMap
