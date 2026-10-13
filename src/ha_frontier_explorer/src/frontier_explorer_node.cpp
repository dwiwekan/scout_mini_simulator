#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#ifdef HAVE_YOLO_MSGS
#include <yolo_msgs/msg/detection_array.hpp>
#endif

#include <queue>
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class FrontierExplorer : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FrontierExplorer() : Node("frontier_explorer"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

    // ---- Parameters ----
    // relative names so the node works under a robot namespace (e.g. /scout_mini/map)
    map_topic_           = declare_parameter<std::string>("map_topic", "map");
    map_frame_           = declare_parameter<std::string>("map_frame", "map");
    base_frame_          = declare_parameter<std::string>("base_frame", "base_link");

    min_frontier_size_   = declare_parameter<int>("min_frontier_size", 12);
    connectivity8_       = declare_parameter<int>("cluster_connectivity", 8) == 8;
    alpha_               = declare_parameter<double>("alpha_ig", 1.0);
    distance_scale_      = declare_parameter<double>("distance_scale", 1.0);
    beta_                = declare_parameter<double>("beta_dist", 1.0);
    gamma_               = declare_parameter<double>("gamma_yaw", 0.2);
    lambda_              = declare_parameter<double>("lambda_risk", 0.0);
    blacklist_radius_m_  = declare_parameter<double>("blacklist_radius_m", 1.0);
    send_goal_interval_s_= declare_parameter<double>("send_goal_interval_s", 2.0);
    // also blacklist reached goals, so a frontier the sensor can't clear isn't chosen forever
    blacklist_reached_   = declare_parameter<bool>("blacklist_reached", true);
    // cancel (and blacklist) a frontier goal Nav2 hasn't finished in this time; <= 0 disables
    goal_timeout_s_      = declare_parameter<double>("goal_timeout_s", 60.0);

    // Object-driven exploration params
    object_goal_          = declare_parameter<std::string>("object_goal", "");
    detections_topic_     = declare_parameter<std::string>("detections_topic", "/yolo/detections_3d");
    object_persistence_s_ = declare_parameter<double>("object_persistence_s", 2.0);
    min_score_            = declare_parameter<double>("min_score", 0.5);
    dis_goal_m_           = declare_parameter<double>("dis_goal_m", 0.2);
    allow_preempt_        = declare_parameter<bool>("allow_preempt", true);
    stop_distance_m_      = declare_parameter<double>("stop_distance_m", 0.8);
    object_lost_resume_s_ = declare_parameter<double>("object_lost_resume_s", 3.0);

    // Map thresholds
    free_occ_max_        = declare_parameter<int>("free_occ_max", 20);  // 0..100, free <= 20
    free_cost_max_       = declare_parameter<int>("free_cost_max", 10); // 0..255, free <= 10
    unknown_cost_        = declare_parameter<int>("unknown_cost", 255); // NO_INFORMATION

    if (!this->has_parameter("use_sim_time"))
      this->declare_parameter<bool>("use_sim_time", true);
    this->get_parameter("use_sim_time", use_sim_time_);

    // ---- Subscribers ----
    if (map_topic_.find("costmap") != std::string::npos) {
      costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
        map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&FrontierExplorer::onCostmap, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "FrontierExplorer started. Subscribing %s (Costmap)", map_topic_.c_str());
    } else {
      occ_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&FrontierExplorer::onOccupancy, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "FrontierExplorer started. Subscribing %s (OccupancyGrid)", map_topic_.c_str());
    }

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    last_send_goal_sec_ = now().seconds();

#ifdef HAVE_YOLO_MSGS
    det_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
      detections_topic_, rclcpp::QoS(10),
      std::bind(&FrontierExplorer::onDetections, this, std::placeholders::_1));
#else
    if (!object_goal_.empty())
      RCLCPP_WARN(get_logger(), "object_goal '%s' ignored: built without yolo_msgs", object_goal_.c_str());
#endif

    // 10Hz monitor
    monitor_timer_ = this->create_wall_timer(100ms, std::bind(&FrontierExplorer::monitorObjectPursuit, this));
  }

private:
  struct FrontierCluster {
    std::vector<int> indices;
    double cx=0, cy=0;
    double gx=0, gy=0;   // frontier cell closest to the centroid (the centroid itself may be inside a wall)
    int size() const { return static_cast<int>(indices.size()); }
  };

  // ===== Occupancy callback =====
  void onOccupancy(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // YOLO 추격 중이거나 Nav2 goal active면 프론티어 계산/전송 금지
    checkGoalTimeout();
    if (pursuing_object_ || goal_active_) return;
    processAndPlan([&](){ return extractFromOccupancy(*msg); });
  }

  // ===== Costmap callback =====
  void onCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg) {
    checkGoalTimeout();
    if (pursuing_object_ || goal_active_) return;
    static bool once=false;
    if(!once){
      RCLCPP_INFO(get_logger(), "Costmap size: %u x %u, res=%.3f, origin=(%.2f,%.2f)",
        msg->metadata.size_x, msg->metadata.size_y, msg->metadata.resolution,
        msg->metadata.origin.position.x, msg->metadata.origin.position.y);
      once=true;
    }
    processAndPlan([&](){ return extractFromCostmap(*msg); });
  }

  // ===== Common pipeline: TF → clustering → scoring → send goal =====
  template <typename Extractor>
  void processAndPlan(Extractor extractor){
    if (pursuing_object_) return;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s->%s not ready: %s", map_frame_.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;
    const double r_yaw = yawFromQuat(tf.transform.rotation);

    auto clusters = extractor();
    if (clusters.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "No frontiers.");
      return;
    }

    double best_score = -1e18;
    const FrontierCluster *best = nullptr;
    for (auto &c : clusters) {
      if (inBlacklist(c.gx, c.gy)) continue;
      double ig = static_cast<double>(c.size());
      double dx = c.gx - rx, dy = c.gy - ry;
      double dist = std::hypot(dx, dy);
      double yaw_to_goal = std::atan2(dy, dx);
      double dyaw = angleDistance(r_yaw, yaw_to_goal);
      double risk = 0.0;
      // 간단한 스코어 (원문과 동일 컨셉, distance_scale_ 사용 가능)
      const double dist_penalty = beta_ * (std::exp(dist / std::max(1e-3, distance_scale_)) - 1.0);
      double score = alpha_ * ig - dist_penalty - gamma_ * std::abs(dyaw) - lambda_ * risk;
      if (score > best_score) { best_score = score; best = &c; }
    }
    if (!best) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "All %zu frontiers are blacklisted. Exploration complete.", clusters.size());
      return;
    }

    const double tnow = now().seconds();
    if (tnow - last_send_goal_sec_ < send_goal_interval_s_) return;
    last_send_goal_sec_ = tnow;

    RCLCPP_INFO(get_logger(), "Frontier goal (%.2f, %.2f): %d cells, score %.2f, %zu candidates",
      best->gx, best->gy, best->size(), best_score, clusters.size());
    sendGoal(best->gx, best->gy, rx, ry);
  }

#ifdef HAVE_YOLO_MSGS
  // ===== YOLO detections → confirm → preempt frontier → send object goal =====
  void onDetections(const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
    if (object_goal_.empty()) return;  // 대상 클래스 미설정
    const double tnow = now().seconds();

    size_t matches = 0;
    geometry_msgs::msg::Point det_pos_map{};
    bool have_pos=false;

    for (const auto &det : msg->detections) {
      if (!det.class_name.empty() && det.class_name == object_goal_ && det.score >= min_score_) {
        geometry_msgs::msg::PointStamped ps_in, ps_out;
        ps_in.header.stamp = now();
        ps_in.header.frame_id = det.bbox3d.frame_id.empty() ? map_frame_ : det.bbox3d.frame_id;
        ps_in.point = det.bbox3d.center.position;
        try {
          tf_buffer_.transform(ps_in, ps_out, map_frame_, 100ms);
          det_pos_map = ps_out.point;
          have_pos = true;
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Transform %s->%s failed for detection: %s",
            ps_in.header.frame_id.c_str(), map_frame_.c_str(), e.what());
        }
        ++matches;
      }
    }

    if (matches > 0) {
      last_object_seen_sec_ = tnow;
      if (have_pos) last_object_pos_ = det_pos_map;
      if (!object_confirmed_ && first_seen_start_sec_ == 0.0)
        first_seen_start_sec_ = tnow;
    } else {
      first_seen_start_sec_ = 0.0;
      object_confirmed_ = false;
    }

    // 지속 관측으로 객체 확정
    if (!object_confirmed_ && first_seen_start_sec_ > 0.0 &&
        (tnow - first_seen_start_sec_) >= object_persistence_s_) {
      object_confirmed_ = true;

      if (!have_pos) {
        RCLCPP_INFO(get_logger(), "Object '%s' confirmed (no position).", object_goal_.c_str());
        return;
      }

      // ★ Frontier 즉시 중단 (race condition 방지: sendGoal보다 먼저 세팅)
      pursuing_object_ = true;

      // Nav2에 보낼 goal 계산
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "TF %s->%s not ready for object approach: %s",
                    map_frame_.c_str(), base_frame_.c_str(), e.what());
        pursuing_object_ = false; // 실패 시 복구
        return;
      }

      const double rx = tf.transform.translation.x;
      const double ry = tf.transform.translation.y;
      const double ox = last_object_pos_.x;
      const double oy = last_object_pos_.y;
      const double dx = ox - rx;
      const double dy = oy - ry;
      const double dist = std::hypot(dx, dy);

      if (dist < 1e-3) {
        RCLCPP_WARN(get_logger(), "Object at robot position; not sending goal.");
        pursuing_object_ = false;
        return;
      }

      // 객체 전방 여유거리(standoff)를 남기고 접근
      const double ux = dx / dist;
      const double uy = dy / dist;
      const double d_goal = std::max(dist - dis_goal_m_, 0.0);
      if (d_goal < 1e-3) {
        RCLCPP_INFO(get_logger(), "Object within standoff (%.2fm). Skipping object goal.", dis_goal_m_);
        pursuing_object_ = false;
        return;
      }
      const double gx = rx + d_goal * ux;
      const double gy = ry + d_goal * uy;

      // 기존 goal이 있으면 프리엠션
      if (goal_active_) {
        if (allow_preempt_) {
          cancelCurrentGoal();   // 즉시 goal_active_=false 처리
        } else {
          RCLCPP_INFO(get_logger(), "Goal active and preemption disabled. Skipping object goal.");
          pursuing_object_ = false;
          return;
        }
      }

      // 전송 주기 제한
      if (tnow - last_send_goal_sec_ < send_goal_interval_s_) return;
      last_send_goal_sec_ = tnow;

      sendGoal(gx, gy, rx, ry, true);

      // 재전송 방지
      first_seen_start_sec_ = 0.0;

      RCLCPP_INFO(get_logger(),
        "Object '%s' confirmed (%.2fs). Pursuing: goal=(%.2f, %.2f), standoff=%.2fm",
        object_goal_.c_str(), object_persistence_s_, gx, gy, dis_goal_m_);
    } else if (matches > 0) {
      const double remain = std::max(0.0, object_persistence_s_ - (tnow - first_seen_start_sec_));
      RCLCPP_DEBUG(get_logger(), "Seeing '%s' (matches=%zu, score>=%.2f). Need %.2fs more to confirm.",
        object_goal_.c_str(), matches, min_score_, remain);
    }
  }
#endif

  // ===== 객체 추격 상태 감시(정지/재개) =====
  void monitorObjectPursuit() {
    if (!pursuing_object_) return;

    const double tnow = now().seconds();

    // 객체 유실 시 프론티어 재개
    if ((tnow - last_object_seen_sec_) > object_lost_resume_s_) {
      RCLCPP_INFO(get_logger(), "Object lost for %.2fs. Resuming frontier.", object_lost_resume_s_);
      if (goal_active_) cancelCurrentGoal();
      pursuing_object_ = false;
      object_confirmed_ = false;
      current_goal_is_object_ = false;
      return;
    }

    // 최신 객체 위치와 로봇 위치를 이용해 정지 조건 검사
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const std::exception&) {
      return; // TF 준비 안되면 다음 주기
    }
    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;

    const double ox = last_object_pos_.x;  // map 좌표
    const double oy = last_object_pos_.y;
    const double dist = std::hypot(ox - rx, oy - ry);

    if (dist <= stop_distance_m_) {
      RCLCPP_INFO(get_logger(), "Reached stop radius (%.2fm) to object. Cancelling goal and stopping.", stop_distance_m_);
      if (goal_active_) cancelCurrentGoal();
      pursuing_object_ = false;
      object_confirmed_ = false;
      current_goal_is_object_ = false;
    }
  }

  // ===== Frontier 추출 (OccupancyGrid) =====
  std::vector<FrontierCluster> extractFromOccupancy(const nav_msgs::msg::OccupancyGrid &map) {
    const int w = map.info.width, h = map.info.height;
    const auto &data = map.data;
    std::vector<uint8_t> is_frontier(w*h, 0);

    auto idx = [w](int x, int y){ return y*w + x; };
    auto inb = [w,h](int x, int y){ return (0<=x && x<w && 0<=y && y<h); };

    for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
      int i = idx(x,y);
      if (data[i] != -1) continue; // unknown only
      bool has_free = false;
      const int nx[4] = {1,-1,0,0};
      const int ny[4] = {0,0,1,-1};
      for (int k=0;k<4;++k){
        int xx=x+nx[k], yy=y+ny[k];
        if (!inb(xx,yy)) continue;
        int j = idx(xx,yy);
        if (data[j] >= 0 && data[j] <= free_occ_max_) { has_free = true; break; }
      }
      if (has_free) is_frontier[i] = 1;
    }
    return clusterAndCentroid(is_frontier, w, h,
      [&](int gx, int gy, double &wx, double &wy){
        gridToWorld(map, gx, gy, wx, wy);
      });
  }

  // ===== Frontier 추출 (Nav2 Costmap) =====
  std::vector<FrontierCluster> extractFromCostmap(const nav2_msgs::msg::Costmap &cm) {
    const int w = static_cast<int>(cm.metadata.size_x);
    const int h = static_cast<int>(cm.metadata.size_y);
    const auto &data = cm.data;
    std::vector<uint8_t> is_frontier(w*h, 0);

    auto idx = [w](int x, int y){ return y*w + x; };
    auto inb = [w,h](int x, int y){ return (0<=x && x<w && 0<=y && y<h); };

    for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
      int i = idx(x,y);
      if (data[i] != unknown_cost_) continue; // unknown only (255)
      bool has_free = false;
      const int nx[4] = {1,-1,0,0};
      const int ny[4] = {0,0,1,-1};
      for (int k=0;k<4;++k){
        int xx=x+nx[k], yy=y+ny[k];
        if (!inb(xx,yy)) continue;
        int j = idx(xx,yy);
        if (data[j] <= free_cost_max_) { has_free = true; break; }
      }
      if (has_free) is_frontier[i] = 1;
    }
    return clusterAndCentroid(is_frontier, w, h,
      [&](int gx, int gy, double &wx, double &wy){
        gridToWorld(cm, gx, gy, wx, wy);
      });
  }

  // ===== 공통: 클러스터링 + 무게중심 =====
  template <typename ToWorldFn>
  std::vector<FrontierCluster> clusterAndCentroid(const std::vector<uint8_t> &is_frontier,
                                                  int w, int h, ToWorldFn toWorld) {
    auto idx = [w](int x, int y){ return y*w + x; };
    auto inb = [w,h](int x, int y){ return (0<=x && x<w && 0<=y && y<h); };

    std::vector<uint8_t> vis(w*h, 0);
    std::vector<FrontierCluster> clusters;
    const int K = connectivity8_ ? 8 : 4;
    const int dx8[8] = {1,1,1,-1,-1,-1,0,0};
    const int dy8[8] = {1,0,-1,1,0,-1,1,-1};
    const int dx4[4] = {1,-1,0,0};
    const int dy4[4] = {0,0,1,-1};
    const int *DX = connectivity8_ ? dx8 : dx4;
    const int *DY = connectivity8_ ? dy8 : dy4;

    for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
      int s = idx(x,y);
      if (!is_frontier[s] || vis[s]) continue;
      FrontierCluster c;
      std::queue<int> q; q.push(s); vis[s]=1;
      while(!q.empty()){
        int v = q.front(); q.pop();
        c.indices.push_back(v);
        int vx = v % w, vy = v / w;
        for (int k=0;k<K;++k){
          int xx=vx+DX[k], yy=vy+DY[k];
          if(!inb(xx,yy)) continue;
          int u = idx(xx,yy);
          if(is_frontier[u] && !vis[u]){ vis[u]=1; q.push(u); }
        }
      }
      if (c.size() >= min_frontier_size_) {
        double sx=0.0, sy=0.0;
        for (int v : c.indices){
          int vx = v % w, vy = v / w;
          double wx, wy; toWorld(vx, vy, wx, wy);
          sx += wx; sy += wy;
        }
        c.cx = sx / c.size();
        c.cy = sy / c.size();
        double best_d2 = 1e18;
        for (int v : c.indices){
          double wx, wy; toWorld(v % w, v / w, wx, wy);
          const double d2 = (wx-c.cx)*(wx-c.cx) + (wy-c.cy)*(wy-c.cy);
          if (d2 < best_d2) { best_d2 = d2; c.gx = wx; c.gy = wy; }
        }
        clusters.push_back(c);
      }
    }
    return clusters;
  }

  // ===== grid→world 변환(OccupancyGrid) =====
  void gridToWorld(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy,
                   double &wx, double &wy) const {
    const double res = map.info.resolution;
    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;
    wx = ox + (gx + 0.5) * res;
    wy = oy + (gy + 0.5) * res;
  }

  // ===== grid→world 변환(Costmap) =====
  void gridToWorld(const nav2_msgs::msg::Costmap &cm, int gx, int gy,
                   double &wx, double &wy) const {
    const double res = cm.metadata.resolution;
    const double ox  = cm.metadata.origin.position.x;
    const double oy  = cm.metadata.origin.position.y;
    wx = ox + (gx + 0.5) * res;
    wy = oy + (gy + 0.5) * res;
  }

  // ===== 유틸 =====
  double yawFromQuat(const geometry_msgs::msg::Quaternion &q) const {
    double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
    double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double angleDistance(double a, double b) const {
    double d = std::fmod(b - a + M_PI, 2*M_PI);
    if (d < 0) d += 2*M_PI;
    return d - M_PI;
  }

  bool inBlacklist(double x, double y) const {
    for (const auto &p : blacklist_) {
      double dx = x - p.first, dy = y - p.second;
      if (std::hypot(dx,dy) < blacklist_radius_m_) return true;
    }
    return false;
  }

  // ===== Nav2 goal =====
  void sendGoal(double gx, double gy, double rx, double ry, bool is_object_goal=false) {
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not ready.");
      return;
    }
    auto goal = NavigateToPose::Goal();
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = map_frame_;
    goal.pose.pose.position.x = gx;
    goal.pose.pose.position.y = gy;

    double yaw = std::atan2(gy - ry, gx - rx);
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(yaw*0.5);
    goal.pose.pose.orientation.w = std::cos(yaw*0.5);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback = [this, is_object_goal](GoalHandle::SharedPtr handle){
      if (!handle) {
        RCLCPP_WARN(this->get_logger(), "Goal rejected by server.");
      } else {
        goal_active_ = true;
        goal_start_sec_ = this->now().seconds();
        current_goal_handle_ = handle;
        current_goal_is_object_ = is_object_goal;
        RCLCPP_INFO(this->get_logger(), "Goal accepted.");
      }
    };
    opts.result_callback = [this, gx, gy](const GoalHandle::WrappedResult & result){
      goal_active_ = false;
      current_goal_handle_.reset();
      pursuing_object_ = false;
      current_goal_is_object_ = false;

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Goal reached.");
        if (blacklist_reached_) blacklist_.emplace_back(gx, gy);
      } else {
        RCLCPP_WARN(this->get_logger(), "Goal failed. Blacklisting this area.");
        blacklist_.emplace_back(gx, gy);
      }
    };
    nav_client_->async_send_goal(goal, opts);
  }

  // a cancelled goal ends in the result callback as not SUCCEEDED, which blacklists it
  void checkGoalTimeout() {
    if (!goal_active_ || current_goal_is_object_ || goal_timeout_s_ <= 0.0) return;
    if (now().seconds() - goal_start_sec_ < goal_timeout_s_) return;
    RCLCPP_WARN(get_logger(), "Frontier goal not reached in %.0f s.", goal_timeout_s_);
    if (current_goal_handle_) nav_client_->async_cancel_goal(current_goal_handle_);
    goal_start_sec_ = now().seconds();   // don't re-cancel on every map until the result arrives
  }

  void cancelCurrentGoal() {
    if (current_goal_handle_) {
      RCLCPP_INFO(get_logger(), "Cancelling current goal...");
      nav_client_->async_cancel_goal(current_goal_handle_);
    }
    // preemption 중 재진입 방지
    goal_active_ = false;
  }

  // === Members ===
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr       costmap_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  // YOLO detections subscription (state)
#ifdef HAVE_YOLO_MSGS
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr det_sub_;
#endif
  double last_object_seen_sec_ = 0.0;
  double first_seen_start_sec_ = 0.0;
  bool   object_confirmed_ = false;
  geometry_msgs::msg::Point last_object_pos_{};
  std::string last_object_frame_{};

  // Nav2 goal management
  GoalHandle::SharedPtr current_goal_handle_;
  bool current_goal_is_object_ = false;
  bool pursuing_object_ = false;

  std::vector<std::pair<double,double>> blacklist_;
  bool   goal_active_ = false;

  std::string map_topic_;
  std::string map_frame_;
  std::string base_frame_;

  int    min_frontier_size_;
  bool   connectivity8_;
  double alpha_, beta_, gamma_, lambda_;
  double distance_scale_;
  double blacklist_radius_m_, send_goal_interval_s_;
  bool   blacklist_reached_;
  double goal_timeout_s_;
  double goal_start_sec_ = 0.0;
  bool   use_sim_time_;
  double last_send_goal_sec_;

  int    free_occ_max_;   // 0..100
  int    free_cost_max_;  // 0..255
  int    unknown_cost_;   // 255

  // === Params for object-goal fusion ===
  std::string object_goal_;
  std::string detections_topic_;
  double object_persistence_s_{};
  double min_score_{};
  double dis_goal_m_{};
  bool   allow_preempt_{};
  double stop_distance_m_{};
  double object_lost_resume_s_{};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
