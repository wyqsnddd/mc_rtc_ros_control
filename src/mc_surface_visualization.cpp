//
// Copyright 2021 mc_rtc development team
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/logging.h>

#include <Eigen/Geometry>

#include <fstream>

namespace
{

geometry_msgs::msg::Pose svaToPose(const sva::PTransformd & t)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d & p = t.translation();
  Eigen::Quaterniond q(t.rotation().transpose());
  q.normalize();
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}

geometry_msgs::msg::Point eigenToPoint(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

} // namespace

class SurfaceVisualizationNode : public rclcpp::Node
{
public:
  SurfaceVisualizationNode() : rclcpp::Node("mc_surface_visualization")
  {
    this->declare_parameter<std::string>("robot", "JVRC1");
    this->declare_parameter<std::string>("frame_id", "map");

    auto robot_name = this->get_parameter("robot").as_string();
    auto frame_id = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "Loading robot module: %s", robot_name.c_str());

    auto rm = mc_rbdyn::RobotLoader::get_robot_module({robot_name});
    robots_ = mc_rbdyn::loadRobot(*rm);
    auto & robot = robots_->robot();
    robot.forwardKinematics();

    auto surface_names = robot.availableSurfaces();
    RCLCPP_INFO(this->get_logger(), "Robot %s loaded with %zu surfaces", robot.name().c_str(), surface_names.size());

    // Publish URDF for RViz2 RobotModel display
    publishRobotDescription(rm->urdf_path);

    // Publish static TF for all robot bodies
    publishStaticTF(robot, frame_id);

    buildMarkers(robot, surface_names, frame_id);

    auto qos = rclcpp::QoS(1).transient_local();
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("surface_markers", qos);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), [this]() { pub_->publish(markers_); });
  }

private:
  void buildMarkers(const mc_rbdyn::Robot & robot, const std::vector<std::string> & surface_names,
                    const std::string & frame_id)
  {
    int id = 0;
    for(const auto & name : surface_names)
    {
      const auto & surface = robot.surface(name);
      sva::PTransformd pose = robot.surfacePose(name);

      if(surface.type() == "planar")
      {
        addPlanarSurface(id, dynamic_cast<const mc_rbdyn::PlanarSurface &>(surface), pose, frame_id, name);
      }
      else if(surface.type() == "cylindrical")
      {
        addCylindricalSurface(id, dynamic_cast<const mc_rbdyn::CylindricalSurface &>(surface), pose, frame_id, name);
      }
      else if(surface.type() == "gripper")
      {
        addGripperSurface(id, dynamic_cast<const mc_rbdyn::GripperSurface &>(surface), pose, frame_id, name);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Surface %s: unsupported type '%s', skipping", name.c_str(),
                    surface.type().c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Built %zu markers", markers_.markers.size());
  }

  void addPlanarSurface(int & id, const mc_rbdyn::PlanarSurface & surface, const sva::PTransformd & pose,
                        const std::string & frame_id, const std::string & name)
  {
    // Polygon outline
    visualization_msgs::msg::Marker polygon;
    polygon.header.frame_id = frame_id;
    polygon.ns = "surface_polygon";
    polygon.id = id++;
    polygon.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon.action = visualization_msgs::msg::Marker::ADD;
    polygon.scale.x = 0.01; // line width
    polygon.color.r = 0.0;
    polygon.color.g = 0.8;
    polygon.color.b = 0.0;
    polygon.color.a = 1.0;
    polygon.pose.orientation.w = 1.0;
    polygon.lifetime = rclcpp::Duration(0, 0);

    const auto & planar_points = surface.planarPoints();
    for(const auto & pp : planar_points)
    {
      Eigen::Vector3d world_pt =
          (sva::PTransformd{Eigen::Vector3d(pp.first, pp.second, 0.0)} * pose).translation();
      polygon.points.push_back(eigenToPoint(world_pt));
    }
    // Close the loop
    if(!planar_points.empty())
    {
      Eigen::Vector3d first_pt =
          (sva::PTransformd{Eigen::Vector3d(planar_points[0].first, planar_points[0].second, 0.0)} * pose)
              .translation();
      polygon.points.push_back(eigenToPoint(first_pt));
    }
    markers_.markers.push_back(polygon);

    // Normal arrow
    visualization_msgs::msg::Marker normal;
    normal.header.frame_id = frame_id;
    normal.ns = "surface_normal";
    normal.id = id++;
    normal.type = visualization_msgs::msg::Marker::ARROW;
    normal.action = visualization_msgs::msg::Marker::ADD;
    normal.scale.x = 0.01;  // shaft diameter
    normal.scale.y = 0.02;  // head diameter
    normal.scale.z = 0.05;  // head length
    normal.color.r = 0.0;
    normal.color.g = 0.0;
    normal.color.b = 1.0;
    normal.color.a = 1.0;
    normal.pose.orientation.w = 1.0;
    normal.lifetime = rclcpp::Duration(0, 0);

    Eigen::Vector3d start = pose.translation();
    Eigen::Vector3d end = (sva::PTransformd{Eigen::Vector3d(0, 0, 0.2)} * pose).translation();
    normal.points.push_back(eigenToPoint(start));
    normal.points.push_back(eigenToPoint(end));
    markers_.markers.push_back(normal);
  }

  void addCylindricalSurface(int & id, const mc_rbdyn::CylindricalSurface & surface, const sva::PTransformd & pose,
                             const std::string & frame_id, const std::string & name)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "surface_cylinder";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 2.0 * surface.radius();
    marker.scale.y = 2.0 * surface.radius();
    marker.scale.z = surface.width();
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.pose = svaToPose(pose);
    marker.lifetime = rclcpp::Duration(0, 0);
    markers_.markers.push_back(marker);
  }

  void addGripperSurface(int & id, const mc_rbdyn::GripperSurface & surface, const sva::PTransformd & pose,
                         const std::string & frame_id, const std::string & name)
  {
    for(const auto & p : surface.pointsFromOrigin())
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id;
      arrow.ns = "surface_gripper";
      arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.scale.x = 0.005; // shaft diameter
      arrow.scale.y = 0.01;  // head diameter
      arrow.scale.z = 0.025; // head length
      arrow.color.r = 0.0;
      arrow.color.g = 0.0;
      arrow.color.b = 1.0;
      arrow.color.a = 1.0;
      arrow.pose.orientation.w = 1.0;
      arrow.lifetime = rclcpp::Duration(0, 0);

      Eigen::Vector3d start = (p * pose).translation();
      // Arrow along contact normal (Z-axis of the contact point frame)
      Eigen::Matrix3d rotation = (p * pose).rotation().transpose();
      Eigen::Vector3d end = start + rotation.col(2) * 0.05;

      arrow.points.push_back(eigenToPoint(start));
      arrow.points.push_back(eigenToPoint(end));
      markers_.markers.push_back(arrow);
    }
  }

  void publishRobotDescription(const std::string & urdf_path)
  {
    std::ifstream ifs(urdf_path);
    if(!ifs.is_open())
    {
      RCLCPP_WARN(this->get_logger(), "Could not open URDF: %s", urdf_path.c_str());
      return;
    }
    std::string urdf_content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    auto desc_qos = rclcpp::QoS(1).transient_local();
    desc_pub_ = this->create_publisher<std_msgs::msg::String>("robot_description", desc_qos);
    std_msgs::msg::String msg;
    msg.data = urdf_content;
    desc_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published robot_description from %s", urdf_path.c_str());
  }

  void publishStaticTF(const mc_rbdyn::Robot & robot, const std::string & frame_id)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for(int i = 0; i < robot.mb().nrBodies(); ++i)
    {
      const auto & body_name = robot.mb().body(i).name();
      const auto & pose = robot.bodyPosW()[static_cast<size_t>(i)];

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = frame_id;
      t.child_frame_id = body_name;

      const Eigen::Vector3d & p = pose.translation();
      Eigen::Quaterniond q(pose.rotation().transpose());
      q.normalize();

      t.transform.translation.x = p.x();
      t.transform.translation.y = p.y();
      t.transform.translation.z = p.z();
      t.transform.rotation.w = q.w();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();

      transforms.push_back(t);
    }

    tf_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(this->get_logger(), "Published %zu static TF frames", transforms.size());
  }

  mc_rbdyn::RobotsPtr robots_;
  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr desc_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurfaceVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
