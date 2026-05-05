//
// Copyright 2021 mc_rtc development team
//

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <regex>

namespace
{

geometry_msgs::msg::Pose svaToPose(const sva::PTransformd & t)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d & p = t.translation();
  // SVA rotation() returns body-to-world as its transpose
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

class ConvexVisualizationNode : public rclcpp::Node
{
public:
  ConvexVisualizationNode() : rclcpp::Node("mc_convex_visualization")
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

    RCLCPP_INFO(this->get_logger(), "Robot %s loaded with %zu convexes", robot.name().c_str(),
                robot.convexes().size());

    publishRobotDescription(rm->urdf_path);
    publishStaticTF(robot, frame_id);

    buildMarkers(robot, frame_id);

    auto qos = rclcpp::QoS(1).transient_local();
    pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("convex_markers", qos);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), [this]() { pub_->publish(markers_); });
  }

private:
  void buildMarkers(const mc_rbdyn::Robot & robot, const std::string & frame_id)
  {
    int id = 0;
    for(const auto & [name, convex_pair] : robot.convexes())
    {
      const auto & body = convex_pair.first;
      const auto & sch_obj = convex_pair.second;
      sva::PTransformd pose = robot.collisionTransform(name) * robot.bodyPosW(body);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.ns = "convex";
      marker.id = id++;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
      // Default lifetime 0 = forever
      marker.lifetime = rclcpp::Duration(0, 0);

      if(auto * poly = dynamic_cast<sch::S_Polyhedron *>(sch_obj.get()))
      {
        addPolyhedron(marker, *poly, pose);
      }
      else if(auto * box = dynamic_cast<sch::S_Box *>(sch_obj.get()))
      {
        addBox(marker, *box, pose);
      }
      else if(auto * cylinder = dynamic_cast<sch::S_Cylinder *>(sch_obj.get()))
      {
        addCylinder(marker, *cylinder, pose);
      }
      else if(auto * sphere = dynamic_cast<sch::S_Sphere *>(sch_obj.get()))
      {
        addSphere(marker, *sphere, pose);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Convex %s: unsupported shape type, skipping", name.c_str());
        continue;
      }

      markers_.markers.push_back(marker);
    }

    RCLCPP_INFO(this->get_logger(), "Built %zu markers", markers_.markers.size());
  }

  void addPolyhedron(visualization_msgs::msg::Marker & marker, sch::S_Polyhedron & poly,
                     const sva::PTransformd & pose)
  {
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // Pose is identity — vertices are pre-transformed to world frame
    marker.pose.orientation.w = 1.0;

    const auto & sch_vertices = poly.getPolyhedronAlgorithm()->vertexes_;
    const auto & sch_triangles = poly.getPolyhedronAlgorithm()->triangles_;

    // Pre-transform vertices to world frame
    std::vector<Eigen::Vector3d> world_vertices;
    world_vertices.reserve(sch_vertices.size());
    for(const auto * v : sch_vertices)
    {
      const auto & c = v->getCoordinates();
      Eigen::Vector3d local{c.m_x, c.m_y, c.m_z};
      world_vertices.push_back((sva::PTransformd{local} * pose).translation());
    }

    // Build triangle list with correct winding order (matching RobotConvex.cpp)
    for(const auto & t : sch_triangles)
    {
      const auto a_coord = sch_vertices[t.a]->getCoordinates();
      const auto b_coord = sch_vertices[t.b]->getCoordinates();
      const auto c_coord = sch_vertices[t.c]->getCoordinates();
      auto cross = (a_coord - b_coord) ^ (a_coord - c_coord);
      auto dot = t.normal * cross;

      if(dot < 0)
      {
        marker.points.push_back(eigenToPoint(world_vertices[t.c]));
        marker.points.push_back(eigenToPoint(world_vertices[t.b]));
        marker.points.push_back(eigenToPoint(world_vertices[t.a]));
      }
      else
      {
        marker.points.push_back(eigenToPoint(world_vertices[t.a]));
        marker.points.push_back(eigenToPoint(world_vertices[t.b]));
        marker.points.push_back(eigenToPoint(world_vertices[t.c]));
      }
    }
  }

  void addBox(visualization_msgs::msg::Marker & marker, sch::S_Box & box, const sva::PTransformd & pose)
  {
    marker.type = visualization_msgs::msg::Marker::CUBE;
    double x, y, z;
    box.getBoxParameters(x, y, z);
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
    marker.pose = svaToPose(pose);
  }

  void addCylinder(visualization_msgs::msg::Marker & marker, sch::S_Cylinder & cylinder,
                   const sva::PTransformd & pose)
  {
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    double radius = cylinder.getRadius();
    auto p1 = cylinder.getP1();
    auto p2 = cylinder.getP2();
    double length = std::sqrt((p2.m_x - p1.m_x) * (p2.m_x - p1.m_x) + (p2.m_y - p1.m_y) * (p2.m_y - p1.m_y)
                              + (p2.m_z - p1.m_z) * (p2.m_z - p1.m_z));
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = length;
    marker.pose = svaToPose(pose);
  }

  void addSphere(visualization_msgs::msg::Marker & marker, sch::S_Sphere & sphere,
                 const sva::PTransformd & pose)
  {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    double r = sphere.getRadius();
    marker.scale.x = 2.0 * r;
    marker.scale.y = 2.0 * r;
    marker.scale.z = 2.0 * r;
    marker.pose = svaToPose(pose);
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

    // Resolve relative mesh paths to absolute file:// URIs
    std::string urdf_dir = std::filesystem::path(urdf_path).parent_path().string();
    std::regex mesh_regex(R"_(filename\s*=\s*"([^"]+)")_");
    std::string result;
    std::sregex_iterator it(urdf_content.begin(), urdf_content.end(), mesh_regex);
    std::sregex_iterator end;
    size_t last_pos = 0;

    for(; it != end; ++it)
    {
      auto & match = *it;
      std::string path = match[1].str();
      result.append(urdf_content, last_pos, match.position() - last_pos);

      if(path.find("://") == std::string::npos)
      {
        std::filesystem::path abs_path = std::filesystem::weakly_canonical(std::filesystem::path(urdf_dir) / path);
        result += "filename=\"file://" + abs_path.string() + "\"";
      }
      else
      {
        result += match[0].str();
      }
      last_pos = match.position() + match[0].length();
    }
    result.append(urdf_content, last_pos, std::string::npos);

    auto desc_qos = rclcpp::QoS(1).transient_local();
    desc_pub_ = this->create_publisher<std_msgs::msg::String>("robot_description", desc_qos);
    std_msgs::msg::String msg;
    msg.data = result;
    desc_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published robot_description from %s (mesh paths resolved)", urdf_path.c_str());
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
  rclcpp::spin(std::make_shared<ConvexVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
