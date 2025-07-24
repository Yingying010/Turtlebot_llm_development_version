#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "phasespace_msgs/msg/camera.hpp"
#include "phasespace_msgs/msg/cameras.hpp"
#include "phasespace_msgs/msg/marker.hpp"
#include "phasespace_msgs/msg/markers.hpp"
#include "phasespace_msgs/msg/rigid.hpp"
#include "phasespace_msgs/msg/rigids.hpp"
#include "RSJparser.tcc"
#include "owl.hpp"
#include <cmath> 
using namespace std;

// for debugging purposes
inline void printInfo(const OWL::Markers &markers)
{
  for(OWL::Markers::const_iterator m = markers.begin(); m != markers.end(); m++)
    if(m->cond > 0)
      cout << "   " << m->id << ") pos=" << m->x << "," << m->y << "," << m->z << endl;
}

inline void printInfo(const OWL::Rigids &rigids)
{
  for(OWL::Rigids::const_iterator r = rigids.begin(); r != rigids.end(); r++)
    if(r->cond > 0)
      cout << "   " << r->id << ") pose=" << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
           << " " << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
           << endl;
}

float computeHeadingPitch(float qw, float qx, float qy, float qz) {
    double gimbal_eps = 1e-6;
    double n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

    if (n == 0.0) n = 1.0;
    qw /= n; qx /= n; qy /= n; qz /= n;

    double sinp = 2.0 * (qw*qy - qz*qx);
    sinp = std::clamp(sinp, -1.0, 1.0);
    double pitch_rad = std::asin(sinp);

    bool gimbal_lock = std::abs(std::abs(sinp) - 1.0) < gimbal_eps;

    auto to360 = [](double rad) -> float {
      double deg = rad * 180.0 / M_PI;
      deg = std::fmod(deg + 360.0, 360.0);
      return static_cast<float>(deg);
    };

    return to360(pitch_rad);

}



int main(int argc, char** argv)
{
  // initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("phasespace_client_node");
  // create the context
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Cameras cameras;
  OWL::Rigids rigids;
  // USE COMMAND LINE ARGUMENTS
  // get the owl server address through the command line
  // 'ros2 run phasespace_client phasespace_client_node 192.168.1.17 drone.json 1'
  // getting the address of the owl server
  // string address =  argc > 1 ? argv[1] : "192.168.1.17";
  string address =  argc > 1 ? argv[1] : "192.168.1.2";
  // getting the name of the json file
  // string json_name = argc > 2 ? argv[2] : "none.json";
  string json_name = argc > 2 ? argv[2] : "/home/ubuntu2204/ros2_ws/src/phasespace_client/rigid_body_objects/Mona7_turtlebot_blue.json";
  if(json_name == "none.json") {
    cout << "No json file provided. Exiting..." << endl;
    return 0;
  }
  // id of the body to track
  // string desired_body_to_track = argc > 3 ? argv[3] : '1';
  // int desired_body_to_track = argc > 3 ? std::stoi(argv[3]) : 1;
  std::map<int, std::string> rigid_id_to_name;
  // cout << "desired_body_to_track: " << desired_body_to_track << endl;
  // build path to json file which are in the rigid_objects folder
  std::string package_path = ament_index_cpp::get_package_share_directory("phasespace_client");
  string json_path = "/home/ubuntu2204/ros2_ws/src/phasespace_client/rigid_body_objects/Mona7_turtlebot_blue.json";
  std::cout << "json_path: " << json_path << std::endl;
  // read the json file
  ifstream json_file(json_path);
  string json_file_str((istreambuf_iterator<char>(json_file)), istreambuf_iterator<char>());
  cout << "json_file_str: " << json_file_str << endl;
  RSJresource my_json(json_file_str); // str is a string containing the json file
  cout << "my_json: " << my_json.as_str() << endl;

    
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr errorsPub = node->create_publisher<std_msgs::msg::String>("phasespace_errors", 10);
  rclcpp::Publisher<phasespace_msgs::msg::Cameras>::SharedPtr camerasPub = node->create_publisher<phasespace_msgs::msg::Cameras>("phasespace_cameras", 10);
  rclcpp::Publisher<phasespace_msgs::msg::Markers>::SharedPtr markersPub = node->create_publisher<phasespace_msgs::msg::Markers>("phasespace_markers", 10);
  rclcpp::Publisher<phasespace_msgs::msg::Rigids>::SharedPtr rigidsPub = node->create_publisher<phasespace_msgs::msg::Rigids>("phasespace_rigids", 10);
  rclcpp::Publisher<phasespace_msgs::msg::Rigid>::SharedPtr rigid_one_Pub = node->create_publisher<phasespace_msgs::msg::Rigid>("/phasespace_body_one", 10);
  std::map<int, rclcpp::Publisher<phasespace_msgs::msg::Rigid>::SharedPtr> rigid_pubs;
  // simple example
  if (owl.open(address) <= 0) {
    RCLCPP_ERROR(node->get_logger(), "❌ OWL open() failed. Could not connect to OWL server at %s", address.c_str());
    return 0;
  } else {
    RCLCPP_INFO(node->get_logger(), "✅ owl.open() success.");
  }
  
  if (owl.initialize("timebase=1,1000000") <= 0) {
    RCLCPP_ERROR(node->get_logger(), "❌ OWL initialize() failed.");
    return 0;
  } else {
    RCLCPP_INFO(node->get_logger(), "✅ owl.initialize() success.");
  }
  
  owl.streaming(1);
  RCLCPP_INFO(node->get_logger(), "🟡 Called owl.streaming(1)");
  RCLCPP_INFO(node->get_logger(), "🟢 OWL streaming state: %d", owl.streaming());

  // create rigid bodies
  for (auto ii = my_json["trackers"].as_array().begin(); ii != my_json["trackers"].as_array().end(); ++ii) {
      uint32_t tracker_id = (*ii)["id"].as<int>();
      std::string tracker_name = (*ii)["name"].as<std::string>();
      rigid_id_to_name[tracker_id] = tracker_name;

      bool created = owl.createTracker(tracker_id, "rigid", tracker_name);
      if (!created)
          std::cerr << "❌ Failed to create tracker " << tracker_name << " (" << tracker_id << ")" << std::endl;

      for (auto it = (*ii)["markers"].as_array().begin(); it != (*ii)["markers"].as_array().end(); ++it) {
          uint32_t marker_id = (*it)["id"].as<int>();
          std::string marker_name = (*it)["name"].as<std::string>();
          std::string marker_pos = (*it)["options"].as<std::string>();

          bool ok = owl.assignMarker(tracker_id, marker_id, marker_name, marker_pos);
          if (!ok) {
              std::cerr << "❌ Failed to assign marker " << marker_id << " to tracker " << tracker_id << std::endl;
          }
      }
  }

  // start streaming
  owl.streaming(1);
  RCLCPP_INFO(node->get_logger(), "OWL streaming state: %d", owl.streaming(1));


  std::cout << "🌐 Trackers in system: " << owl.property("trackers").str() << std::endl;

  // main loop
  while (rclcpp::ok() && owl.isOpen() && owl.property<int>("initialized")) {
    const OWL::Event *event = owl.nextEvent(1000);
    if (!event) continue;

    if (event->type_id() == OWL::Type::ERROR) {
      cerr << event->name() << ": " << event->str() << endl;
      std_msgs::msg::String str;
      str.data = event->str();
      errorsPub->publish(str);
    }
    else if (event->type_id() == OWL::Type::CAMERA) {
      if (event->name() == string("cameras") && event->get(cameras) > 0) {
        phasespace_msgs::msg::Cameras out;
        for (OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++) {
          phasespace_msgs::msg::Camera cam;
          cam.id = c->id;
          cam.flags = c->flags;
          cam.x = c->pose[0];
          cam.y = c->pose[1];
          cam.z = c->pose[2];
          cam.qw = c->pose[3];
          cam.qx = c->pose[4];
          cam.qy = c->pose[5];
          cam.qz = c->pose[6];
          cam.cond = c->cond;
          out.cameras.push_back(cam);
        }
        camerasPub->publish(out);
      }
    }
    else if (event->type_id() == OWL::Type::FRAME) {  
      if (event->find("markers", markers) > 0) {
        phasespace_msgs::msg::Markers out;
        for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++) {
          if (m->cond > 0) {
            phasespace_msgs::msg::Marker mout;
            mout.id = m->id;
            mout.time = m->time;
            mout.flags = m->flags;
            mout.cond = m->cond;
            mout.x = m->x;
            mout.y = m->y;
            mout.z = m->z;
            out.markers.push_back(mout);
          }
        }
        markersPub->publish(out);
      }
      
      if (event->find("rigids", rigids) > 0) {
        phasespace_msgs::msg::Rigids out;

        for (OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++) {
          phasespace_msgs::msg::Rigid rout;
          rout.id = r->id;
          rout.time = r->time;
          rout.flags = r->flags;
          rout.cond = r->cond;
          rout.x = r->pose[0];
          rout.y = r->pose[1];
          rout.z = r->pose[2];
          rout.qw = r->pose[3];
          rout.qx = r->pose[4];
          rout.qy = r->pose[5];
          rout.qz = r->pose[6];

          float heading_y = computeHeadingPitch(r->pose[3], r->pose[4], r->pose[5], r->pose[6]);

          rout.heading_y = heading_y;

          // 构造 topic 名称（如 /mona5/phasespace_body）
          std::string topic_name;
          if (rigid_id_to_name.count(r->id)) {
            topic_name = "/phasespace_body_"+ rigid_id_to_name[r->id];
          } else {
            topic_name = "/phasespace_body_" + std::to_string(r->id);
          }

          // 若尚未创建 publisher，则创建
          if (rigid_pubs.find(r->id) == rigid_pubs.end()) {
            rigid_pubs[r->id] = node->create_publisher<phasespace_msgs::msg::Rigid>(topic_name, 10);
            std::cout << "✅ Created topic: " << topic_name << std::endl;
          }

          // 发布该 rigid 数据
          rigid_pubs[r->id]->publish(rout);
          out.rigids.push_back(rout);
        }

        // 同时也发布汇总信息
        rigidsPub->publish(out);
      }

    }
  }

  owl.done();
  owl.close();

  return 0;
}
