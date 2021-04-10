/**
 * @file drake_interface_node.cpp
 * @author Boston Cleek
 * @date 2021-02-16
 * @brief Interface Drake Physics with ROS
 *
 * @PARAMETERS:
 *
 * @PUBLISHES:
 *    joint_states (sensor_msgs/JointState) - joint names, positions, and velocities
 *    com_state (quadruped_msgs/CoMState) - COM pose and velocity twist in world frame
 * @SUBSCRIBES:
 *    joint_torque_cmd (quadruped_msgs/JointTorqueCmd) - joint torques
 * @SERVICES:
 *    start_position (std_srvs/Empty) - sets robot to starting configuration
 */

// TODO: publish actual reaction forces at feet and joint torques

// C++
#include <memory>
#include <filesystem>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

// Quadruped Control
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>

// Drake
#include <drake/systems/framework/framework_common.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/contact_results_to_lcm.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/manipulation/util/robot_plan_utils.h>

using drake::math::RigidTransformd;
using drake::systems::VectorBase;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

const static std::string LOGNAME = "drake_interface";
static bool joint_cmd_received = false;
static bool start_config_received = false;

// The joint torque map, maps joint actuator names to index in control vector.
static std::map<std::string, unsigned int> joint_torque_map;
// Control vector
static VectorXd tau;

void jointTorqueCallback(const quadruped_msgs::JointTorqueCmd::ConstPtr& msg)
{
  joint_cmd_received = true;
  if (msg->actuator_name.size() != msg->torque.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "The number of joint torque commands do not match "
                                    "the number of joint actuator in control message");
    return;
  }

  for (unsigned int i = 0; i < msg->actuator_name.size(); i++)
  {
    try
    {
      const auto index = joint_torque_map.at(msg->actuator_name.at(i));
      tau(index) = msg->torque.at(i);
      // std::cout << msg->actuator_name.at(i) << " " << index << std::endl;
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Actuator name does not exist in joint torque map");
    }
  }
  // std::cout << "Tau: \n" << tau << std::endl;
}

bool startConfigCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Resetting robot to starting configuration");
  start_config_received = true;
  return true;
}

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting drake interface node");
  ros::init(argc, argv, "drake_interface");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher com_pub = nh.advertise<quadruped_msgs::CoMState>("com_state", 1);
  ros::Subscriber joint_torque_sub =
      nh.subscribe("joint_torque_cmd", 1, jointTorqueCallback);

  ros::ServiceServer start_server =
      nh.advertiseService("start_position", startConfigCallback);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Robot
  const auto urdf_path = pnh.param<std::string>("urdf_path", "../urdf/robot.urdf");
  ROS_INFO_NAMED(LOGNAME, "File path: %s", urdf_path.c_str());

  // Physics
  const auto time_step = pnh.param<double>("time_step", 0.001);
  const auto viz_time_step = pnh.param<double>("viz_time_step", 0.001);
  const auto static_friction = pnh.param<double>("static_friction", 1.0);
  const auto dynamic_friction = pnh.param<double>("dynamic_friction", 1.0);
  const auto penetration_allowance = pnh.param<double>("penetration_allowance", 0.001);
  // const auto stiction_tolerance = pnh.param<double>("stiction_tolerance", 0.001);
  const auto real_time_rate = pnh.param<double>("real_time_rate", 1.0);

  // Robot initial pose
  // See MultibodyPlant SetPositions() to set init joint positions
  std::vector<double> init_pose = { 0.0, 0.0, 0.0 };
  std::vector<double> init_orientation = { 0.0, 0.0, 0.0, 1.0 };
  pnh.getParam("initial_pose/postion", init_pose);
  pnh.getParam("initial_pose/orientation", init_orientation);

  // Robot kinematics
  const auto base_link_name = pnh.param<std::string>("links/base_link", "base_link");

  // Robot initial joints
  const auto num_joints =
      static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 0));
  std::vector<std::string> joint_names;
  std::vector<std::string> joint_actuator_names;
  std::vector<double> init_joint_positions;
  std::vector<double> init_joint_torques;
  pnh.getParam("joints/joint_names", joint_names);
  pnh.getParam("joints/joint_actuator_names", joint_actuator_names);
  pnh.getParam("joints/init_joint_positions", init_joint_positions);
  pnh.getParam("joints/init_joint_torques", init_joint_torques);

  if ((num_joints != joint_names.size()) || (num_joints != joint_actuator_names.size()) ||
      (num_joints != init_joint_positions.size()) ||
      (num_joints != init_joint_torques.size()))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Invalid joint configuration. Num(joints) = Num(joint motors) "
                           "= Num(joint positions) = Num(joint torques)");

    ROS_ERROR_NAMED(LOGNAME,
                    "Num(joints): %lu, Num(joint motors): %lu, Num(joint positions): "
                    "%lu, Num(joint torques): %lu ",
                    joint_names.size(), joint_actuator_names.size(),
                    init_joint_positions.size(), init_joint_torques.size());
  }

  for (unsigned int i = 0; i < num_joints; i++)
  {
    joint_torque_map.emplace(joint_actuator_names.at(i), i);
  }

  // Place base_link relative to world_body
  const Vector3d start_position(init_pose.data());
  const Quaterniond start_orientation(init_orientation.data());
  const RigidTransformd Twb(start_orientation, start_position);

  // Ground plane color
  const drake::Vector4<double> green(0.5, 1.0, 0.5, 1.0);

  // Consructs the system diagram
  drake::systems::DiagramBuilder<double> builder;

  // Adds a MultibodyPlant and a SceneGraph instance to a diagram builder,
  // connecting the geometry ports.
  auto pair = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<drake::multibody::MultibodyPlant<double>>(time_step));

  // Represents a robot in a tree of bodies
  drake::multibody::MultibodyPlant<double>& plant = pair.plant;

  // Parses SDF and URDF input files into a MultibodyPlant and (optionally) a SceneGraph.
  drake::multibody::Parser(&plant).AddModelFromFile(urdf_path);

  // Add model of the ground.
  // Registers geometry in a SceneGraph with a given geometry::Shape
  // to be used for visualization of a given body.
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               drake::geometry::HalfSpace(), "GroundVisualGeometry",
                               green);

  plant.set_penetration_allowance(penetration_allowance);

  // For a time-stepping model only static friction is used.
  const drake::multibody::CoulombFriction<double> ground_friction(static_friction,
                                                                  dynamic_friction);

  // Registers geometry in a SceneGraph with a given geometry::Shape
  // to be used for the contact modeling of a given body.
  plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                  drake::geometry::HalfSpace(), "GroundCollisionGeometry",
                                  ground_friction);

  // This method must be called after all elements in the model
  // (joints, bodies, force elements, constraints, etc.) are added
  //  and before any computations are performed.
  plant.Finalize();

  if (!plant.is_finalized())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "plant failed to finalize");
  }

  // Publish contact results to drake_visualizer.
  drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant);

  // Connects the newly added DrakeVisualizer to the given SceneGraph
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, pair.scene_graph);

  // Builds the Diagram
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  drake::systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  ROS_INFO_NAMED(LOGNAME, "Number of actuated joints: %i", plant.num_actuated_dofs());
  ROS_INFO_NAMED(LOGNAME, "Number of actuaters: %i", plant.num_actuators());
  ROS_INFO_NAMED(LOGNAME, "Number of generalized positions: %i", plant.num_positions());
  ROS_INFO_NAMED(LOGNAME, "Number of generalized velocities: %i", plant.num_velocities());

  // Acuator insput port
  const drake::systems::InputPort<double>& input_port = plant.get_actuation_input_port();
  // const drake::systems::OutputPort<double>& output_port = plant.get_state_output_port();
  // ROS_INFO_NAMED(LOGNAME, "Input port name: %s", input_port.get_name().c_str());
  // ROS_INFO_NAMED(LOGNAME, "Output port name: %s", output_port.get_name().c_str());

  // Set initial joint torque
  // VectorXd tau = VectorXd::Zero(num_joints);
  tau = Eigen::Map<VectorXd, Eigen::Unaligned>(init_joint_torques.data(),
                                               init_joint_torques.size());

  input_port.FixValue(&plant_context, tau);

  // Set initial positons
  // TODO: set init COM pose here too?
  Eigen::VectorBlock<drake::VectorX<double>> state_vec =
      plant_context.get_mutable_discrete_state(0).get_mutable_value();

  const VectorXd init_joint_vec = Eigen::Map<VectorXd, Eigen::Unaligned>(
      init_joint_positions.data(), init_joint_positions.size());

  state_vec.segment(7, init_joint_positions.size()) = init_joint_vec;

  // Set pose of base_link in the world
  const drake::multibody::Body<double>& base_link = plant.GetBodyByName(base_link_name);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, base_link, Twb);

  // Advancing the state of hybrid dynamic systems forward in time.
  drake::systems::Simulator simulator(*diagram, std::move(diagram_context));
  simulator.Initialize();
  simulator.set_target_realtime_rate(real_time_rate);
  // simulator.set_publish_every_time_step(true);
  // simulator.AdvanceTo(10.0);

  auto current_time = 0.0;
  while (nh.ok())
  {
    // TODO: which context to use? I think this is required to get the current trajectory context.
    const drake::systems::Context<double>& context = simulator.get_context();

    if (start_config_received)
    {
      // Reset initial joint positions
      Eigen::VectorBlock<drake::VectorX<double>> state_vec =
          plant_context.get_mutable_discrete_state(0).get_mutable_value();
      state_vec.segment(7, init_joint_positions.size()) = init_joint_vec;

      // Reset initial joint torques
      tau = Eigen::Map<VectorXd, Eigen::Unaligned>(init_joint_torques.data(),
                                                   init_joint_torques.size());
      input_port.FixValue(&plant_context, tau);

      // // Reset free body pose in world
      plant.SetFreeBodyPoseInWorldFrame(&plant_context, base_link, Twb);

      start_config_received = false;
    }

    if (joint_cmd_received)
    {
      input_port.FixValue(&plant_context, tau);
      joint_cmd_received = false;
    }

    // states
    // Orientation: qw, qx, qy, qz
    // Position: x, y, z
    // Joint positions
    // Angular velocity: rdot, pdot, ydot
    // Velocity vector: xdot, ydot, zdot
    // Joint velocity
    const drake::VectorX<double>& state_vector =
        context.get_discrete_state_vector().CopyToVector();

    ////////////////
    // Joint states
    std::vector<double> joint_positions(num_joints);
    VectorXd::Map(&joint_positions.at(0), num_joints) =
        state_vector.segment(7, num_joints);

    std::vector<double> joint_velocities(num_joints);
    VectorXd::Map(&joint_velocities.at(0), num_joints) = state_vector.tail(num_joints);

    // TODO: add effort to msg?
    sensor_msgs::JointState js_msg;
    js_msg.header.frame_id = "";
    js_msg.header.stamp = ros::Time::now();
    js_msg.name = joint_names;
    js_msg.position = joint_positions;
    js_msg.velocity = joint_velocities;

    joint_pub.publish(js_msg);

    ////////////////
    // CoM
    quadruped_msgs::CoMState com_msg;
    com_msg.pose.orientation.w = state_vector(0);
    com_msg.pose.orientation.x = state_vector(1);
    com_msg.pose.orientation.y = state_vector(2);
    com_msg.pose.orientation.z = state_vector(3);

    com_msg.pose.position.x = state_vector(4);
    com_msg.pose.position.y = state_vector(5);
    com_msg.pose.position.z = state_vector(6);

    com_msg.twist.angular.x = state_vector(7 + num_joints);
    com_msg.twist.angular.y = state_vector(8 + num_joints);
    com_msg.twist.angular.z = state_vector(9 + num_joints);

    com_msg.twist.linear.x = state_vector(10 + num_joints);
    com_msg.twist.linear.y = state_vector(11 + num_joints);
    com_msg.twist.linear.z = state_vector(12 + num_joints);

    com_pub.publish(com_msg);

    // const drake::multibody::Joint<double>& RL_hip_joint =
    // plant.GetJointByName("RL_hip_joint"); const drake::multibody::Joint<double>&
    // FL_hip_joint = plant.GetJointByName("FL_hip_joint"); const
    // drake::multibody::Joint<double>& RR_hip_joint =
    // plant.GetJointByName("RR_hip_joint"); const drake::multibody::Joint<double>&
    // FR_hip_joint = plant.GetJointByName("FR_hip_joint");

    // const double& RL_hip_joint_velocity = RL_hip_joint.GetOneVelocity(context);
    // const double& FL_hip_joint_velocity = FL_hip_joint.GetOneVelocity(context);
    // const double& RR_hip_joint_velocity = RR_hip_joint.GetOneVelocity(context);
    // const double& FR_hip_joint_velocity = FR_hip_joint.GetOneVelocity(context);

    // std::cout << "Joint velocity test: \n";
    // std::cout << RL_hip_joint_velocity << "\n";
    // std::cout << FL_hip_joint_velocity << "\n";
    // std::cout << RR_hip_joint_velocity << "\n";
    // std::cout << FR_hip_joint_velocity << "\n";

    // ROS_INFO_NAMED(LOGNAME, "Num groups: %i, with num elements: %i",
    // discrete_values.num_groups(), discrete_values.size()); ROS_INFO_NAMED(LOGNAME,
    // "Real time rate: %s", output_port.GetFullDescription().c_str()); ROS_INFO_NAMED(LOGNAME,
    // "Real time rate: %f", simulator.get_actual_realtime_rate());
    simulator.AdvanceTo(current_time);
    current_time += viz_time_step;
  }

  ros::waitForShutdown();
  return 0;
}
