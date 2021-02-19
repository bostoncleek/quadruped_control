/**
 * @file drake_interface_node.cpp
 * @author Boston Cleek
 * @date 2021-02-16
 * @brief Interface Drake Physics with ROS
 *
 * @PARAMETERS:
 *
 * @PUBLISHES:
 *
 * @SUBSCRIBES:
 *
 */

// C++
#include <memory>
#include <filesystem>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <quadruped_msgs/CoMState.h>

// Drkae
#include <drake/systems/framework/framework_common.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/contact_results_to_lcm.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/manipulation/util/robot_plan_utils.h>


using drake::math::RigidTransformd;
using drake::systems::VectorBase;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

const std::string LOGNAME = "drake_interface";


int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting drake interface node");
  ros::init(argc, argv, "drake_interface");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher com_pub = nh.advertise<quadruped_msgs::CoMState>("com_state", 1);

  // // Use 1 thread
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // Robot
  const auto urdf_path = pnh.param<std::string>("urdf_path", "../urdf/robot.urdf");
  ROS_INFO_NAMED(LOGNAME, "File path: %s", urdf_path.c_str());

  // Physics
  const auto time_step = pnh.param<double>("time_step", 0.001);
  const auto viz_time_step = pnh.param<double>("viz_time_step", 0.001);
  const auto static_friction = pnh.param<double>("static_friction", 1.0);
  const auto dynamic_friction = pnh.param<double>("dynamic_friction", 1.0);
  const auto penetration_allowance = pnh.param<double>("penetration_allowance", 0.001);
  const auto stiction_tolerance = pnh.param<double>("stiction_tolerance", 0.001);
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
  std::vector<std::string> joint_names;
  std::vector<double> init_joint_positions;
  pnh.getParam("joints/joint_names", joint_names);
  pnh.getParam("joints/joint_positions", init_joint_positions);

  const unsigned int num_joints = joint_names.size();
  if (joint_names.size() != init_joint_positions.size())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Number of initial joint positions does not match the number of joints");
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

  const drake::systems::InputPort<double>& input_port = plant.get_actuation_input_port();
  // const drake::systems::OutputPort<double>& output_port = plant.get_state_output_port();
  // ROS_INFO_NAMED(LOGNAME, "Input port name: %s", input_port.get_name().c_str());
  // ROS_INFO_NAMED(LOGNAME, "Output port name: %s", output_port.get_name().c_str());

  // Set initial positons   
  // TODO: set init COM pose here too?
  Eigen::VectorBlock<drake::VectorX<double>> state_vec = plant_context.get_mutable_discrete_state(0).get_mutable_value();
  const VectorXd init_joint_vec = Eigen::Map<VectorXd, Eigen::Unaligned>(init_joint_positions.data(), init_joint_positions.size());
  state_vec.segment(7, init_joint_positions.size()) = init_joint_vec;

  // Set torque input to zeros
  VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
  // VectorXd tau = VectorXd::Constant(plant.num_actuated_dofs(), 0.1);

  input_port.FixValue(&plant_context, tau);

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
    // if (current_time > 1.0)
    // {
    //   // ROS_INFO_NAMED(LOGNAME, "Drive");

    //   tau(0) = 0.06;
    //   tau(1) = 0.06;
    //   input_port.FixValue(&plant_context, tau);
    // }

    // TODO: which context to use?
    const drake::systems::Context<double>& context = simulator.get_context();

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
    VectorXd::Map(&joint_positions.at(0), num_joints) = state_vector.segment(7, num_joints);

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


    // const VectorXd actual_pos = plant.GetPositions(plant_context);
    // std::cout << "actual pos: \n" << actual_pos << std::endl;
    // std::cout << "state_vector: \n" << state_vector << std::endl;

    // const drake::multibody::Joint<double>& RL_hip_joint = plant.GetJointByName("RL_hip_joint");
    // const drake::multibody::Joint<double>& FL_hip_joint = plant.GetJointByName("FL_hip_joint");
    // const drake::multibody::Joint<double>& RR_hip_joint = plant.GetJointByName("RR_hip_joint");
    // const drake::multibody::Joint<double>& FR_hip_joint = plant.GetJointByName("FR_hip_joint");
    
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

  ros::shutdown();
  return 0;
}