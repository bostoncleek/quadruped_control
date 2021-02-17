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
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"

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

  // // Use 1 thread
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // Robot
  const auto urdf_path = pnh.param<std::string>("urdf_path", "../urdf/robot.urdf");
  ROS_INFO_NAMED(LOGNAME, "File path: %s", urdf_path.c_str());

  // Physics 
  const auto time_step = pnh.param<double>("time_step", 0.001);
  const auto static_friction = pnh.param<double>("static_friction", 1.0);
  const auto dynamic_friction = pnh.param<double>("dynamic_friction", 1.0);
  const auto penetration_allowance = pnh.param<double>("penetration_allowance", 0.001);
  const auto stiction_tolerance = pnh.param<double>("stiction_tolerance", 0.001);
  const auto real_time_rate = pnh.param<double>("real_time_rate", 1.0);

  // Robot initial pose
  // See MultibodyPlant SetPositions() to set init joint positions
  std::vector<double> init_pose = {0.0, 0.0, 0.0};
  std::vector<double> init_orientation = {0.0, 0.0, 0.0, 1.0};
  pnh.getParam("initial_pose/postion", init_pose);
  pnh.getParam("initial_pose/orientation", init_orientation);

  // Robot kinematics 
  const auto base_link_name = pnh.param<std::string>("links/base_link", "base_link");

  // Robot initial joints 

  // TODO: figure out why mit cheetah robot wont spawn above plane
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
  ROS_INFO_NAMED(LOGNAME, "plant finalized: %s", plant.is_finalized() ? "True" : "False");

  plant.set_penetration_allowance(penetration_allowance);

  // const int num_positions = plant.num_positions();
  // const int num_velocities = plant.num_velocities();
  // ROS_INFO_NAMED(LOGNAME, "num positions: %i", num_positions);
  // ROS_INFO_NAMED(LOGNAME, "num velocities: %i", num_velocities);

  // const VectorXd kp = VectorXd::Constant(num_positions, 100);
  // const VectorXd kd = VectorXd::Constant(num_positions, 10);
  // const VectorXd ki = VectorXd::Zero(num_positions);

  // auto controller = builder.AddSystem<drake::systems::controllers::PidController>(kp, ki, kd);


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

  const drake::systems::InputPort<double>& input_port = plant.get_actuation_input_port();
  const drake::systems::OutputPort<double>& output_port = plant.get_state_output_port();
  ROS_INFO_NAMED(LOGNAME, "Input port name: %s", input_port.get_name().c_str());
  ROS_INFO_NAMED(LOGNAME, "Output port name: %s", output_port.get_name().c_str());

  // Set torque input to zeros
  VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
  input_port.FixValue(&plant_context, tau);

  const drake::multibody::Body<double>& base_link = plant.GetBodyByName(base_link_name);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, base_link, Twb);

  // Advancing the state of hybrid dynamic systems forward in time.
  drake::systems::Simulator simulator(*diagram, std::move(diagram_context));
  simulator.Initialize();
  simulator.set_target_realtime_rate(real_time_rate);
  // simulator.set_publish_every_time_step(true);
  // simulator.AdvanceTo(10.0);

  const auto dt = 1e-3;
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

    // // const drake::systems::Context<double>& context = simulator.get_context();
    // // const drake::systems::DiscreteValues<double>& discrete_values =
    // // context.get_discrete_state(); const drake::VectorX<double>& state_vector =
    // // discrete_values.get_vector().CopyToVector();

    const drake::VectorX<double>& state_vector =
        plant_context.get_discrete_state_vector().CopyToVector();

    // // const drake::multibody::Joint<double>& left_joint =
    // // plant.GetJointByName("left_wheel_axle"); const drake::multibody::Joint<double>&
    // // right_joint = plant.GetJointByName("right_wheel_axle"); const double& left_position
    // // = left_joint.GetOnePosition(context); const double& right_position =
    // // right_joint.GetOnePosition(context); const double& left_velocity =
    // // left_joint.GetOneVelocity(context); const double& right_velocity =
    // // right_joint.GetOneVelocity(context);

    // // std::cout << "Joint position test: \n" << left_position << "\n" << right_position
    // // << std::endl; std::cout << "Joint velocity test: \n" << left_velocity << "\n" <<
    // // right_velocity << std::endl;


    // std::cout << "------------------" << std::endl;
    // std::cout << "Orientation (qx, qy, qz, qw): \n"
    //           << state_vector.segment(0, 4) << std::endl;
    // std::cout << "---" << std::endl;

    // std::cout << "Position: \n" << state_vector.segment(4, 3) << std::endl;
    // std::cout << "---" << std::endl;

    // std::cout << "Joint position: \n" << state_vector.segment(7, 2) << std::endl;
    // std::cout << "---" << std::endl;

    // std::cout << "Angular velocity vector: \n" << state_vector.segment(9, 3) << std::endl;
    // std::cout << "---" << std::endl;

    // std::cout << "Velocity vector: \n" << state_vector.segment(12, 3) << std::endl;
    // std::cout << "---" << std::endl;

    // std::cout << "Joint velocity: \n" << state_vector.segment(15, 2) << std::endl;
    // std::cout << "------------------\n" << std::endl;

    // const drake::systems::BasicVector<double>& state = context.get_discrete_state_vector();
    // const drake::VectorX<double>& state_vector = state.CopyToVector();
    // std::cout << "State: \n" << state_vector << std::endl;
    // std::cout << "size: " << state_vector.size() << std::endl;

    // ROS_INFO_NAMED(LOGNAME, "Num groups: %i, with num elements: %i",
    // discrete_values.num_groups(), discrete_values.size()); ROS_INFO_NAMED(LOGNAME,
    // "Real time rate: %s", output_port.GetFullDescription().c_str()); ROS_INFO_NAMED(LOGNAME,
    // "Real time rate: %f", simulator.get_actual_realtime_rate());
    simulator.AdvanceTo(current_time);
    current_time += dt;
  }

  ros::shutdown();
  return 0;
}