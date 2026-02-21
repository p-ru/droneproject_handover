#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h> 
#include <fmt/core.h>
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Approaching object:" << this->name()<< std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// Action that prints a message and returns SUCCESS
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string& name) :
      BT::SyncActionNode(name, {})
  {
  }


  BT::NodeStatus tick() override
  {
    std::cout << "Checking battery" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};


class GripperInterface
{
public:
  GripperInterface(): _open(true) {}
    
  BT::NodeStatus open() 
  {
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus close() 
  {
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool _open; // shared information
};

// Action that prints a message and returns SUCCESS
class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string& name) :
      BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Opening gripper" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};







class SaySomething : public BT::SyncActionNode {
  public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }
  
  BT::NodeStatus tick() override {
    BT::Optional<std::string> msg = TreeNode::getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
  std::string myString = fmt::format("Robot says: {}", msg.value());
  std::cout << myString << std::endl;
  return BT::NodeStatus::SUCCESS;
  }
};

class ThinkWhatToSay : public BT::SyncActionNode {
public:
  ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("text") };
  }

  // This Action writes a value into the port "text"
  BT::NodeStatus tick() override
  {
    // the output may change at each tick(). Here we keep it simple.
    TreeNode::setOutput("text", "The answer is 42" );
    return BT::NodeStatus::SUCCESS;
  }
};



struct Waypoint {
    double x;
    double y;
    double z;
};

class AllGoalsAchieved : public BT::ConditionNode {
public:
  AllGoalsAchieved(const std::string& name,
                   const BT::NodeConfiguration& cfg)
      : ConditionNode(name, cfg) {}

  BT::NodeStatus tick() override
  {
    auto wps = getInput<std::vector<Waypoint>>("waypoints");
    return (wps && wps->empty()) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::vector<Waypoint>>("waypoints") };
  }
};

/* -------------- SyncAction: GetNextWaypoint ------------------- */
class GetNextWaypoint : public BT::SyncActionNode {
public:
  GetNextWaypoint(const std::string& name,
                  const BT::NodeConfiguration& cfg)
      : SyncActionNode(name, cfg) {}

  BT::NodeStatus tick() override
  {
    auto wps = getInput<std::vector<Waypoint>>("waypoints");
    if(!wps || wps->empty()) return BT::NodeStatus::FAILURE;

    Waypoint next = wps->front();
    wps->erase(wps->begin());
    setOutput("waypoints", *wps);
    setOutput("wp", next);
    return BT::NodeStatus::SUCCESS;
  }
  static BT::PortsList providedPorts() {
    return {
      BT::BidirectionalPort<std::vector<Waypoint>>("waypoints"),
      BT::OutputPort<Waypoint>("wp")
    };
  }
};

// /* -------------- AsyncAction: FlyToPose ------------------------ */
// class FlyToPose : public BT::StatefulActionNode {
// public:
//   FlyToPose(const std::string& name,
//             const BT::NodeConfiguration& cfg)
//       : StatefulActionNode(name, cfg), ac_("fly_to_pose", true) {}

//   BT::NodeStatus onStart() override {
//     Waypoint goal; getInput("goal", goal);
//     // build and send action goal here…
//     return BT::NodeStatus::RUNNING;
//   }
//   BT::NodeStatus onRunning() override {
//     if(ac_.getState() == SUCCEEDED) return BT::NodeStatus::SUCCESS;
//     if(ac_.getState() == ABORTED)   return BT::NodeStatus::FAILURE;
//     return NodeStatus::RUNNING;
//   }
//   void onHalted() override { ac_.cancelGoal(); }

//   static PortsList providedPorts() {
//     return { BT::InputPort<Waypoint>("goal") };
//   }
// private:
//   actionlib::SimpleActionClient<fc_msgs::FlyTo> ac_;
// };

// /* -------------- Condition: CheckArrival ----------------------- */
// class CheckArrival : public BT::ConditionNode {
// public:
//   BT::NodeStatus tick() override {
//     Waypoint goal; getInput("goal", goal);
//     auto now = current_pose();                     // helper
//     double d = distance(now, goal);
//     return (d < 0.5) ? SUCCESS : FAILURE;
//   }
//   static BT::PortsList providedPorts() {
//     return { BT::InputPort<Waypoint>("goal") };
//   }
// };
