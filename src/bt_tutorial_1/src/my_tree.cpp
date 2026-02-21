#include "bt_tutorial_1/custom_nodes.h"
#include <fmt/core.h>
#include <ros/package.h>



int main() {
  BT::BehaviorTreeFactory factory;

  
  std::string const package_path = ros::package::getPath("bt_tutorial_1");
  std::string msg = fmt::format("The path of the package is: {}", package_path);
  std::cout << msg << std::endl;
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerNodeType<CheckBattery>("CheckBattery");
  factory.registerNodeType<OpenGripper>("OpenGripper");

  // Trees are created at run-time from a string.
  // The user can load a tree from a file or write it directly in the code.
  // The tree is written in a specific XML format.


  auto tree = factory.createTreeFromFile(package_path+"/configs/my_tree.xml");
  BT::StdCoutLogger logger_cout(tree);
  tree.tickRootWhileRunning();

  return 0;
}