#include "bt_tutorial_1/custom_nodes.h"
#include <ros/package.h>


int main() { 
    BT::BehaviorTreeFactory factory;


    std::string const package_path = ros::package::getPath("bt_tutorial_1");
    std::string msg = fmt::format("The path of the package is: {}", package_path);
    std::cout << msg << std::endl;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    //std::string const package_path = ros::package::getPath("bt_tutorial_1");


    auto tree = factory.createTreeFromFile(package_path+"/configs/blackboard_tree.xml");
    BT::StdCoutLogger logger_cout(tree);
    tree.tickRootWhileRunning();
    return 0;
}
