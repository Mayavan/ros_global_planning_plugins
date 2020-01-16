#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <queue>
#include <set>
#include <vector>

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace bfs_planner {

class BFSPlanner : public nav_core::BaseGlobalPlanner {
 private:
  unsigned int mx, my;
  double wx, wy;

  std::map<unsigned int, unsigned int> nodes;
  std::set<unsigned int> closedNodeIndices;
  std::queue<unsigned int> openNodeIndices;

  costmap_2d::Costmap2D* costmap;
  std::vector<unsigned int> getNeighbourNodes(unsigned int& nodeIndex);
  bool checkObstacle(unsigned int& currentNodeIndex);

 public:
  BFSPlanner();
  BFSPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
};
};  // namespace bfs_planner
#endif
