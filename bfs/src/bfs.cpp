#include "bfs/bfs.hpp"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(bfs_planner::BFSPlanner, nav_core::BaseGlobalPlanner)

// Default Constructor
namespace bfs_planner {

BFSPlanner::BFSPlanner() {}

BFSPlanner::BFSPlanner(std::string name,
                       costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void BFSPlanner::initialize(std::string name,
                            costmap_2d::Costmap2DROS* costmap_ros) {
  costmap = costmap_ros->getCostmap();

  unsigned int width = costmap->getSizeInCellsX();
  unsigned int height = costmap->getSizeInCellsY();
  unsigned int mapSize = width * height;
}

bool BFSPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
  mx = 0;
  my = 0;
  costmap->mapToWorld(mx, my, wx, wy);
  // Add corresponding index to the queue
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  unsigned int startNodeIndex = costmap->getIndex(mx, my);
  unsigned int currentNodeIndex = startNodeIndex;
  openNodeIndices.push(startNodeIndex);

  // Determine the goal node index
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  unsigned int goalNodeIndex = costmap->getIndex(mx, my);

  std::vector<unsigned int> neighbours;
  while ((currentNodeIndex != goalNodeIndex) && !openNodeIndices.empty()) {
    // Make the next node as the current node
    currentNodeIndex = openNodeIndices.front();
    openNodeIndices.pop();

    neighbours = getNeighbourNodes(currentNodeIndex);

    // Add the neighbours to openNodeIndices if not an obstacle
    for (auto i = neighbours.begin(); i != neighbours.end(); i++) {
      if (!checkObstacle(*i)) {
        openNodeIndices.push(*i);
        nodes.insert(std::make_pair(*i, currentNodeIndex));
      }
    }

    closedNodeIndices.insert(currentNodeIndex);

    // Check if next in queue has been visited already and remove if already
    // visited
    while (closedNodeIndices.find(openNodeIndices.front()) !=
           closedNodeIndices.end())
      openNodeIndices.pop();
  }

  // Find the path
  currentNodeIndex = goalNodeIndex;
  plan.push_back(goal);

  unsigned int parentIndex;
  while (currentNodeIndex != startNodeIndex) {
    parentIndex = (*nodes.find(currentNodeIndex)).second;

    // Convert node index to world coordinates
    costmap->indexToCells(parentIndex, mx, my);
    costmap->mapToWorld(mx, my, wx, wy);

    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

    new_goal.pose.position.x = wx;
    new_goal.pose.position.y = wy;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    currentNodeIndex = parentIndex;
  }
  plan.push_back(start);
  std::reverse(plan.begin(), plan.end());

  return true;
}

// Private
std::vector<unsigned int> BFSPlanner::getNeighbourNodes(
    unsigned int& nodeIndex) {
  std::vector<unsigned int> neighbours;
  neighbours.reserve(8);

  unsigned int index;
  // Left Node
  index = nodeIndex - 1;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Right Node
  index = nodeIndex + 1;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Top Node
  index = nodeIndex + my;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Bottom Node
  index = nodeIndex - my;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Top Left Node
  index = nodeIndex - 1 + my;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Top Right Node
  index = nodeIndex + 1 + my;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Top Left Node
  index = nodeIndex + my - 1;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  // Bottom Right Node
  index = nodeIndex - my + 1;
  if (!checkObstacle(index)) neighbours.emplace_back(index);

  neighbours.shrink_to_fit();

  return neighbours;
}

bool BFSPlanner::checkObstacle(unsigned int& currentNodeIndex) {
  costmap->indexToCells(currentNodeIndex, mx, my);
  if (costmap->getCost(mx, my) == costmap_2d::FREE_SPACE)
    return false;
  else
    return true;
}

};  // namespace bfs_planner