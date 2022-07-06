#include "path.h"

using namespace HybridAStar;


/**
 * @brief CLEAR PATH
 * 
 */
void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  AddNode(node, 0);
  AddVehicle(node, 1);
  PublishPath();
  PublishPathNodes();
  PublishPathVehicles();
}

/**
 * @brief TRACE PATH
 */
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  AddSegment(node);
//  AddNode(node, i);
//  i++;
//  AddVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

/**
 * @brief TRACE PATH
 * 
 * @param nodePath 
 */
void Path::UpdatePath(const std::vector<Node3D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    AddSegment(nodePath[i]);
    AddNode(nodePath[i], k);
    k++;
    AddVehicle(nodePath[i], k);
    k++;
  }

  return;
}

/**
 * @brief add segment
 * 
 * @param node 
 */
void Path::AddSegment(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

/**
 * @brief add node
 * 
 * @param node 
 * @param i 
 */
void Path::AddNode(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (is_smoothed_) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

void Path::AddVehicle(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (is_smoothed_) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::steelblue.red;
    pathVehicle.color.g = Constants::steelblue.green;
    pathVehicle.color.b = Constants::steelblue.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getPhi());
  pathVehicles.markers.push_back(pathVehicle);
}
