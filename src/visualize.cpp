#include "visualize.h"
using namespace HybridAStar;
/**
 * @brief clear visualization
 *
 */
void Visualize::clear() {
  poses3D.poses.clear();
  poses3Dreverse.poses.clear();
  poses2D.poses.clear();

  // 3D costs
  visualization_msgs::MarkerArray costCubes3D;
  visualization_msgs::Marker costCube3D;
  // clear the cost heatmap
  costCube3D.header.frame_id = "path";
  costCube3D.header.stamp = ros::Time::now();
  costCube3D.id = 0;
  costCube3D.action = 3;
  costCubes3D.markers.push_back(costCube3D);
  pubNodes3DCosts.publish(costCubes3D);

  // 2D costs
  visualization_msgs::MarkerArray costCubes2D;
  visualization_msgs::Marker costCube2D;
  // clear the cost heatmap
  costCube2D.header.frame_id = "path";
  costCube2D.header.stamp = ros::Time::now();
  costCube2D.id = 0;
  costCube2D.action = 3;
  costCubes2D.markers.push_back(costCube2D);
  pubNodes2DCosts.publish(costCubes2D);
}

/**
 * @brief current 3D node
 * @param node
 */
void Visualize::publishNode3DPose(Node3D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX() * Constants::cellSize;
  pose.pose.position.y = node.getY() * Constants::cellSize;

  // Forward
  if (node.getPrim() < 3) {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getPhi());
  } else {  // Reverse
    pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(node.getPhi() + M_PI);
  }

  // publish the pose
  pubNode3D.publish(pose);
}

/**
 * @brief all expanded 3D nodes
 * @param node
 */
void Visualize::publishNode3DPoses(Node3D& node) {
  geometry_msgs::Pose pose;
  pose.position.x = node.getX() * Constants::cellSize;
  pose.position.y = node.getY() * Constants::cellSize;

  // Forward
  if (node.getPrim() < 3) {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getPhi());
    poses3D.poses.push_back(pose);
    poses3D.header.stamp = ros::Time::now();
    // publish the posearray
    pubNodes3D.publish(poses3D);
  }
  // Reverse
  else {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getPhi() + M_PI);
    poses3Dreverse.poses.push_back(pose);
    poses3Dreverse.header.stamp = ros::Time::now();
    // publish the posearray
    pubNodes3Dreverse.publish(poses3Dreverse);
  }
}

/**
 * @brief current 2D node
 * @param node
 */
void Visualize::publishNode2DPose(Node2D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
  pose.pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // publish the pose
  pubNode2D.publish(pose);
}

/**
 * @brief all expanded 2D nodes
 *
 * @param node
 */
void Visualize::publishNode2DPoses(Node2D& node) {
  if (node.isDiscovered()) {
    geometry_msgs::Pose pose;
    pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
    pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);

    poses2D.poses.push_back(pose);
    poses2D.header.stamp = ros::Time::now();
    // publish the posearray
    pubNodes2D.publish(poses2D);
  }
}

/**
 * @brief cost heatmap 3D
 * @param nodes
 * @param width
 * @param height
 * @param depth
 */
void Visualize::publishNode3DCosts(Node3D* nodes, int width, int height,
                                   int depth) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  int idx;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // determine the max and min values
  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    // iterate over all headings
    for (int k = 0; k < depth; ++k) {
      idx = k * width * height + i;

      // set the minimum for the cell
      if (nodes[idx].isClosed() || nodes[idx].isOpen()) {
        values[i] = nodes[idx].getC();
      }
    }

    // set a new minimum
    if (values[i] > 0 && values[i] < min) {
      min = values[i];
    }

    // set a new maximum
    if (values[i] > 0 && values[i] > max && values[i] != 1000) {
      max = values[i];
    }
  }

  // paint the cubes
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (values[i] != 1000) {
      count++;

      // delete all previous markers
      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }

      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y =
          ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "3D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 3D nodes expanded " << std::endl;
  }

  // publish the costcubes
  pubNodes3DCosts.publish(costCubes);
}

/**
 * @brief cost heatmap 2D
 * @param nodes a pointer to Node2D
 * @param width
 * @param height
 */
void Visualize::publishNode2DCosts(Node2D* nodes, int width, int height) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // determine the max and min values
  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    // set the minimum for the cell
    if (nodes[i].isDiscovered()) {
      values[i] = nodes[i].getG();

      // set a new minimum
      if (values[i] > 0 && values[i] < min) {
        min = values[i];
      }

      // set a new maximum
      if (values[i] > 0 && values[i] > max) {
        max = values[i];
      }
    }
  }

  // paint the cubes
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (nodes[i].isDiscovered()) {
      count++;

      // delete all previous markers
      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }

      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y =
          ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "2D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 2D nodes expanded " << std::endl;
  }

  // publish the costcubes
  pubNodes2DCosts.publish(costCubes);
}
