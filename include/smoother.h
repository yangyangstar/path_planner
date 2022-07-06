/**
 * @file smoother.h
 * @author Karl Kurzer
 * @brief
 * @version 0.1
 * @date 2022-04-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "constants.h"
#include "dynamicvoronoi.h"
#include "helper.h"
#include "node3d.h"
#include "vector2d.h"
namespace HybridAStar {
/**
 * @brief This class takes a path object and smoothes the nodes of the path. It
 * also uses the Voronoi diagram as well as the configuration space.
 */
class Smoother {
 public:
  Smoother() {}

  /**
   * @brief This function takes a path consisting of nodes and attempts to
     iteratively smooth the same using gradient descent.

     During the different interations the following cost are being calculated
     obstacleCost
     curvatureCost
     smoothnessCost
     voronoiCost
   *
   */
  void SmoothPath(DynamicVoronoi& voronoi);

  /**
   * @brief Given a node pointer the path to the root node will be traced
     recursively \param node a 3D node, usually the goal node \param i a
     parameter for counting the number of nodes
   */
  void TracePath(const Node3D* node);

  // returns the path of the smoother object
  const std::vector<Node3D>& GetPath() { return path_; }

  // obstacleCost - pushes the path away from obstacles
  Vector2D ObstacleTerm(Vector2D xi);

  // curvatureCost - forces a maximum curvature of 1/R along the path ensuring
  // drivability
  Vector2D CurvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);

  // smoothnessCost - attempts to spread nodes equidistantly and with the same
  // orientation
  Vector2D SmoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi,
                          Vector2D xip1, Vector2D xip2);

  // voronoiCost - trade off between path length and closeness to obstaclesg
  Vector2D VoronoiTerm(Vector2D xi);

  // a boolean test, whether vector is on the grid or not
  bool IsOnGrid(Vector2D vec) {
    if (vec.getX() >= 0 && vec.getX() < width_ && vec.getY() >= 0 &&
        vec.getY() < height_) {
      return true;
    }
    return false;
  }

 private:
  // maximum possible curvature of the non-holonomic vehicle
  float kappaMax = 1.f / (Constants::r * 1.1);
  // maximum distance to obstacles that is penalized
  float obsDMax = Constants::minRoadWidth;
  // maximum distance for obstacles to influence the voronoi field
  float vorObsDMax = Constants::minRoadWidth;
  // falloff rate for the voronoi field
  float alpha = 0.1;
  // weight for the obstacle term
  float wObstacle_ = 0.2;
  // weight for the voronoi term
  float wVoronoi_ = 0.2;
  // weight for the curvature term
  float wCurvature_ = 0;
  // weight for the smoothness term
  float wSmoothness_ = 0.2;
  // voronoi diagram describing the topology of the map
  DynamicVoronoi voronoi_;
  // width of the map
  int width_;
  // height of the map
  int height_;
  // path to be smoothed
  std::vector<Node3D> path_;
};
}  // namespace HybridAStar
#endif  // SMOOTHER_H
