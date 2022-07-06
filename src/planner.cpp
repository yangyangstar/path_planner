#include "planner.h"

using namespace HybridAStar;
/**
 * @brief Construct a new Planner:: Planner object
 *
 */
Planner::Planner() {
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;

  // publish the topics
  pubStart = n_node.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/start", 1);

  // subscribe the topics
  if (Constants::manual) {
    subMap = n_node.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n_node.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal =
      n_node.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n_node.subscribe("/initialpose", 1, &Planner::setStart, this);
};

/**
 * @brief LOOKUPTABLES
 *
 */
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

/**
 * @brief MAP
 *
 * @param map
 */
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid_ = map;
  // update the configuration space with the current map
  configurationSpace.updateGrid(map);
  // create array for Voronoi diagram
  //  ros::Time start_time = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) {
    binMap[x] = new bool[height];
  }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoi_diagram_.InitializeMap(width, height, binMap);
  voronoi_diagram_.Update();
  voronoi_diagram_.Visualize("/home/liuyang/图片/vis.pgm");
  //  ros::Time end_time = ros::Time::now();
  //  ros::Duration d(end_time - start_time);
  //  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual &&
      listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0),
                            "/map", nullptr)) {
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid_->info.height >= start.pose.pose.position.y &&
        start.pose.pose.position.y >= 0 &&
        grid_->info.width >= start.pose.pose.position.x &&
        start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else {
      validStart = false;
    }

    Plan();
  }
}

/**
 * @brief initialize start
 *
 * @param initial
 */
void Planner::setStart(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y
            << " t:" << Helper::toDeg(t) << std::endl;

  if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) {
      Plan();
    }

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y
              << " t:" << Helper::toDeg(t) << std::endl;
  }
}

/**
 * @brief initialize goal
 *
 * @param end
 */
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y
            << " t:" << Helper::toDeg(t) << std::endl;

  if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) {
      Plan();
    }

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y
              << " t:" << Helper::toDeg(t) << std::endl;
  }
}

/**
 * @brief plan the path
 *
 */
void Planner::Plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {
    // lists allowcated row major order
    int width = grid_->info.width;
    int height = grid_->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // debug goal
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);

    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // debug start
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);

    // start and time the planning
    ros::Time start_time = ros::Time::now();

    // clear the visualization
    visualization.clear();
    // clear the path
    path.clear();
    smoothedPath_.clear();
    // find the path
    Node3D* nSolution =
        Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,
                               configurationSpace, dubinsLookup, visualization);
    // trace the path
    smoother_.TracePath(nSolution);
    // create the updated path
    path.UpdatePath(smoother_.GetPath());
    // smooth the path
    smoother_.SmoothPath(voronoi_diagram_);
    // create the updated path
    smoothedPath_.UpdatePath(smoother_.GetPath());
    ros::Time end_time = ros::Time::now();
    ros::Duration d(end_time - start_time);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // publish the results of the search
    path.PublishPath();
    path.PublishPathNodes();
    path.PublishPathVehicles();
    smoothedPath_.PublishPath();
    smoothedPath_.PublishPathNodes();
    smoothedPath_.PublishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    delete[] nodes3D;
    delete[] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
