/**
 * @file dynamicvoronoi.h
 * @author karl kurzer (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-04-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include <queue>

#include "bucketedqueue.h"

namespace HybridAStar {
//! A DynamicVoronoi object computes and updates a distance map and Voronoi
//! diagram.
class DynamicVoronoi {
 public:
  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map
  void InitializeEmpty(int sizeX, int sizeY, bool initGridMap = true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void InitializeMap(int sizeX, int sizeY, bool **gridMap);

  //! add an obstacle at the specified cell coordinate
  void OccupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void ClearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void ExchangeObstacles(const std::vector<IntPoint> &newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void Update(bool updateRealDist = true);
  //! prune the Voronoi diagram
  void Prune();

  //! returns the obstacle distance at the specified location
  float GetDistance(int x, int y) const;
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool IsVoronoi(int x, int y) const;
  //! checks whether the specified location is occupied
  bool IsOccupied(int x, int y) const;
  //! write the current distance map and voronoi diagram as ppm file
  void Visualize(const char *filename = "result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int GetSizeX() const { return sizeX_; }
  //! returns the vertical size of the workspace/map
  unsigned int GetSizeY() const { return sizeY_; }

  // was private, changed to public for obstX, obstY
 public:
  struct DataCell {
    // stores the Euclidean distance from each cell to the closest occupied cell
    // in the corresponding grid map
    float dist;
    char voronoi;
    char queueing;
    // stores for all cells the coordinates of their closest occupied cell
    int obstX;
    int obstY;
    // use to ensure proper processing of cells in the wavefronts, in particular
    // where raise and lower wavefronts overlap
    bool needsRaise;
    int sqdist;
  };

  typedef enum {
    voronoiKeep = -4,
    freeQueued = -3,
    voronoiRetry = -2,
    voronoiPrune = -1,
    free = 0,
    occupied = 1
  } State;
  typedef enum {
    fwNotQueued = 1,
    fwQueued = 2,
    fwProcessed = 3,
    bwQueued = 4,
    bwProcessed = 1
  } QueueingState;
  typedef enum { invalidObstData = SHRT_MAX / 2 } ObstDataState;
  typedef enum { pruned, keep, retry } markerMatchResult;

  // methods
  void SetObstacle(int x, int y);
  void RemoveObstacle(int x, int y);
  inline void CheckVoronoi(int x, int y, int nx, int ny, DataCell &c,
                           DataCell &nc);
  void RecheckVoro();
  void CommitAndColorize(bool updateRealDist = true);
  inline void ReviveVoroNeighbors(int &x, int &y);

  inline bool IsOccupied(int &x, int &y, DataCell &c);
  inline markerMatchResult MarkerMatch(int x, int y);

  // queues
  BucketPrioQueue open_pq_;
  std::queue<INTPOINT> pruneQueue_;

  std::vector<INTPOINT> removeList_;
  std::vector<INTPOINT> addList_;
  std::vector<INTPOINT> lastObstacles_;

  // maps
  int sizeX_;
  int sizeY_;
  DataCell **data_;
  bool **gridMap_;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  //  DataCell** getData(){ return data; }
};
}  // namespace HybridAStar

#endif
