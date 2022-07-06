/**
 * @file dynamicvoronoi.cpp
 * @author Karl Kurzer
 * @brief
 * @version 0.1
 * @date 2022-04-15
 *
 * @copyright Copyright (data) 2022
 *
 */

#include "dynamicvoronoi.h"

#include <math.h>

#include <iostream>
#include <limits>

using namespace HybridAStar;

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = std::sqrt(2.0);
  data_ = NULL;
  gridMap_ = NULL;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data_) {
    for (int x = 0; x < sizeX_; x++) delete[] data_[x];
    delete[] data_;
  }
  if (gridMap_) {
    for (int x = 0; x < sizeX_; x++) delete[] gridMap_[x];
    delete[] gridMap_;
  }
}

void DynamicVoronoi::InitializeEmpty(int sizeX, int sizeY, bool init_gridmap) {
  sizeX_ = sizeX;
  sizeY_ = sizeY;
  if (data_) {
    for (int x = 0; x < sizeX_; x++) {
      delete[] data_[x];
    }
    delete[] data_;
  }
  data_ = new DataCell *[sizeX_];
  for (int x = 0; x < sizeX_; x++) data_[x] = new DataCell[sizeY_];

  if (init_gridmap) {
    if (gridMap_) {
      for (int x = 0; x < sizeX_; x++) {
        delete[] gridMap_[x];
      }
      delete[] gridMap_;
    }

    gridMap_ = new bool *[sizeX_];
    for (int x = 0; x < sizeX_; x++) {
      gridMap_[x] = new bool[sizeY_];
    }
  }

  DataCell data;
  data.dist = std::numeric_limits<float>::infinity();
  data.sqdist = std::numeric_limits<int>::max();
  data.obstX = invalidObstData;
  data.obstY = invalidObstData;
  data.voronoi = free;
  data.queueing = fwNotQueued;
  data.needsRaise = false;

  for (int x = 0; x < sizeX_; x++) {
    for (int y = 0; y < sizeY_; y++) {
      data_[x][y] = data;
    }
  }

  if (init_gridmap) {
    for (int x = 0; x < sizeX_; x++) {
      for (int y = 0; y < sizeY_; y++) {
        gridMap_[x][y] = false;
      }
    }
  }
}

void DynamicVoronoi::InitializeMap(int sizeX, int sizeY, bool **gridMap) {
  gridMap_ = gridMap;
  InitializeEmpty(sizeX, sizeY, false);

  for (int x = 0; x < sizeX; x++) {
    for (int y = 0; y < sizeY; y++) {
      if (gridMap_[x][y]) {
        DataCell data = data_[x][y];
        if (!IsOccupied(x, y, data)) {
          bool isSurrounded = true;

          for (int dx = -1; dx <= 1; dx++) {
            int nx = x + dx;
            if (nx <= 0 || nx >= sizeX - 1) {
              continue;
            }
            for (int dy = -1; dy <= 1; dy++) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              int ny = y + dy;
              if (ny <= 0 || ny >= sizeY - 1) {
                continue;
              }

              if (!gridMap_[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }

          if (isSurrounded) {
            data.obstX = x;
            data.obstY = y;
            data.sqdist = 0;
            data.dist = 0;
            data.voronoi = occupied;
            data.queueing = fwProcessed;
            data_[x][y] = data;
          } else {
            SetObstacle(x, y);
          }
        }
      }
    }
  }
}

void DynamicVoronoi::OccupyCell(int x, int y) {
  gridMap_[x][y] = true;
  SetObstacle(x, y);
}

void DynamicVoronoi::ClearCell(int x, int y) {
  gridMap_[x][y] = false;
  RemoveObstacle(x, y);
}

void DynamicVoronoi::SetObstacle(int x, int y) {
  DataCell data = data_[x][y];
  if (IsOccupied(x, y, data)) {
    return;
  }

  addList_.push_back(INTPOINT(x, y));
  data.obstX = x;
  data.obstY = y;
  data_[x][y] = data;
}

void DynamicVoronoi::RemoveObstacle(int x, int y) {
  DataCell data = data_[x][y];
  if (!IsOccupied(x, y, data)) {
    return;
  }

  removeList_.push_back(INTPOINT(x, y));
  data.obstX = invalidObstData;
  data.obstY = invalidObstData;
  data.queueing = bwQueued;
  data_[x][y] = data;
}

void DynamicVoronoi::ExchangeObstacles(const std::vector<INTPOINT> &points) {
  for (unsigned int i = 0; i < lastObstacles_.size(); i++) {
    int x = lastObstacles_[i].x;
    int y = lastObstacles_[i].y;

    bool has_obstacle = gridMap_[x][y];
    if (has_obstacle) {
      continue;
    }
    RemoveObstacle(x, y);
  }

  lastObstacles_.clear();
  lastObstacles_.reserve(points.size());

  for (unsigned int i = 0; i < points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool has_obstacle = gridMap_[x][y];
    if (has_obstacle) {
      continue;
    }
    SetObstacle(x, y);
    lastObstacles_.push_back(points[i]);
  }
}

void DynamicVoronoi::Update(bool updateRealDist) {
  CommitAndColorize(updateRealDist);

  while (!open_pq_.empty()) {
    INTPOINT p = open_pq_.pop();
    int x = p.x;
    int y = p.y;
    DataCell data = data_[x][y];

    if (data.queueing == fwProcessed) {
      continue;
    }

    if (data.needsRaise) {
      // RAISE
      // get neighbor node
      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX_ - 1) continue;
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY_ - 1) continue;
          DataCell nc = data_[nx][ny];
          if (nc.obstX != invalidObstData && !nc.needsRaise) {
            if (!IsOccupied(nc.obstX, nc.obstY, data_[nc.obstX][nc.obstY])) {
              open_pq_.push(nc.sqdist, INTPOINT(nx, ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) {
                nc.dist = INFINITY;
              }
              nc.sqdist = INT_MAX;
              data_[nx][ny] = nc;
            } else {
              if (nc.queueing != fwQueued) {
                open_pq_.push(nc.sqdist, INTPOINT(nx, ny));
                nc.queueing = fwQueued;
                data_[nx][ny] = nc;
              }
            }
          }
        }
      }
      data.needsRaise = false;
      data.queueing = bwProcessed;
      data_[x][y] = data;
    } else if (data.obstX != invalidObstData &&
               IsOccupied(data.obstX, data.obstY,
                          data_[data.obstX][data.obstY])) {
      // LOWER
      data.queueing = fwProcessed;
      data.voronoi = occupied;

      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX_ - 1) continue;
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY_ - 1) continue;
          DataCell nc = data_[nx][ny];
          if (!nc.needsRaise) {
            int distx = nx - data.obstX;
            int disty = ny - data.obstY;
            int newSqDistance = distx * distx + disty * disty;
            /* 满足以下两个条件之一，需要进行lower更新：
             * <1>
             * 对于某个neighbor_cell(nc)，如果上次它到最近障碍物的距离比本次它到cell(x,y)的最近障碍物的距离还要大;
             * <2>
             * 对于某个neighbor_cell(nc)，如果上次它到最近障碍物的距离与本次它到cell(x,y)的最近障碍物的距离相等，
             * 但是上次它的最近障碍物在本次消失
             */
            bool overwrite = (newSqDistance < nc.sqdist);
            if (!overwrite && newSqDistance == nc.sqdist) {
              if (nc.obstX == invalidObstData ||
                  !IsOccupied(nc.obstX, nc.obstY, data_[nc.obstX][nc.obstY]))
                overwrite = true;
            }

            if (overwrite) {
              open_pq_.push(newSqDistance, INTPOINT(nx, ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double)newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = data.obstX;
              nc.obstY = data.obstY;
            } else {
              CheckVoronoi(x, y, nx, ny, data, nc);
            }
            data_[nx][ny] = nc;
          }
        }
      }
    }
    data_[x][y] = data;
  }
}

float DynamicVoronoi::GetDistance(int x, int y) const {
  if ((x > 0) && (x < sizeX_) && (y > 0) && (y < sizeY_)) {
    return data_[x][y].dist;
  } else {
    return -INFINITY;
  }
}

bool DynamicVoronoi::IsVoronoi(int x, int y) const {
  DataCell data = data_[x][y];
  return (data.voronoi == free || data.voronoi == voronoiKeep);
}

void DynamicVoronoi::CommitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i = 0; i < addList_.size(); i++) {
    INTPOINT p = addList_[i];
    int x = p.x;
    int y = p.y;
    DataCell data = data_[x][y];

    if (data.queueing != fwQueued) {
      if (updateRealDist) {
        data.dist = 0;
      }
      data.sqdist = 0;
      data.obstX = x;
      data.obstY = y;
      data.queueing = fwQueued;
      data.voronoi = occupied;
      data_[x][y] = data;
      open_pq_.push(0, INTPOINT(x, y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i = 0; i < removeList_.size(); i++) {
    INTPOINT p = removeList_[i];
    int x = p.x;
    int y = p.y;
    DataCell data = data_[x][y];

    if (IsOccupied(x, y, data)) {
      // obstacle was removed and reinserted
      continue;
    }
    open_pq_.push(0, INTPOINT(x, y));
    if (updateRealDist) {
      data.dist = INFINITY;
    }
    data.sqdist = INT_MAX;
    data.needsRaise = true;
    data_[x][y] = data;
  }
  removeList_.clear();
  addList_.clear();
}

// Condition-based GVD
void DynamicVoronoi::CheckVoronoi(int x, int y, int nx, int ny, DataCell &data,
                                  DataCell &nc) {
  if ((data.sqdist > 1 || nc.sqdist > 1) && nc.obstX != invalidObstData) {
    if (abs(data.obstX - nc.obstX) > 1 || abs(data.obstY - nc.obstY) > 1) {
      // compute dist from x,y to obstacle of nx,ny
      int dxy_x = x - nc.obstX;
      int dxy_y = y - nc.obstY;
      int sqdxy = dxy_x * dxy_x + dxy_y * dxy_y;
      int stability_xy = sqdxy - data.sqdist;
      if (sqdxy < data.sqdist) {
        return;
      }

      // compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - data.obstX;
      int dnxy_y = ny - data.obstY;
      int sqdnxy = dnxy_x * dnxy_x + dnxy_y * dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy < nc.sqdist) {
        return;
      }

      // which cell is added to the Voronoi diagram?
      if (stability_xy <= stability_nxy && data.sqdist > 2) {
        if (data.voronoi != free) {
          data.voronoi = free;
          ReviveVoroNeighbors(x, y);
          pruneQueue_.push(INTPOINT(x, y));
        }
      }
      if (stability_nxy <= stability_xy && nc.sqdist > 2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          ReviveVoroNeighbors(nx, ny);
          pruneQueue_.push(INTPOINT(nx, ny));
        }
      }
    }
  }
}

void DynamicVoronoi::ReviveVoroNeighbors(int &x, int &y) {
  for (int dx = -1; dx <= 1; dx++) {
    int nx = x + dx;
    if (nx <= 0 || nx >= sizeX_ - 1) {
      continue;
    }
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      int ny = y + dy;
      if (ny <= 0 || ny >= sizeY_ - 1) {
        continue;
      }
      DataCell nc = data_[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise &&
          (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data_[nx][ny] = nc;
        pruneQueue_.push(INTPOINT(nx, ny));
      }
    }
  }
}

bool DynamicVoronoi::IsOccupied(int x, int y) const {
  DataCell data = data_[x][y];
  return (data.obstX == x && data.obstY == y);
}

bool DynamicVoronoi::IsOccupied(int &x, int &y, DataCell &data) {
  return (data.obstX == x && data.obstY == y);
}

void DynamicVoronoi::Visualize(const char *filename) {
  // write pgm files

  FILE *F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX_, sizeY_);

  for (int y = sizeY_ - 1; y >= 0; y--) {
    for (int x = 0; x < sizeX_; x++) {
      unsigned char data = 0;
      if (IsVoronoi(x, y)) {
        fputc(255, F);
        fputc(0, F);
        fputc(0, F);
      } else if (data_[x][y].sqdist == 0) {
        fputc(0, F);
        fputc(0, F);
        fputc(0, F);
      } else {
        float f = 80 + (data_[x][y].dist * 5);
        if (f > 255) f = 255;
        if (f < 0) f = 0;
        data = (unsigned char)f;
        fputc(data, F);
        fputc(data, F);
        fputc(data, F);
      }
    }
  }
  fclose(F);
}

void DynamicVoronoi::Prune() {
  // filler
  while (!pruneQueue_.empty()) {
    INTPOINT p = pruneQueue_.front();
    pruneQueue_.pop();
    int x = p.x;
    int y = p.y;

    if (data_[x][y].voronoi == occupied) {
      continue;
    }
    if (data_[x][y].voronoi == freeQueued) {
      continue;
    }

    data_[x][y].voronoi = freeQueued;
    open_pq_.push(data_[x][y].sqdist, p);

    /* an 8-connected grid model
      top_left     top    top_right
              \     |      /
               \    |     /
        left----- data ----right
               /    |    \
              /     |     \
    below_left    below  belowright
    */

    DataCell tr, tl, br, bl;
    tr = data_[x + 1][y + 1];
    tl = data_[x - 1][y + 1];
    br = data_[x + 1][y - 1];
    bl = data_[x - 1][y - 1];

    DataCell r, b, t, l;
    r = data_[x + 1][y];
    l = data_[x - 1][y];
    t = data_[x][y + 1];
    b = data_[x][y - 1];

    if (x + 2 < sizeX_ && r.voronoi == occupied) {
      // fill to the right
      if (tr.voronoi != occupied && br.voronoi != occupied &&
          data_[x + 2][y].voronoi != occupied) {
        r.voronoi = freeQueued;
        open_pq_.push(r.sqdist, INTPOINT(x + 1, y));
        data_[x + 1][y] = r;
      }
    }
    if (x - 2 >= 0 && l.voronoi == occupied) {
      // fill to the left
      if (tl.voronoi != occupied && bl.voronoi != occupied &&
          data_[x - 2][y].voronoi != occupied) {
        l.voronoi = freeQueued;
        open_pq_.push(l.sqdist, INTPOINT(x - 1, y));
        data_[x - 1][y] = l;
      }
    }
    if (y + 2 < sizeY_ && t.voronoi == occupied) {
      // fill to the top
      if (tr.voronoi != occupied && tl.voronoi != occupied &&
          data_[x][y + 2].voronoi != occupied) {
        t.voronoi = freeQueued;
        open_pq_.push(t.sqdist, INTPOINT(x, y + 1));
        data_[x][y + 1] = t;
      }
    }
    if (y - 2 >= 0 && b.voronoi == occupied) {
      // fill to the bottom
      if (br.voronoi != occupied && bl.voronoi != occupied &&
          data_[x][y - 2].voronoi != occupied) {
        b.voronoi = freeQueued;
        open_pq_.push(b.sqdist, INTPOINT(x, y - 1));
        data_[x][y - 1] = b;
      }
    }
  }

  while (!open_pq_.empty()) {
    INTPOINT p = open_pq_.pop();
    DataCell data = data_[p.x][p.y];
    int voronoi_type = data.voronoi;
    if (voronoi_type != freeQueued &&
        voronoi_type != voronoiRetry) {  // || voronoi_type>free ||
                                         // voronoi_type==voronoiPrune
                                         // || voronoi_type==voronoiKeep) {
      //      assert(voronoi_type!=retry);
      continue;
    }

    markerMatchResult r = MarkerMatch(p.x, p.y);
    if (r == pruned)
      data.voronoi = voronoiPrune;
    else if (r == keep)
      data.voronoi = voronoiKeep;
    else {  // r==retry
      data.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue_.push(p);
    }
    data_[p.x][p.y] = data;

    if (open_pq_.empty()) {
      while (!pruneQueue_.empty()) {
        INTPOINT p = pruneQueue_.front();
        pruneQueue_.pop();
        open_pq_.push(data_[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

DynamicVoronoi::markerMatchResult DynamicVoronoi::MarkerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i = 0;
  int count = 0;
  //  int obstacleCount=0;
  int voroCount = 0;
  int voroCountFour = 0;

  for (dy = 1; dy >= -1; dy--) {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++) {
      if (dx || dy) {
        nx = x + dx;
        DataCell nc = data_[nx][ny];
        int voronoi_type = nc.voronoi;
        bool b = (voronoi_type != voronoiPrune && voronoi_type != occupied);
        //	if (voronoi_type==occupied) obstacleCount++;
        f[i] = b;

        if (b) {
          voroCount++;
          if (!(dx && dy)) {
            voroCountFour++;
          }
        }

        if (b && !(dx && dy)) {
          count++;
        }
        //	if (voronoi_type<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }

  std::cout << "voroCount: " << voroCount << std::endl;
  std::cout << "voroCountFour: " << voroCountFour << std::endl;

  if (voroCount < 3 && voroCountFour == 1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected, P14
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) ||
      (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) {
    return keep;
  }

  // 8-connected, P28
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) {
    return keep;
  }

  // keep voro cells inside of blocks and retry later
  if (voroCount >= 5 && voroCountFour >= 3 &&
      data_[x][y].voronoi != voronoiRetry) {
    return retry;
  }

  return pruned;
}
