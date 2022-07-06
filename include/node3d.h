/**
 * @file node3d.h
 * @author Karl Kurzer
 * @brief 
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>

#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/**
 * @brief A three dimensional node class that is at the heart of the algorithm.
 * Each node has a unique configuration (x, y, theta) in the configuration space C.
 */
class Node3D {
 public:
  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float phi, float g, float h, const Node3D* pred, int prim = 0) {
    x_ = x;
    y_ = y;
    phi_ = phi;
    g_ = g;
    h_ = h;
    pred_ = pred;
    o_ = false;
    c_ = false;
    idx_ = -1;
    prim_ = prim;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x_; }
  /// get the y position
  float getY() const { return y_; }
  /// get the heading theta
  float getPhi() const { return phi_; }
  /// get the cost-so-far (real value)
  float getG() const { return g_; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h_; }
  /// get the total estimated cost
  float getC() const { return g_ + h_; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx_; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim_; }
  /// determine whether the node is open
  bool isOpen() const { return o_; }
  /// determine whether the node is closed
  bool isClosed() const { return c_; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred_; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { x_ = x; }
  /// set the y position
  void setY(const float& y) { y_ = y; }
  /// set the heading theta
  void setT(const float& phi) { phi_ = phi; }
  /// set the cost-so-far (real value)
  void setG(const float& g) { g_ = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { h_ = h; }
  /// set and get the index of the node in the 3D grid
  int setIdx(int width, int height) { idx_ = (int)(phi_ / Constants::deltaHeadingRad) * width * height + (int)(y_) * width + (int)(x_); return idx_;}
  /// open the node
  void open() { o_ = true; c_ = false;}
  /// close the node
  void close() { c_ = true; o_ = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D* pred) { pred_ = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const Node3D& rhs) const;

  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  bool isInRange(const Node3D& goal) const;

  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const;

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node3D* createSuccessor(const int i);

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static const float dx[];
  /// Possible movements in the y direction
  static const float dy[];
  /// Possible movements regarding heading theta
  static const float dt[];

 private:
  /// the x position
  float x_;
  /// the y position
  float y_;
  /// the heading theta
  float phi_;
  /// the cost-so-far
  float g_;
  /// the cost-to-go
  float h_;
  /// the index of the node in the 3D array
  int idx_;
  /// the open value
  bool o_;
  /// the closed value
  bool c_;
  /// the motion primitive of the node
  int prim_;
  /// the predecessor pointer
  const Node3D* pred_;
};

}  // namespace HybridAStar
#endif // NODE3D_H
