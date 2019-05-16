// Author: Tucker Haydon

#undef NDEBUG
#include <cassert>

#include <iostream>
#include <memory>

#include "node.h"
#include "node_eigen.h"
#include "directed_edge.h"
#include "graph.h"

using namespace game_engine;

void test_NodeEigen() {
  NodeEigen<2> n1(Eigen::Vector2d(0,0));
}

void test_Node2D() {
  { // Check equality
    Eigen::Matrix<double, 2, 1> m1, m2, m3;
    m1 << 1,2;
    m2 << 1,2;
    m3 << 1,3;
    const Node2D n1(m1);
    const Node2D n2(m2);
    const Node2D n3(m3);
    assert(true == (n1 == n2));
    assert(false == (n1 == n3));
  }

  { // Check hash equality
    Eigen::Matrix<double, 2, 1> m1, m2, m3;
    m1 << 1,2;
    m2 << 1,2;
    m3 << 1.0001,2.0001;
    const Node2D n1(m1);
    const Node2D n2(m2);
    const Node2D n3(m3);
    assert(true == (n1.Hash() == n2.Hash()));
    assert(false == (n1.Hash() == n3.Hash()));
  }
}

void test_Node3D() {
  { // Check equality
    Eigen::Matrix<double, 3, 1> m1, m2, m3;
    m1 << 1,2,1;
    m2 << 1,2,1;
    m3 << 1,3,1;
    const Node3D n1(m1);
    const Node3D n2(m2);
    const Node3D n3(m3);
    assert(true == (n1 == n2));
    assert(false == (n1 == n3));
  }

  { // Check hash equality
    Eigen::Matrix<double, 3, 1> m1, m2, m3;
    m1 << 1,2,1;
    m2 << 1,2,1;
    m3 << 1.0001,2.0001,1;
    const Node3D n1(m1);
    const Node3D n2(m2);
    const Node3D n3(m3);assert(true == (n1.Hash() == n2.Hash()));
    assert(false == (n1.Hash() == n3.Hash()));
  }
}

void test_DirectedEdge2D() {
  { // Construction
    Eigen::Matrix<double, 2, 1> mSource;
    mSource << 1,2;
    Eigen::Matrix<double, 2, 1> mSink;
    mSink << 2,2;
    
    const auto source = std::make_shared<Node2D>(mSource);
    const auto sink = std::make_shared<Node2D>(mSink);
    const double cost = 3.14;
    const DirectedEdge2D edge(source, sink, cost);
    assert(true == (*edge.Source() == *source));
    assert(true == (*edge.Sink() == *sink));
    assert(true == (cost == edge.Cost()));
  }
}

void test_Graph2D() {
  { // Test graph access
    Eigen::Matrix<double, 2, 1> m1, m2, m3, m4, m5, m6;
    m1 << 1,2;
    m2 << 2,2;
    m3 << 3,2;
    m4 << 2,5;
    m5 << 1,2;
    m6 << 0,0;

    const auto n1 = std::make_shared<Node2D>(m1);
    const auto n2 = std::make_shared<Node2D>(m2);
    const auto n3 = std::make_shared<Node2D>(m3);
    const auto n4 = std::make_shared<Node2D>(m4);
    const DirectedEdge2D edge1(n1,n2), edge2(n1,n3), edge3(n3,n4), edge4(n2,n4);
    const Graph2D graph({edge1, edge2, edge3, edge4});

    const auto n5 = std::make_shared<Node2D>(m5);
    const auto n6 = std::make_shared<Node2D>(m6);
    assert(2 == graph.Edges(n5).size());
    assert(0 == graph.Edges(n6).size());  
  }

  { // Test neighbors function
    Eigen::Matrix<double, 2, 1> m1, m2;
    m1 << 1,2;
    m2 << 2,2;

    const auto n1 = std::make_shared<Node2D>(m1);
    const auto n2 = std::make_shared<Node2D>(m2);
    const DirectedEdge2D edge1(n1,n2);
    const Graph2D graph({edge1});

    assert(1 == graph.Neighbors(n1).size());
    assert(*n2 == *graph.Neighbors(n1)[0]);
  }
}

int main(int argc, char** argv) {
  test_NodeEigen();
  test_Node2D();
  test_Node3D();
  test_DirectedEdge2D();
  test_Graph2D();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
