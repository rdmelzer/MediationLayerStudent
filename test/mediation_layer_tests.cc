// Author: Tucker Haydon


#include <cstdlib>
#include <iostream>
#include <Eigen/StdVector>

#undef NDEBUG
#include <cassert>

#include "trajectory.h"
#include "trajectory_warden.h"
#include "quad_state.h"
#include "quad_state_warden.h"

using namespace game_engine;

void test_TrajectoryWarden() {
  { // Trivial
    TrajectoryWarden warden;

    Trajectory dummy_trajectory;
    assert(0 == warden.Keys().size());
    assert(false == warden.Read("", dummy_trajectory));
    assert(false == warden.Write("", dummy_trajectory));
    assert(false == warden.Await("", dummy_trajectory));
  }

  { // Test read/write
    TrajectoryWarden warden;
    Eigen::Matrix<double, 11, 1> m;
    m << 1,1,1,1,1,1,1,1,1,1,1;
    Trajectory trajectory_write({m});
    assert(true == warden.Register("test"));
    assert(true == warden.Write("test", trajectory_write));

    Trajectory trajectory_read;
    assert(true == warden.Read("test", trajectory_read));
    assert(trajectory_read.Size() == trajectory_write.Size());
    assert(trajectory_read.PVAYT(0).isApprox(trajectory_write.PVAYT(0)));
  }

}

void test_Trajectory() {
  { // Trivial
    Trajectory trajectory;
    assert(0 == trajectory.Size());
  }

  { // Test access
    TrajectoryVector3D hist = {};
    Eigen::Matrix<double, 11, 1> m1;
    m1 << 1,1,1,2,2,2,3,3,3,0.1,0.2;
    hist.push_back(m1);
    Trajectory trajectory(hist);

    assert(Eigen::Vector3d(1,1,1).isApprox(trajectory.Position(0)));
    assert(Eigen::Vector3d(2,2,2).isApprox(trajectory.Velocity(0)));
    assert(Eigen::Vector3d(3,3,3).isApprox(trajectory.Acceleration(0)));
    assert(0.1 == trajectory.Yaw(0));
    assert(0.2 == trajectory.Time(0));

    Eigen::Matrix<double, 9, 1> m2;
    m2 << 1,1,1,2,2,2,3,3,3;
    assert((m2).isApprox(trajectory.PVA(0)));
    
    Eigen::Matrix<double, 11, 1> m3;
    m3 << 1,1,1,2,2,2,3,3,3,0.1,0.2;
    assert((m3).isApprox(trajectory.PVAYT(0)));
  }
}

void test_QuadState() {
  { // Trivial
    Eigen::Matrix<double, 13, 1> m;
    m << 1,1,1,2,2,2,1,0,0,0,1,2,3;
    QuadState state(m);
    assert((Eigen::Vector3d(1,1,1)).isApprox(state.Position()));
    assert((Eigen::Vector3d(2,2,2)).isApprox(state.Velocity()));
    assert((Eigen::Vector4d(1,0,0,0)).isApprox(state.Orientation()));
    assert((Eigen::Vector3d(1,2,3)).isApprox(state.Twist()));
  }
}

void test_QuadStateWarden() {
  { // Trivial
    QuadStateWarden warden;

    QuadState dummy_state;
    assert(0 == warden.Keys().size());
    assert(false == warden.Read("", dummy_state));
    assert(false == warden.Write("", dummy_state));
    assert(false == warden.Await("", dummy_state));
  }

  { // Test read/write
    QuadStateWarden warden;
    Eigen::Matrix<double, 13, 1> m;
    m << 0,0,0,0,0,0,1,0,0,0,0,0,0;
    QuadState state_write({m});
    assert(true == warden.Register("test"));
    assert(true == warden.Write("test", state_write));

    QuadState state_read;
    assert(true == warden.Read("test", state_read));
    assert(state_read.Position().isApprox(state_write.Position()));
    assert(state_read.Velocity().isApprox(state_write.Velocity()));
    assert(state_read.Orientation().isApprox(state_write.Orientation()));
    assert(state_read.Twist().isApprox(state_write.Twist()));
  }
}

int main(int argc, char** argv) {
  test_Trajectory();
  test_TrajectoryWarden();
  test_QuadState();
  test_QuadStateWarden();

  std::cout << "All tests passed!" << std::endl;
  return EXIT_SUCCESS;
}
