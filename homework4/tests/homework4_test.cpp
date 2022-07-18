#include <gtest/gtest.h>
#include <ros/ros.h>
#include <fstream>
#include <ros/package.h>
#include <jsoncpp/json/json.h>
#include "ObjectEkf.hpp"
#include <eigen3/Eigen/Dense>


Json::Value test_vectors;

TEST(Homework4, instantiation)
{
  double x0 = 1.0;
  double y0 = 2.0;
  double xvel0 = -0.4;
  double yvel0 = -1.0;
  std::string frame_id = "test_frame";
  int id = 33;
  ros::Time current_time = ros::Time::now();

  auto uut = std::make_shared<homework4::ObjectEkf>(x0, xvel0, y0, yvel0, id, current_time, frame_id);
  ASSERT_STREQ(frame_id.c_str(), uut->frame_id_.c_str());
  ASSERT_EQ(id, uut->id_);
  ASSERT_EQ(current_time, uut->estimate_stamp_);
  ASSERT_EQ(current_time, uut->spawn_stamp_);
  ASSERT_EQ(current_time, uut->measurement_stamp_);

  Eigen::Vector4d test_vect;
  test_vect << x0, xvel0, y0, yvel0;
  ASSERT_TRUE(test_vect.isApprox(uut->X_));
  ASSERT_TRUE(Eigen::Matrix4d::Identity().isApprox(uut->P_));

  auto est_output = uut->getEstimate();
  EXPECT_DOUBLE_EQ(est_output.pose.position.x, test_vect(0)) << "X position estimate output is incorrect";
  EXPECT_DOUBLE_EQ(est_output.pose.position.y, test_vect(2)) << "Y position estimate output is incorrect";
  EXPECT_DOUBLE_EQ(est_output.velocity.linear.x, test_vect(1)) << "X velocity estimate output is incorrect";
  EXPECT_DOUBLE_EQ(est_output.velocity.linear.y, test_vect(3)) << "Y velocity estimate output is incorrect";
}

TEST(Homework4, q_and_r)
{
  Eigen::Matrix4d q;
  Eigen::Matrix2d r;
  for (int i = 0; i < 16; i++) {
    q.data()[i] = test_vectors["q_and_r"]["Q"][i].asDouble();
  }
  for (int i = 0; i < 4; i++) {
    r.data()[i] = test_vectors["q_and_r"]["R"][i].asDouble();
  }
  double q_pos = sqrt(q(0, 0));
  double q_vel = sqrt(q(1, 1));
  double r_pos = sqrt(r(0, 0));

  auto uut = std::make_shared<homework4::ObjectEkf>(0, 0, 0, 0, 0, ros::Time::now(), "test_frame");
  uut->setQ(q_pos, q_vel);
  uut->setR(r_pos);
  EXPECT_TRUE(q.isApprox(uut->Q_, 1e-9)) << "Q matrix not set properly";
  EXPECT_TRUE(r.isApprox(uut->R_, 1e-9)) << "R matrix not set properly";
}

TEST(Homework4, state_prediction)
{
  auto uut = std::make_shared<homework4::ObjectEkf>(0, 0, 0, 0, 0, ros::Time::now(), "test_frame");
  double dt = test_vectors["state_prediction"]["dt"].asDouble();
  for (int i = 0; i < test_vectors["state_prediction"]["invals"].size(); i++) {
    const Json::Value& inval = test_vectors["state_prediction"]["invals"][i];
    const Json::Value& outval = test_vectors["state_prediction"]["outvals"][i];
    homework4::StateVector in_vect;
    homework4::StateVector out_vect;
    for (int j = 0; j < 4; j++) {
      in_vect.data()[j] = inval[j].asDouble();
      out_vect.data()[j] = outval[j].asDouble();
    }
    ASSERT_TRUE(out_vect.isApprox(uut->statePrediction(dt, in_vect))) << "State prediction function incorrect";
  }
}

TEST(Homework4, state_jacobian)
{
  auto uut = std::make_shared<homework4::ObjectEkf>(0, 0, 0, 0, 0, ros::Time::now(), "test_frame");
  for (int i = 0; i < test_vectors["state_jacobian"]["invals"].size(); i++) {
    const Json::Value& outval = test_vectors["state_jacobian"]["outvals"][i];
    homework4::StateMatrix out_mat;
    for (int j = 0; j < 16; j++) {
      out_mat.data()[j] = outval[j].asDouble();
    }
    ASSERT_TRUE(out_mat.isApprox(uut->stateJacobian(test_vectors["state_jacobian"]["invals"][i].asDouble(), homework4::StateVector::Zero()))) << "State Jacobian function incorrect";
  }
}

TEST(Homework4, cov_prediction)
{
  auto uut = std::make_shared<homework4::ObjectEkf>(0, 0, 0, 0, 0, ros::Time::now(), "test_frame");
  for (int i = 0; i < test_vectors["cov_prediction"]["invals"].size(); i++) {
    const Json::Value& inval = test_vectors["cov_prediction"]["invals"][i];
    const Json::Value& outval = test_vectors["cov_prediction"]["outvals"][i];
    const Json::Value& a_val = test_vectors["cov_prediction"]["a_vals"][i];
    const Json::Value& q_val = test_vectors["cov_prediction"]["q_vals"][i];

    homework4::StateMatrix out_mat;
    homework4::StateMatrix a_mat;
    homework4::StateMatrix q_mat;
    homework4::StateMatrix in_mat;
    for (int j = 0; j < 16; j++) {
      out_mat.data()[j] = outval[j].asDouble();
      in_mat.data()[j] = inval[j].asDouble();
      a_mat.data()[j] = a_val[j].asDouble();
      q_mat.data()[j] = q_val[j].asDouble();
    }
    ASSERT_TRUE(out_mat.isApprox(uut->covPrediction(a_mat, q_mat, in_mat))) << "Covariance prediction function incorrect";
  }
}

TEST(Homework4, update_filter)
{
  ros::Time start_time = ros::Time::now();
  auto uut = std::make_shared<homework4::ObjectEkf>(test_vectors["update_filter"]["start_x"].asDouble(),
                                                    test_vectors["update_filter"]["start_xdot"].asDouble(),
                                                    test_vectors["update_filter"]["start_y"].asDouble(),
                                                    test_vectors["update_filter"]["start_ydot"].asDouble(),
                                                    0, start_time, "test_frame");

  for (int i = 0; i < test_vectors["update_filter"]["invals"].size(); i++) {
    const Json::Value& inval = test_vectors["update_filter"]["invals"][i];
    const Json::Value& outval = test_vectors["update_filter"]["outvals"][i];

    homework4::StateVector out_state;
    homework4::StateMatrix out_cov;
    for (int j = 0; j < 4; j++) {
      out_state.data()[j] = outval["state"][j].asDouble();
    }
    for (int j = 0; j < 16; j++) {
      out_cov.data()[j] = outval["cov"][j].asDouble();
    }

    avs_lecture_msgs::TrackedObject obj;
    obj.header.stamp = start_time + ros::Duration(inval["time"].asDouble());
    obj.pose.position.x = inval["x"].asDouble();
    obj.pose.position.y = inval["y"].asDouble();
    uut->updateFilterMeasurement(obj);

    ASSERT_TRUE(out_state.isApprox(uut->X_)) << "State estimate update is incorrect";
    ASSERT_TRUE(out_cov.isApprox(uut->P_)) << "State covariance update is incorrect";
  }

}

int main(int argc, char** argv)
{

  // Load test vector data
  std::string filename = ros::package::getPath("homework4") + "/tests/test_vectors.json";
  std::ifstream f(filename);
  if (f.fail()) {
    std::cout << "Failed to load test vector file " << filename << std::endl;
    return 1;
  } else {
    std::stringstream ss;
    ss << f.rdbuf();
    Json::Reader reader;
    bool parse_successful = reader.parse(ss.str(), test_vectors);
    if (!parse_successful) {
      std::cout << "Failed to parse test vector file" << std::endl;
      return 1;
    }
  }

  // Run tests
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
