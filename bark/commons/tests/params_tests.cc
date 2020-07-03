// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <fstream>
#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"

#include "bark/commons/distribution/distributions_1d.hpp"
#include "bark/commons/params/setter_params.hpp"

TEST(setter_params, param_tests) {
  std::cout << "Start test\n";

  bark::commons::SetterParams params(true);

  params.SetReal("Test::2", 0.5f);
  EXPECT_EQ(params.GetReal("Test::2", "", 1.0f), 0.5f);

  params.SetInt("Test::2", 2);
  EXPECT_EQ(params.GetInt("Test::2", "", 1), 2);

  params.SetBool("Test::5", true);
  EXPECT_EQ(params.GetBool("Test::5", "", false), true);

  params.SetListListFloat("Test::2", {{0, 1}, {0, 2}, {0.5, 1.5}});
  EXPECT_EQ(params.GetListListFloat("Test::2", "", {{0, 1}, {0, 2}}),
            std::vector<std::vector<float>>({{0, 1}, {0, 2}, {0.5, 1.5}}));

  auto child_params = params.AddChild("Child");
  child_params->SetInt("Test::4", 21);
  EXPECT_EQ(params.GetInt("Child::Test::4", "", 1), 21);

  child_params->SetBool("Test::5", true);
  EXPECT_EQ(params.GetBool("Child::Test::5", "", false), true);

  auto child_params2 = child_params->AddChild("Child2");
  child_params2->SetReal("Test::22", 4031.1f);
  EXPECT_EQ(params.GetReal("Child::Child2::Test::22", "", 1), 4031.1f);

  child_params2->SetListListFloat("Test::21", {{0, 22}, {1, 2}, {0.5, 1.5}});
  EXPECT_EQ(
      params.GetListListFloat("Child::Child2::Test::21", "", {{0, 1}, {0, 2}}),
      std::vector<std::vector<float>>({{0, 22}, {1, 2}, {0.5, 1.5}}));

  child_params2->SetListFloat("Test::248", {0, 1, 2});
  EXPECT_EQ(params.GetListFloat("Child::Child2::Test::248", "", {99, 100}),
            std::vector<float>({0, 1, 2}));

  params.SetReal("Test1::Test2::Test3::2", 123123.23783f);
  auto child_params3 = params.AddChild("Test1")->AddChild("Test2");
  EXPECT_EQ(child_params3->GetReal("Test3::2", "", 1.0f), 123123.23783f);
  auto child_params4 = child_params3->AddChild("Test3");
  EXPECT_EQ(child_params4->GetReal("2", "", 1.0f), 123123.23783f);

  // Setters used for distribution spec
  params.SetReal("Test2::25::123::LowerBound", -10.0f);
  params.SetReal("Test2::25::123::UpperBound", 10.0f);
  params.SetInt("Test2::25::123::RandomSeed", 1000);
  params.SetDistribution("Test2::25::123", "UniformDistribution1D");

  auto dist_uniform = params.GetDistribution(
      "Test2::25::123", "some description", "NormalDistribution1D");
  EXPECT_TRUE(typeid(*dist_uniform) ==
              typeid(bark::commons::UniformDistribution1D));
  EXPECT_EQ(dist_uniform->GetParams()->GetReal("LowerBound", "some description",
                                               2323.0),
            -10.0f);

  // No defaults for dist spec fiven
  bark::commons::SetterParams params2(false);
  auto dist_uniform2 = params2.GetDistribution(
      "Test2::25::123123", "some description", "NormalDistribution1D");
  EXPECT_TRUE(typeid(*dist_uniform2) ==
              typeid(bark::commons::NormalDistribution1D));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
