// Copyright 2026 Team Hector
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STATUS_SAMPLER_HPP_
#define STATUS_SAMPLER_HPP_

#include <atomic>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cm_test_suite/cm_under_test.hpp"

namespace cm_test_suite
{
/// What the services and the activity topic showed at one moment.
struct StatusSample
{
  ReportedState reported;
  ReportedState activity;
};

/// Collects what the controller manager reports while a step runs.
class StatusSampler
{
public:
  explicit StatusSampler(CmUnderTest & cm);
  ~StatusSampler();

  StatusSampler(const StatusSampler &) = delete;
  StatusSampler & operator=(const StatusSampler &) = delete;

  void stop();

  /// Each differs from the one before. Only valid after stop().
  const std::vector<StatusSample> & samples() const { return samples_; }
  const std::optional<std::string> & error() const { return error_; }

private:
  std::atomic<bool> running_{true};
  std::vector<StatusSample> samples_;
  std::optional<std::string> error_;
  std::thread thread_;
};

}  // namespace cm_test_suite

#endif  // STATUS_SAMPLER_HPP_
