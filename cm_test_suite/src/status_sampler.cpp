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

#include "status_sampler.hpp"

#include <chrono>
#include <exception>
#include <utility>

namespace cm_test_suite
{
namespace
{
constexpr std::chrono::microseconds SAMPLE_PERIOD{200};

bool same(const ReportedState & a, const ReportedState & b)
{
  return a.controllers == b.controllers && a.hardware == b.hardware &&
         a.transitioning == b.transitioning;
}
}  // namespace

StatusSampler::StatusSampler(CmUnderTest & cm)
: thread_(
    [this, &cm]
    {
      while (running_)
      {
        try
        {
          StatusSample sample{cm.reported(), cm.activity()};
          if (
            samples_.empty() || !same(sample.reported, samples_.back().reported) ||
            !same(sample.activity, samples_.back().activity))
          {
            samples_.push_back(std::move(sample));
          }
        }
        catch (const std::exception & e)
        {
          error_ = e.what();
          return;
        }
        std::this_thread::sleep_for(SAMPLE_PERIOD);
      }
    })
{
}

StatusSampler::~StatusSampler() { stop(); }

void StatusSampler::stop()
{
  running_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

}  // namespace cm_test_suite
