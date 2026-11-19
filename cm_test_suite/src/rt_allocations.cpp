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

#include "cm_test_suite/rt_allocations.hpp"

#include <atomic>
#include <cstdlib>
#include <new>

#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define CM_TEST_SUITE_COUNT_ALLOCATIONS 0
#else
#define CM_TEST_SUITE_COUNT_ALLOCATIONS 1
#endif

namespace cm_test_suite
{
namespace
{
thread_local int rt_depth = 0;
std::atomic<uint64_t> allocations{0};
}  // namespace

#if CM_TEST_SUITE_COUNT_ALLOCATIONS
namespace
{
void count_allocation()
{
  if (rt_depth > 0)
  {
    allocations.fetch_add(1, std::memory_order_relaxed);
  }
}
}  // namespace
#endif

bool rt_allocation_counting_available() { return CM_TEST_SUITE_COUNT_ALLOCATIONS != 0; }

uint64_t rt_allocations() { return allocations.load(); }

RtScope::RtScope() { ++rt_depth; }

RtScope::~RtScope() { --rt_depth; }

}  // namespace cm_test_suite

#if CM_TEST_SUITE_COUNT_ALLOCATIONS

void * operator new(std::size_t size)
{
  cm_test_suite::count_allocation();
  if (void * memory = std::malloc(size == 0 ? 1 : size))
  {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new(std::size_t size, std::align_val_t alignment)
{
  cm_test_suite::count_allocation();
  const auto align = static_cast<std::size_t>(alignment);
  const std::size_t rounded = (size + align - 1) / align * align;
  if (void * memory = std::aligned_alloc(align, rounded == 0 ? align : rounded))
  {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void * memory) noexcept { std::free(memory); }

void operator delete(void * memory, std::size_t) noexcept { std::free(memory); }

void operator delete(void * memory, std::align_val_t) noexcept { std::free(memory); }

void operator delete(void * memory, std::size_t, std::align_val_t) noexcept { std::free(memory); }

#endif
