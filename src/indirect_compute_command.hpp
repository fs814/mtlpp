#pragma once

#include "defines.hpp"
#include "ns.hpp"

namespace mtlpp {
class IndirectComputeCommand : public ns::Object {
public:
  IndirectComputeCommand() {}
  IndirectComputeCommand(const ns::Handle &handle) : ns::Object(handle) {}

  void SetComputePipelineState(const ComputePipelineState &state)
      MTLPP_AVAILABLE(11_0, 13_0);
  void SetImageBlockWidth(uint32_t width, uint32_t height)
      MTLPP_AVAILABLE(11_0, 14_0);
  void SetKernelBuffer(const Buffer &buffer, uint32_t offset, uint32_t index)
      MTLPP_AVAILABLE(11_0, 13_0);
  void SetThreadgroupMemoryLength(uint32_t length, uint32_t index)
      MTLPP_AVAILABLE(11_0, 13_0);
  void SetStageInRegion(const Region &region) MTLPP_AVAILABLE(11_0, 13_0);
  void SetBarrier();
  void ClearBarrier();
  void ConcurrentDispatchThreadgroups(const Size &threadgroupsPerGrid,
                                      const Size &threadsPerThreadgroup);
  void ConcurrentDispatchThreads(const Size &threadPerGrid,
                                 const Size &threadPerThreadgroup);
  void Reset();
} MTLPP_AVAILABLE(11_0, 13_0);
} // namespace mtlpp
