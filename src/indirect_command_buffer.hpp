#pragma once

#include "defines.hpp"
#include "resource.hpp"

namespace mtlpp {

enum class IndirectCommandType {
  IndirectCommandTypeDraw = 0x1 << 0,
  IndirectCommandTypeDrawIndexed = 0x1 << 1,
  IndirectCommandTypeDrawPatches = 0x1 << 2,
  IndirectCommandTypeDrawIndexedPatches = 0x1 << 3,
  IndirectCommandTypeConcurrentDispatch = 0x1 << 5,
  IndirectCommandTypeConcurrentDispatchThreads = 0x1 << 6,
};

class IndirectCommandBufferDescriptor : public ns::Object {
public:
  IndirectCommandBufferDescriptor();
  IndirectCommandBufferDescriptor(const ns::Handle &handle)
      : ns::Object(handle) {}

  IndirectCommandType GetCommandTypes() const;
  bool IsInheritBuffers() const;
  uint32_t GetMaxVertexBufferBindCount() const;
  uint32_t GetMaxFragmentBufferBindCount() const;
  bool IsInheritPipelineState() const;

  void SetCommandTypes(IndirectCommandType indirectCommandType);
  void SetInheritBuffers(bool inheritBuffers);
  void SetMaxVertexBufferBindCount(uint32_t maxVertexBufferBindCount);
  void SetMaxFragmentBufferBindCount(uint32_t maxFragmentBufferBindCount);
  void SetInheritPipelineState(bool inheritPipelineState);
};

class IndirectCommandBuffer : public Resource {
public:
  IndirectCommandBuffer() {}
  IndirectCommandBuffer(const ns::Handle &handle) : Resource(handle) {}

  uint32_t GetSize() const;
  IndirectRenderCommand IndirectRenderCommandAt(int commandIndex) const;
  IndirectComputeCommand IndirectComputeCommandAt(int commandIndex) const;
  void Reset(const ns::Range &range);
};
} // namespace mtlpp
