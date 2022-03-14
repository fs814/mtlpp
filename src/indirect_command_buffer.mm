#include "indirect_command_buffer.hpp"

#include <Metal/MTLIndirectCommandBuffer.h>

namespace mtlpp {
IndirectCommandBufferDescriptor::IndirectCommandBufferDescriptor()
    : ns::Object(ns::Handle{
        (__bridge void*)[[MTLIndirectCommandBufferDescriptor alloc] init]
            }){}

IndirectCommandType IndirectCommandBufferDescriptor::GetCommandTypes() const {
  Validate();
  return IndirectCommandType(
      [(__bridge MTLIndirectCommandBufferDescriptor *)m_ptr commandTypes]);
}

bool IndirectCommandBufferDescriptor::IsInheritBuffers() const {
    Validate();
    return [(__bridge MTLIndirectCommandBufferDescriptor *)m_ptr inheritBuffers];
}

bool IndirectCommandBufferDescriptor::IsInheritPipelineState() const {
    Validate();
    return [(__bridge MTLIndirectCommandBufferDescriptor *)m_ptr inheritPipelineState];
}

uint32_t IndirectCommandBufferDescriptor::GetMaxVertexBufferBindCount() const {
    Validate();
    return [(__bridge MTLIndirectCommandBufferDescriptor*)m_ptr maxVertexBufferBindCount];
}

uint32_t IndirectCommandBufferDescriptor::GetMaxFragmentBufferBindCount() const {
    Validate();
    return [(__bridge MTLIndirectCommandBufferDescriptor*)m_ptr maxFragmentBufferBindCount];
}

void IndirectCommandBufferDescriptor::SetCommandTypes(IndirectCommandType indirectCommandType) {
  Validate();
  [(__bridge MTLIndirectCommandBufferDescriptor *)m_ptr
      setCommandTypes:MTLIndirectCommandType(indirectCommandType)];
}

void IndirectCommandBufferDescriptor::SetInheritBuffers(bool inheritBuffers){
    Validate();
    [(__bridge MTLIndirectCommandBufferDescriptor *)m_ptr setInheritBuffers:inheritBuffers];
}

void IndirectCommandBufferDescriptor::SetInheritPipelineState(bool inheritPipelineState){
    Validate();
    [(__bridge MTLIndirectCommandBufferDescriptor*)m_ptr setInheritPipelineState:inheritPipelineState];
}

void IndirectCommandBufferDescriptor::SetMaxVertexBufferBindCount(uint32_t maxVertexBufferBindCount){
    Validate();
    [(__bridge MTLIndirectCommandBufferDescriptor*)m_ptr setMaxVertexBufferBindCount:maxVertexBufferBindCount];
}

void IndirectCommandBufferDescriptor::SetMaxFragmentBufferBindCount(uint32_t maxFragmentBufferBindCount){
    Validate();
    [(__bridge MTLIndirectCommandBufferDescriptor*)m_ptr setMaxFragmentBufferBindCount:maxFragmentBufferBindCount];
}

uint32_t IndirectCommandBuffer::GetSize() const {
  Validate();
  return [(__bridge id<MTLIndirectCommandBuffer>)m_ptr size];
}

IndirectRenderCommand
IndirectCommandBuffer::IndirectRenderCommandAt(int commandIndex) const {
  Validate();
  return ns::Handle{
      (__bridge void *)[(__bridge id<MTLIndirectCommandBuffer>)m_ptr
          indirectRenderCommandAt:commandIndex]};
}

IndirectComputeCommand
IndirectCommandBuffer::IndirectComputeCommandAt(int commandIndex) const {
  Validate();
  return ns::Handle{
      (__bridge void *)[(__bridge id<MTLIndirectCommandBuffer>)m_ptr
          indirectComputeCommandAt:commandIndex]};
}

void IndirectCommandBuffer::Reset(const ns::Range &range) {
  Validate();
  [(__bridge id<MTLIndirectCommandBuffer>)m_ptr reset:range];
}



}
