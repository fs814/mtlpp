#pragma once

#include "argument_encoder.hpp"
#include "buffer.hpp"
#include "defines.hpp"
#include "device.hpp"
#include "indirect_command_buffer.hpp"

#include "Metal/MTLArgumentEncoder.h"

namespace mtlpp {
Device ArgumentEncoder::GetDevice() const {
  Validate();
  return ns::Handle{
      (__bridge void *)[(__bridge id<MTLArgumentEncoder>)m_ptr device]};
}

ns::String ArgumentEncoder::GetLabel() const {
  Validate();
  return ns::Handle{
      (__bridge void *)[(__bridge id<MTLArgumentEncoder>)m_ptr label]};
}

void ArgumentEncoder::SetLabel(const ns::String &label) {
  Validate();
  [(__bridge id<MTLArgumentEncoder>)m_ptr
      setLabel:(__bridge NSString *)label.GetPtr()];
}

uint32_t ArgumentEncoder::GetEncodedLength() const {
  Validate();
  return uint32_t([(__bridge id<MTLArgumentEncoder>)m_ptr encodedLength]);
}

void ArgumentEncoder::SetArgumentBuffer(const Buffer &buffer, uint32_t offset) {
  Validate();
  [(__bridge id<MTLArgumentEncoder>)m_ptr
      setArgumentBuffer:(__bridge id<MTLBuffer>)buffer.GetPtr()
                 offset:offset];
}

void ArgumentEncoder::SetIndirectCommandBuffer(
    const IndirectCommandBuffer &indirectCommandBuffer,
    AAPLArgumentBufferBufferID aaplArgumentBufferBufferID) {
  Validate();
  [(__bridge id<MTLArgumentEncoder>)m_ptr
      setIndirectCommandBuffer:(__bridge id<MTLIndirectCommandBuffer>)
                                   indirectCommandBuffer.GetPtr()
                       atIndex:(int)aaplArgumentBufferBufferID];
}

}
