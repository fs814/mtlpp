#pragma once

#include "defines.hpp"
#include "ns.hpp"

namespace mtlpp {
class Device;

class ArgumentEncoder : public ns::Object {
public:
  ArgumentEncoder() {}
  ArgumentEncoder(const ns::Handle &handle) : ns::Object(handle) {}

  Device GetDevice() const;
  ns::String GetLabel() const;

  void SetLabel(const ns::String &label);

  uint32_t GetEncodedLength() const;

  void SetArgumentBuffer(const Buffer &buffer, uint32_t offset);
  void SetIndirectCommandBuffer(
      const IndirectCommandBuffer &indirectCommandBuffer,
      AAPLArgumentBufferBufferID aaplArgumentBufferBufferID);
  // void EndEncoding();
};

} // namespace mtlpp
