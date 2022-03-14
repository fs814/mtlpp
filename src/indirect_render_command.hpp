#pragma once

#include "defines.hpp"
#include "ns.hpp"

namespace mtlpp {
class IndirectRenderCommand : public ns::Object {
public:
  IndirectRenderCommand() {}
  IndirectRenderCommand(const ns::Handle &handle) : ns::Object(handle) {}

  void SetRenderPipelineState(const RenderPipelineState &pipelineState);
  void SetVertexBuffer(const Buffer &buffer, uint32_t offset, uint32_t index);
  void SetFragmentBuffer(const Buffer &buffer, uint32_t offset, uint32_t index);
  void DrawPrimitives(PrimitiveType primitiveType, uint32_t vertexStart,
                      uint32_t vertexCount, uint32_t instanceCount,
                      uint32_t baseInstance);
  void DrawIndexedPrimitives(PrimitiveType primitiveType, uint32_t indexCount,
                             IndexType indexType, const Buffer &indexBuffer,
                             uint32_t indexBufferOffset, uint32_t instanceCount,
                             uint32_t baseVertex, uint32_t baseInstance);
  // void DrawPatches(uint32_t numberOfPatchControlPoints, uint32_t patchStart,
  //                  uint32_t patchCount, const Buffer &patchIndexBuffer,
  //                  uint32_t patchIndexBufferOffset, uint32_t instanceCount,
  //                  uint32_t baseInstance) MTLPP_AVAILABLE(10_12, 10_0);
  // void DrawPatches(uint32_t numberOfPatchControlPoints,
  //                  const Buffer &patchIndexBuffer,
  //                  uint32_t patchIndexBufferOffset,
  //                  const Buffer &indirectBuffer, uint32_t indirectBufferOffset)
  //     MTLPP_AVAILABLE(10_12, NA);
  // void DrawIndexedPatches(uint32_t numberOfPatchControlPoints,
  //                         uint32_t patchStart, uint32_t patchCount,
  //                         const Buffer &patchIndexBuffer,
  //                         uint32_t patchIndexBufferOffset,
  //                         const Buffer &controlPointIndexBuffer,
  //                         uint32_t controlPointIndexBufferOffset,
  //                         uint32_t instanceCount, uint32_t baseInstance)
  //     MTLPP_AVAILABLE(10_12, 10_0);
  // void DrawIndexedPatches(
  //     uint32_t numberOfPatchControlPoints, const Buffer &patchIndexBuffer,
  //     uint32_t patchIndexBufferOffset, const Buffer &controlPointIndexBuffer,
  //     uint32_t controlPointIndexBufferOffset, const Buffer &indirectBuffer,
  //     uint32_t indirectBufferOffset) MTLPP_AVAILABLE(10_12, NA);
  // void Reset();
} MTLPP_AVAILABLE(10_14, 12_0);
} // namespace mtlpp
