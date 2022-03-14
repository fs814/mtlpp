#include "indirect_render_command.hpp"
#include "buffer.hpp"
#include "depth_stencil.hpp"
#include "render_pipeline.hpp"
#include "sampler.hpp"
#include "texture.hpp"
#include <Metal/MTLBuffer.h>
#include <Metal/MTLIndirectRenderCommand.h>

namespace mtlpp {
void IndirectRenderCommand::SetRenderPipelineState(
    const RenderPipelineState &pipelineState) {
  Validate();
  [(__bridge id<MTLIndirectRenderCommand>)m_ptr
      setRenderPipelineState:(__bridge id<MTLRenderPipelineState>)
                                 pipelineState.GetPtr()];
}

void IndirectRenderCommand::SetVertexBuffer(const Buffer &buffer,
                                            uint32_t offset, uint32_t index) {
  Validate();
  [(__bridge id<MTLIndirectRenderCommand>)m_ptr
      setVertexBuffer:(__bridge id<MTLBuffer>)buffer.GetPtr()
               offset:offset
              atIndex:index];
}

void IndirectRenderCommand::SetFragmentBuffer(const Buffer &buffer,
                                              uint32_t offset, uint32_t index) {
  Validate();
  [(__bridge id<MTLIndirectRenderCommand>)m_ptr
      setFragmentBuffer:(__bridge id<MTLBuffer>)buffer.GetPtr()
                 offset:offset
                atIndex:index];
}

void IndirectRenderCommand::DrawPrimitives(PrimitiveType primitiveType,
                                           uint32_t vertexStart,
                                           uint32_t vertexCount,
                                           uint32_t instanceCount,
                                           uint32_t baseInstance) {
  Validate();
  [(__bridge id<MTLIndirectRenderCommand>)m_ptr
      drawPrimitives:MTLPrimitiveType(primitiveType)
         vertexStart:vertexStart
         vertexCount:vertexCount
       instanceCount:instanceCount
        baseInstance:baseInstance];
}

void IndirectRenderCommand::DrawIndexedPrimitives(
    PrimitiveType primitiveType, uint32_t indexCount, IndexType indexType,
    const Buffer &indexBuffer, uint32_t indexBufferOffset,
    uint32_t instanceCount, uint32_t baseVertex, uint32_t baseInstance) {
  Validate();
  [(__bridge id<MTLIndirectRenderCommand>)m_ptr
      drawIndexedPrimitives:MTLPrimitiveType(primitiveType)
                 indexCount:indexCount
                  indexType:MTLIndexType(indexType)
                indexBuffer:(__bridge id<MTLBuffer>)indexBuffer.GetPtr()
          indexBufferOffset:indexBufferOffset
              instanceCount:instanceCount
                 baseVertex:baseVertex
               baseInstance:baseInstance];
}

}
