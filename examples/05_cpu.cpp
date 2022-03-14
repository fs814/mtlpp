#include "../mtlpp.hpp"
#include "window.hpp"
#include <math.h>
#include <simd/simd.h>

#define AAPLNumObjects 15
static const uint32_t AAPLMaxFramesInFlight = 3;

mtlpp::Device g_device;
mtlpp::CommandQueue g_commandQueue;
mtlpp::Buffer g_vertexBuffer;
mtlpp::Buffer g_vertexBufferArr[AAPLNumObjects];
mtlpp::Buffer g_objectParameters;
mtlpp::Buffer g_frameStateBuffer[AAPLMaxFramesInFlight];
mtlpp::Buffer g_indirectFrameStateBuffer;
mtlpp::RenderPipelineState g_renderPipelineState;
mtlpp::IndirectCommandBuffer g_indirectCommandBuffer;
vector_float2 g_aspectScale;


#define AAPLGridWidth 5
#define AAPLGridHeight ((AAPLNumObjects + AAPLGridWidth - 1) / AAPLGridWidth)
#define AAPLViewScale 0.25
#define AAPLObjectSize 2.0
#define AAPLObjecDistance 2.1

int g_inFlightIndex = 0;

struct float2 {
  float x;
  float y;

  float2(float x, float y) : x(x), y(y) {}

  float2 operator/(float scalar) const {
    float invScalar = 1.f / scalar;
    return float2(x * invScalar, y * invScalar);
  }

  float2 operator+(float scalar) const {
    return float2(x + scalar, y + scalar);
  }
  float2 operator+(float2 &rhs) const { return float2(x + rhs.x, y + rhs.y); }
  float2 operator-(float scalar) const {
    return float2(x - scalar, y - scalar);
  }
  float2 operator-() const { return float2(-x, -y); }
  float2 operator*(float scalar) const {
    return float2(x * scalar, y * scalar);
  }
  float2 operator*(double scalar) const {
    return float2(x * scalar, y * scalar);
  }
};

float2 operator*(double scalar, float2 float2var) {
  return float2(scalar * float2var.x, scalar * float2var.y);
}
float2 operator+(float2 lfs, float2 rhs) {
  return float2(lfs.x + rhs.y, lfs.y + rhs.y);
}

typedef struct {
  packed_float2 position;
  packed_float2 texcoord;
} AAPLVertex;

typedef struct AAPLFrameState {
  vector_float2 aspectScale;
} AAPLFrameState;

typedef struct AAPLObjectParameters {
  packed_float2 position;
} AAPLObjectParameters;

typedef enum AAPLVertexBufferIndex {
  AAPLVertexBufferIndexVertices,
  AAPLVertexBufferIndexObjectParams,
  AAPLVertexBufferIndexFrameState
} AAPLVertexBufferIndex;

typedef enum AAPLKernelBufferIndex {
  AAPLKernelBufferIndexFrameState,
  AAPLKernelBufferIndexObjectParams,
  AAPLKernelBufferIndexArguments
} AAPLKernelBufferIndex;

typedef enum AAPLArgumentBufferBufferID {
  AAPLArgumentBufferIDCommandBuffer,
  AAPLArgumentBufferIDObjectMesh
} AAPLArgumentBufferBufferID;

void Render(const Window &win) {
  mtlpp::CommandBuffer commandBuffer = g_commandQueue.CommandBuffer();

  mtlpp::BlitCommandEncoder blitCommandEncoder = commandBuffer.BlitCommandEncoder();
  blitCommandEncoder.Copy(g_frameStateBuffer[g_inFlightIndex], 0, g_indirectFrameStateBuffer, 0, g_indirectFrameStateBuffer.GetLength());
  blitCommandEncoder.EndEncoding();

  mtlpp::RenderPassDescriptor renderPassDesc = win.GetRenderPassDescriptor();
  if (renderPassDesc) {
    mtlpp::RenderCommandEncoder renderCommandEncoder =
        commandBuffer.RenderCommandEncoder(renderPassDesc);

    renderCommandEncoder.SetCullMode(mtlpp::CullMode::Back);
    renderCommandEncoder.SetRenderPipelineState(g_renderPipelineState);

    for(int i=0;i<AAPLNumObjects;i++){
      renderCommandEncoder.UseResource(g_vertexBufferArr[i], mtlpp::ResourceUsage::Read);
    }

    renderCommandEncoder.UseResource(g_objectParameters,mtlpp::ResourceUsage::Read);

    renderCommandEncoder.UseResource(g_indirectFrameStateBuffer,mtlpp::ResourceUsage::Read);

    renderCommandEncoder.ExecuteCommandsInBuffer(g_indirectCommandBuffer, ns::Range(0, AAPLNumObjects));

    renderCommandEncoder.EndEncoding();

    // renderCommandEncoder.SetRenderPipelineState(g_renderPipelineState);
    // renderCommandEncoder.SetVertexBuffer(g_vertexBuffer, 0, 0);
    // renderCommandEncoder.Draw(mtlpp::PrimitiveType::Triangle, 0, 3);
    // renderCommandEncoder.EndEncoding();
    commandBuffer.Present(win.GetDrawable());
  }

  commandBuffer.Commit();
  commandBuffer.WaitUntilCompleted();
}

void NewGearMeshWithNumTeeth(int id, int numTeeth) {
  static const float innerRatio = 0.8;
  static const float toothWidth = 0.25;
  static const float toothSlope = 0.2;

  uint32_t numVertices = numTeeth * 12;
  uint32_t bufferSize = sizeof(AAPLVertex) * numVertices;

  // mtlpp::Buffer vertexBuffer = g_device.NewBuffer(
  //    bufferSize, mtlpp::ResourceOptions::CpuCacheModeDefaultCache); //??

  g_vertexBufferArr[id] = g_device.NewBuffer(
      bufferSize, mtlpp::ResourceOptions::CpuCacheModeDefaultCache); //??

  AAPLVertex *meshVertices = (AAPLVertex *)g_vertexBufferArr[id].GetContents();

  const double angle = 2.0 * M_PI / (double)numTeeth;
  static const packed_float2 origin = {0.0, 0.0};
  int vtx = 0;

  // Build triangles for teeth of gear
  for (int tooth = 0; tooth < numTeeth; tooth++) {
    // Calculate angles for tooth and groove
    const float toothStartAngle = tooth * angle;
    const float toothTip1Angle = (tooth + toothSlope) * angle;
    const float toothTip2Angle = (tooth + toothSlope + toothWidth) * angle;
    ;
    const float toothEndAngle = (tooth + 2 * toothSlope + toothWidth) * angle;
    const float nextToothAngle = (tooth + 1.0) * angle;

    // Calculate positions of vertices needed for the tooth
    const packed_float2 groove1 = {sin(toothStartAngle) * innerRatio,
                            cos(toothStartAngle) * innerRatio};
    const packed_float2 tip1 = {sin(toothTip1Angle), cos(toothTip1Angle)};
    const packed_float2 tip2 = {sin(toothTip2Angle), cos(toothTip2Angle)};
    const packed_float2 groove2 = {sin(toothEndAngle) * innerRatio,
                            cos(toothEndAngle) * innerRatio};
    const packed_float2 nextGroove = {sin(nextToothAngle) * innerRatio,
                               cos(nextToothAngle) * innerRatio};

    // Right top triangle of tooth
    meshVertices[vtx].position = groove1;
    meshVertices[vtx].texcoord = (groove1 + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = tip1;
    meshVertices[vtx].texcoord = (tip1 + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = tip2;
    meshVertices[vtx].texcoord = (tip2 + 1.0) / 2.0;
    vtx++;

    // Left bottom triangle of tooth
    meshVertices[vtx].position = groove1;
    meshVertices[vtx].texcoord = (groove1 + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = tip2;
    meshVertices[vtx].texcoord = (tip2 + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = groove2;
    meshVertices[vtx].texcoord = (groove2 + 1.0) / 2.0;
    vtx++;

    // Slice of circle from bottom of tooth to center of gear
    meshVertices[vtx].position = origin;
    meshVertices[vtx].texcoord = (origin + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = groove1;
    meshVertices[vtx].texcoord = (groove1 + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = groove2;
    meshVertices[vtx].texcoord = (groove2 + 1.0) / 2.0;
    vtx++;

    // Slice of circle from the groove to the center of gear
    meshVertices[vtx].position = origin;
    meshVertices[vtx].texcoord = (origin + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = groove2;
    meshVertices[vtx].texcoord = (groove2 + 1.0) / 2.0;
    vtx++;

    meshVertices[vtx].position = nextGroove;
    meshVertices[vtx].texcoord = (nextGroove + 1.0) / 2.0;
    vtx++;
  }
  // return vertexBuffer;
}

int main() {
  /*
  const char shadersSrc[] = R"""(
        #include <metal_stdlib>
        using namespace metal;

        vertex float4 vertFunc(
            const device packed_float3* vertexArray [[buffer(0)]],
            unsigned int vID[[vertex_id]])
        {
            return float4(vertexArray[vID], 1.0);
        }

        fragment half4 fragFunc()
        {
            return half4(1.0, 0.0, 0.0, 1.0);
        }
    )""";
    */


  const char shadersSrc[] = R"""(
  #include <metal_stdlib>

  using namespace metal;

  // Include header shared between this Metal shader code and C code executing
  //Metal API commands
  //#include "AAPLShaderTypes.h"

#include <simd/simd.h>

// Constants shared between shader and C code
#define AAPLNumObjects    15

#define AAPLGridWidth     5
#define AAPLGridHeight    ((AAPLNumObjects+AAPLGridWidth-1)/AAPLGridWidth)

// Scale of each object when drawn
#define AAPLViewScale    0.25

// Because the objects are centered at origin, the scale appliced
#define AAPLObjectSize    2.0

// Distance between each object
#define AAPLObjecDistance 2.1

// Structure defining the layout of each vertex.  Shared between C code filling in the vertex data
//   and Metal vertex shader consuming the vertices
typedef struct
{
    packed_float2 position;
    packed_float2 texcoord;
} AAPLVertex;

// Structure defining the layout of variable changing once (or less) per frame
typedef struct AAPLFrameState
{
    vector_float2 aspectScale;
} AAPLFrameState;

// Structure defining parameters for each rendered object
typedef struct AAPLObjectPerameters
{
    packed_float2 position;
} AAPLObjectPerameters;

// Buffer index values shared between the vertex shader and C code
typedef enum AAPLVertexBufferIndex
{
    AAPLVertexBufferIndexVertices,
    AAPLVertexBufferIndexObjectParams,
    AAPLVertexBufferIndexFrameState
} AAPLVertexBufferIndex;

// Buffer index values shared between the compute kernel and C code
typedef enum AAPLKernelBufferIndex
{
    AAPLKernelBufferIndexFrameState,
    AAPLKernelBufferIndexObjectParams,
    AAPLKernelBufferIndexArguments
} AAPLKernelBufferIndex;

typedef enum AAPLArgumentBufferBufferID
{
    AAPLArgumentBufferIDCommandBuffer,
    AAPLArgumentBufferIDObjectMesh
} AAPLArgumentBufferBufferID;

  // Vertex shader outputs and per-fragment inputs
  struct RasterizerData
  {
      float4 position [[position]];
      float2 tex_coord;
  };

  vertex RasterizerData
  vertexShader(uint                         vertexID      [[ vertex_id ]],
               uint                         objectIndex   [[ instance_id ]],
               const device AAPLVertex *    vertices      [[
  buffer(AAPLVertexBufferIndexVertices) ]], const device AAPLObjectPerameters*
  object_params [[ buffer(AAPLVertexBufferIndexObjectParams) ]], constant
  AAPLFrameState *    frame_state   [[ buffer(AAPLVertexBufferIndexFrameState)
  ]])
  {
      RasterizerData out;

      float2 worldObjectPostion  = object_params[objectIndex].position;
      float2 modelVertexPosition = vertices[vertexID].position;
      float2 worldVertexPosition = modelVertexPosition + worldObjectPostion;
      float2 clipVertexPosition  = frame_state->aspectScale * AAPLViewScale *
  worldVertexPosition;

      out.position = float4(clipVertexPosition.x, clipVertexPosition.y, 0, 1);
      out.tex_coord = float2(vertices[vertexID].texcoord);

      return out;
  }

  fragment float4
  fragmentShader(RasterizerData in [[ stage_in ]])
  {
      float4 output_color = float4(in.tex_coord.x, in.tex_coord.y, 0, 1);

      return output_color;
  }
  )""";

  const float vertexData[] = {
      0.0f, 1.0f, 0.0f, -1.0f, -1.0f, 0.0f, 1.0f, -1.0f, 0.0f,
  };
  g_device = mtlpp::Device::CreateSystemDefaultDevice();
  g_commandQueue = g_device.NewCommandQueue();

  mtlpp::Library library =
      g_device.NewLibrary(shadersSrc, mtlpp::CompileOptions(), nullptr);
  mtlpp::Function vertFunc = library.NewFunction("vertexShader");
  mtlpp::Function fragFunc = library.NewFunction("fragmentShader");

  //mtlpp::Function vertFunc = library.NewFunction("vertFunc");
  //mtlpp::Function fragFunc = library.NewFunction("fragFunc");

  g_vertexBuffer =
      g_device.NewBuffer(vertexData, sizeof(vertexData),
                         mtlpp::ResourceOptions::CpuCacheModeDefaultCache);

  mtlpp::RenderPipelineDescriptor renderPipelineDesc;
  renderPipelineDesc.SetLabel("MyPipeline");
  renderPipelineDesc.SetSampleCount(1);
  renderPipelineDesc.SetVertexFunction(vertFunc);
  renderPipelineDesc.SetFragmentFunction(fragFunc);
  renderPipelineDesc.GetColorAttachments()[0].SetPixelFormat(
      mtlpp::PixelFormat::BGRA8Unorm);
  renderPipelineDesc.SetDepthAttachmentPixelFormat(
      mtlpp::PixelFormat::Depth32Float);
  renderPipelineDesc.SetSupportIndirectCommandBuffers(true);

  g_renderPipelineState =
      g_device.NewRenderPipelineState(renderPipelineDesc, nullptr);

  for (int objectIdx = 0; objectIdx <= AAPLNumObjects; objectIdx++) {
    uint32_t numTeeth = (objectIdx < 8) ? objectIdx + 3 : objectIdx * 3;
    // g_vertexBufferArr[objectIdx] = NewGearMeshWithNumTeeth(numTeeth);
    NewGearMeshWithNumTeeth(objectIdx, numTeeth);
  }

  uint32_t g_objectParameterArraySize =
      AAPLNumObjects * sizeof(AAPLObjectParameters);
  AAPLObjectParameters *params =
      (AAPLObjectParameters *)g_objectParameters.GetContents();

  static const vector_float2 gridDimensions = {AAPLGridWidth, AAPLGridHeight};
  const vector_float2 offset = (AAPLObjecDistance / 2.0) * (gridDimensions - 1);

  for (int objectIdx = 0; objectIdx < AAPLNumObjects; objectIdx++) {
    vector_float2 gridPos = {(float)(objectIdx % AAPLGridWidth),
                              (float)(objectIdx / AAPLGridWidth)};
    vector_float2 position = -offset + gridPos * AAPLObjecDistance;

    params[objectIdx].position = position;
  }

  for (int i = 0; i < AAPLMaxFramesInFlight; i++) {
    g_frameStateBuffer[i] = g_device.NewBuffer(
        sizeof(AAPLFrameState), mtlpp::ResourceOptions::StorageModeShared);

    AAPLFrameState* frameState = (AAPLFrameState*)g_frameStateBuffer[i].GetContents();
    g_aspectScale.x = 800.0 / 600.0;
    g_aspectScale.y = 1.0;

    frameState->aspectScale = g_aspectScale;
    //_frameStateBuffer[i].label =
    //    [NSString stringWithFormat:@"Frame state buffer %d", i];
  }

  g_indirectFrameStateBuffer = g_device.NewBuffer(
      sizeof(AAPLFrameState), mtlpp::ResourceOptions::StorageModePrivate);

  mtlpp::IndirectCommandBufferDescriptor indirectCommandBufferDesc;
  indirectCommandBufferDesc.SetCommandTypes(
      mtlpp::IndirectCommandType::IndirectCommandTypeDraw);
  indirectCommandBufferDesc.SetInheritBuffers(false);
  indirectCommandBufferDesc.SetMaxVertexBufferBindCount(3);
  indirectCommandBufferDesc.SetMaxFragmentBufferBindCount(0);
  indirectCommandBufferDesc.SetInheritPipelineState(true);

  g_indirectCommandBuffer = g_device.NewIndirectCommandBuffer(indirectCommandBufferDesc, AAPLNumObjects,mtlpp::ResourceOptions::CpuCacheModeDefaultCache);

  for(int objIndex = 0;objIndex < AAPLNumObjects; objIndex++){
    mtlpp::IndirectRenderCommand ICBCommand = g_indirectCommandBuffer.IndirectRenderCommandAt((uint32_t)objIndex);

    ICBCommand.SetVertexBuffer(g_vertexBufferArr[objIndex],0,AAPLVertexBufferIndexVertices);
    ICBCommand.SetVertexBuffer(g_indirectFrameStateBuffer, 0 ,AAPLVertexBufferIndexFrameState);
    ICBCommand.SetVertexBuffer(g_objectParameters, 0, AAPLVertexBufferIndexObjectParams);

    uint32_t vertexCount = g_vertexBufferArr[objIndex].GetLength()/sizeof(AAPLVertex);// may have problem?

    ICBCommand.DrawPrimitives(mtlpp::PrimitiveType::Triangle, 0, vertexCount, 1, objIndex);
  }

  Window win(g_device, &Render, 800, 600);
  Window::Run();

  return 0;
}
