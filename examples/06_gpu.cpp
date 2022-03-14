#include "../mtlpp.hpp"
#include "window.hpp"
#include <math.h>
#include <simd/simd.h>

static const uint32_t AAPLMaxFramesInFlight = 3;

mtlpp::Device g_device;
mtlpp::CommandQueue g_commandQueue;
mtlpp::Buffer g_vertexBuffer;
//mtlpp::Buffer g_vertexBufferArr[AAPLNumObjects];
mtlpp::Buffer g_objectParameters;
mtlpp::Buffer g_frameStateBuffer[AAPLMaxFramesInFlight];
mtlpp::Buffer g_indirectFrameStateBuffer;
mtlpp::RenderPipelineState g_renderPipelineState;
mtlpp::IndirectCommandBuffer g_indirectCommandBuffer;
vector_float2 g_aspectScale;

mtlpp::ComputePipelineState g_computePipelineState;
mtlpp::Buffer g_icbArgumentBuffer;
mtlpp::ArgumentEncoder g_argumentEncoder;

#include <simd/simd.h>

/////////////////////////////////////////////////////////
#pragma mark - Constants shared between shader and C code
/////////////////////////////////////////////////////////

// Number of unique meshes/objects in the scene
#define AAPLNumObjects    65536

// The number of objects in a row
#define AAPLGridWidth     256

// The number of object in a column
#define AAPLGridHeight    ((AAPLNumObjects+AAPLGridWidth-1)/AAPLGridWidth)

// Scale of each object when drawn
#define AAPLViewScale    0.25

// Because the objects are centered at origin, the scale appliced
#define AAPLObjectSize    2.0

// Distance between each object
#define AAPLObjecDistance 2.1


/////////////////////////////////////////////////////
#pragma mark - Types shared between shader and C code
/////////////////////////////////////////////////////

// Structure defining the layout of each vertex.  Shared between C code filling in the vertex data
// and Metal vertex shader consuming the vertices
typedef struct
{
  packed_float2 position;
  packed_float2 texcoord;
} AAPLVertex;

// Structure defining the layout of variable changing once (or less) per frame
typedef struct AAPLFrameState
{
  vector_float2 translation;
  vector_float2 aspectScale;
} AAPLFrameState;

// Structure defining parameters for each rendered object
typedef struct AAPLObjectParameters
{
  packed_float2 position;
  float boundingRadius;
  uint32_t numVertices;
  uint32_t startVertex;
} AAPLObjectParameters;

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
  AAPLKernelBufferIndexVertices,
  AAPLKernelBufferIndexCommandBufferContainer
} AAPLKernelBufferIndex;

// Argument buffer ID for the ICB encoded by the compute kernel
typedef enum AAPLArgumentBufferBufferID
{
  AAPLArgumentBufferIDCommandBuffer,
} AAPLArgumentBufferBufferID;

typedef struct AAPLObjectMesh {
  AAPLVertex *vertices;
  uint32_t numVerts;
} AAPLObjectMesh;


int g_inFlightIndex = 0;

void Render(const Window &win) {
  mtlpp::CommandBuffer commandBuffer = g_commandQueue.CommandBuffer();

  mtlpp::BlitCommandEncoder resetBlitCommandEncoder = commandBuffer.BlitCommandEncoder();
  resetBlitCommandEncoder.ResetCommandsInBuffer(g_indirectCommandBuffer, ns::Range(0,AAPLNumObjects));
  resetBlitCommandEncoder.EndEncoding();

  mtlpp::ComputeCommandEncoder computeCommandEncoder = commandBuffer.ComputeCommandEncoder();
  computeCommandEncoder.SetComputePipelineState(g_computePipelineState);
  computeCommandEncoder.SetBuffer(g_frameStateBuffer[g_inFlightIndex], 0, AAPLKernelBufferIndex::AAPLKernelBufferIndexFrameState);
  computeCommandEncoder.SetBuffer(g_objectParameters, 0, AAPLKernelBufferIndex::AAPLKernelBufferIndexObjectParams);
  computeCommandEncoder.SetBuffer(g_vertexBuffer,0,AAPLKernelBufferIndex::AAPLKernelBufferIndexVertices);
  computeCommandEncoder.SetBuffer(g_icbArgumentBuffer,0,AAPLKernelBufferIndex::AAPLKernelBufferIndexCommandBufferContainer);

  computeCommandEncoder.UseResource(g_indirectCommandBuffer, mtlpp::ResourceUsage::Write);

  int threadExecutionWidth = g_computePipelineState.GetThreadExecutionWidth();

  computeCommandEncoder.DispatchThreads(ns::Size(AAPLNumObjects,1,1),ns::Size(threadExecutionWidth,1,1));
  computeCommandEncoder.EndEncoding();

  mtlpp::BlitCommandEncoder optimizeBlitEncoder = commandBuffer.BlitCommandEncoder();
  optimizeBlitEncoder.OptimizeIndirectCommandBuffer(g_indirectCommandBuffer, ns::Range(0, AAPLNumObjects));
  optimizeBlitEncoder.EndEncoding();

  mtlpp::RenderPassDescriptor renderPassDesc = win.GetRenderPassDescriptor();
  if (renderPassDesc) {
    mtlpp::RenderCommandEncoder renderCommandEncoder =
        commandBuffer.RenderCommandEncoder(renderPassDesc);

    renderCommandEncoder.SetCullMode(mtlpp::CullMode::Back);
    renderCommandEncoder.SetRenderPipelineState(g_renderPipelineState);

    renderCommandEncoder.UseResource(g_vertexBuffer, mtlpp::ResourceUsage::Read);

    renderCommandEncoder.UseResource(g_objectParameters,mtlpp::ResourceUsage::Read);

    renderCommandEncoder.UseResource(g_indirectFrameStateBuffer,mtlpp::ResourceUsage::Read);

    renderCommandEncoder.ExecuteCommandsInBuffer(g_indirectCommandBuffer, ns::Range(0, AAPLNumObjects));

    renderCommandEncoder.EndEncoding();

    commandBuffer.Present(win.GetDrawable());
  }

  commandBuffer.Commit();
  //commandBuffer.WaitUntilCompleted();
}

AAPLObjectMesh NewGearMeshWithNumTeeth(uint32_t numTeeth,float innerRatio,float toothWidth,float toothSlope)
{
  AAPLObjectMesh mesh;

  // For each tooth, this function generates 2 triangles for tooth itself, 1 triangle to fill
  // the inner portion of the gear from bottom of the tooth to the center of the gear,
  // and 1 triangle to fill the inner portion of the gear below the groove beside the tooth.
  // Hence, the buffer needs 4 triangles or 12 vertices for each tooth.
  uint32_t numVertices = numTeeth * 12;
  uint32_t bufferSize = sizeof(AAPLVertex) * numVertices;

  mesh.numVerts = numVertices;
  mesh.vertices = (AAPLVertex*)malloc(bufferSize);

  const double angle = 2.0*M_PI/(double)numTeeth;
  static const packed_float2 origin = (packed_float2){0.0, 0.0};
  uint32_t vtx = 0;

  // Build triangles for teeth of gear
  for(int tooth = 0; tooth < numTeeth; tooth++)
  {
    // Calculate angles for tooth and groove
    const float toothStartAngle = tooth * angle;
    const float toothTip1Angle  = (tooth+toothSlope) * angle;
    const float toothTip2Angle  = (tooth+toothSlope+toothWidth) * angle;;
    const float toothEndAngle   = (tooth+2*toothSlope+toothWidth) * angle;
    const float nextToothAngle  = (tooth+1.0) * angle;

    // Calculate positions of vertices needed for the tooth
    const packed_float2 groove1    = { sin(toothStartAngle)*innerRatio, cos(toothStartAngle)*innerRatio };
    const packed_float2 tip1       = { sin(toothTip1Angle), cos(toothTip1Angle) };
    const packed_float2 tip2       = { sin(toothTip2Angle), cos(toothTip2Angle) };
    const packed_float2 groove2    = { sin(toothEndAngle)*innerRatio, cos(toothEndAngle)*innerRatio };
    const packed_float2 nextGroove = { sin(nextToothAngle)*innerRatio, cos(nextToothAngle)*innerRatio };

    // Right top triangle of tooth
    mesh.vertices[vtx].position = groove1;
    mesh.vertices[vtx].texcoord = (groove1 + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = tip1;
    mesh.vertices[vtx].texcoord = (tip1 + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = tip2;
    mesh.vertices[vtx].texcoord = (tip2 + 1.0) / 2.0;
    vtx++;

    // Left bottom triangle of tooth
    mesh.vertices[vtx].position = groove1;
    mesh.vertices[vtx].texcoord = (groove1 + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = tip2;
    mesh.vertices[vtx].texcoord = (tip2 + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = groove2;
    mesh.vertices[vtx].texcoord = (groove2 + 1.0) / 2.0;
    vtx++;

    // Slice of circle from bottom of tooth to center of gear
    mesh.vertices[vtx].position = origin;
    mesh.vertices[vtx].texcoord = (origin + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = groove1;
    mesh.vertices[vtx].texcoord = (groove1 + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = groove2;
    mesh.vertices[vtx].texcoord = (groove2 + 1.0) / 2.0;
    vtx++;

    // Slice of circle from the groove to the center of gear
    mesh.vertices[vtx].position = origin;
    mesh.vertices[vtx].texcoord = (origin + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = groove2;
    mesh.vertices[vtx].texcoord = (groove2 + 1.0) / 2.0;
    vtx++;

    mesh.vertices[vtx].position = nextGroove;
    mesh.vertices[vtx].texcoord = (nextGroove + 1.0) / 2.0;
    vtx++;
  }

  return mesh;
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

// Include header shared between this Metal shader code and C code executing Metal API commands

#include <simd/simd.h>

/////////////////////////////////////////////////////////
#pragma mark - Constants shared between shader and C code
/////////////////////////////////////////////////////////

// Number of unique meshes/objects in the scene
#define AAPLNumObjects    65536

// The number of objects in a row
#define AAPLGridWidth     256

// The number of object in a column
#define AAPLGridHeight    ((AAPLNumObjects+AAPLGridWidth-1)/AAPLGridWidth)

// Scale of each object when drawn
#define AAPLViewScale    0.25

// Because the objects are centered at origin, the scale appliced
#define AAPLObjectSize    2.0

// Distance between each object
#define AAPLObjecDistance 2.1


/////////////////////////////////////////////////////
#pragma mark - Types shared between shader and C code
/////////////////////////////////////////////////////

// Structure defining the layout of each vertex.  Shared between C code filling in the vertex data
// and Metal vertex shader consuming the vertices
typedef struct
{
    packed_float2 position;
    packed_float2 texcoord;
} AAPLVertex;

// Structure defining the layout of variable changing once (or less) per frame
typedef struct AAPLFrameState
{
    vector_float2 translation;
    vector_float2 aspectScale;
} AAPLFrameState;

// Structure defining parameters for each rendered object
typedef struct AAPLObjectPerameters
{
    packed_float2 position;
    float boundingRadius;
    uint32_t numVertices;
    uint32_t startVertex;
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
    AAPLKernelBufferIndexVertices,
    AAPLKernelBufferIndexCommandBufferContainer
} AAPLKernelBufferIndex;

// Argument buffer ID for the ICB encoded by the compute kernel
typedef enum AAPLArgumentBufferBufferID
{
    AAPLArgumentBufferIDCommandBuffer,
} AAPLArgumentBufferBufferID;


// This is the argument buffer that contains the ICB.
struct ICBContainer
{
    command_buffer commandBuffer [[ id(AAPLArgumentBufferIDCommandBuffer) ]];
};

// Check whether the object at 'objectIndex' is visible and set draw parameters if so.
// Otherwise, reset the command so that nothing is done.
kernel void
cullMeshesAndEncodeCommands(uint                         objectIndex   [[ thread_position_in_grid ]],
                            constant AAPLFrameState     *frame_state   [[ buffer(AAPLKernelBufferIndexFrameState) ]],
                            device AAPLObjectPerameters *object_params [[ buffer(AAPLKernelBufferIndexObjectParams)]],
                            device AAPLVertex           *vertices      [[ buffer(AAPLKernelBufferIndexVertices) ]],
                            device ICBContainer         *icb_container [[ buffer(AAPLKernelBufferIndexCommandBufferContainer) ]])
{
    float2 worldObjectPostion  = frame_state->translation + object_params[objectIndex].position;
    float2 clipObjectPosition  = frame_state->aspectScale * AAPLViewScale * worldObjectPostion;

    const float rightBounds =  1.0;
    const float leftBounds  = -1.0;
    const float upperBounds =  1.0;
    const float lowerBounds = -1.0;

    bool visible = true;

    // Set the bounding radius in the view space.
    const float2 boundingRadius = frame_state->aspectScale * AAPLViewScale * object_params[objectIndex].boundingRadius;

    // Check if the object's bounding circle has moved outside of the view bounds.
    if(clipObjectPosition.x + boundingRadius.x < leftBounds  ||
       clipObjectPosition.x - boundingRadius.x > rightBounds ||
       clipObjectPosition.y + boundingRadius.y < lowerBounds ||
       clipObjectPosition.y - boundingRadius.y > upperBounds)
    {
        visible = false;
    }
    // Get indirect render commnd object from the indirect command buffer given the object's unique
    // index to set parameters for drawing (or not drawing) the object.
    render_command cmd(icb_container->commandBuffer, objectIndex);

    if(visible)
    {
        // Set the buffers and add a draw command.
        cmd.set_vertex_buffer(frame_state, AAPLVertexBufferIndexFrameState);
        cmd.set_vertex_buffer(object_params, AAPLVertexBufferIndexObjectParams);
        cmd.set_vertex_buffer(vertices, AAPLVertexBufferIndexVertices);

        cmd.draw_primitives(primitive_type::triangle,
                            object_params[objectIndex].startVertex,
                            object_params[objectIndex].numVertices, 1,
                            objectIndex);
    }

    // If the object is not visible, no draw command will be set since so long as the app has reset
    // the indirect command buffer commands with a blit encoder before encoding the draw.
}

// Vertex shader outputs and per-fragment inputs.
struct RasterizerData
{
    float4 position [[position]];
    float2 tex_coord;
};

vertex RasterizerData
vertexShader(uint                     vertexID                [[ vertex_id ]],
             uint                     objectIndex             [[ instance_id ]],
             const device AAPLVertex* vertices                [[ buffer(AAPLVertexBufferIndexVertices) ]],
             const device AAPLObjectPerameters *object_params [[ buffer(AAPLVertexBufferIndexObjectParams) ]],
             constant AAPLFrameState* frame_state             [[ buffer(AAPLVertexBufferIndexFrameState) ]])
{
    RasterizerData out;

    float2 worldObjectPostion  = frame_state->translation + object_params[objectIndex].position;
    float2 modelVertexPosition = vertices[vertexID].position;
    float2 worldVertexPosition = modelVertexPosition + worldObjectPostion;
    float2 clipVertexPosition  = frame_state->aspectScale * AAPLViewScale * worldVertexPosition;

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

mtlpp::Function GPUCommandEncodingKernel = library.NewFunction("cullMeshesAndEncodeCommands");
g_computePipelineState = g_device.NewComputePipelineState(GPUCommandEncodingKernel, nullptr);

AAPLObjectMesh* tempMeshes;
{
tempMeshes = (AAPLObjectMesh*)malloc(sizeof(AAPLObjectMesh) * AAPLNumObjects);

for(int objectIdx = 0; objectIdx < AAPLNumObjects; objectIdx++)
{
// Choose the parameters to generate a mesh so that each one is unique.
uint32_t numTeeth = random() % 50 + 3;
float innerRatio = 0.2 + (random() / (1.0 * RAND_MAX)) * 0.7;
float toothWidth = 0.1 + (random() / (1.0 * RAND_MAX)) * 0.4;
float toothSlope = (random() / (1.0 * RAND_MAX)) * 0.2;

// Create a vertex buffer and initialize it with a unique 2D gear mesh.
tempMeshes[objectIdx] = NewGearMeshWithNumTeeth(numTeeth, innerRatio, toothWidth, toothSlope);
}
}

{
int objectParameterArraySize = AAPLNumObjects * sizeof(AAPLObjectParameters);
g_objectParameters = g_device.NewBuffer(objectParameterArraySize, mtlpp::ResourceOptions::CpuCacheModeDefaultCache);
}

{
size_t bufferSize = 0;

for(int objectIdx = 0; objectIdx < AAPLNumObjects; objectIdx++)
{
size_t meshSize = sizeof(AAPLVertex) * tempMeshes[objectIdx].numVerts;
bufferSize += meshSize;
}

g_vertexBuffer = g_device.NewBuffer(bufferSize, mtlpp::ResourceOptions::CpuCacheModeDefaultCache);
}

{
uint32_t currentStartVertex = 0;

AAPLObjectParameters *params = (AAPLObjectParameters*)g_objectParameters.GetContents();

for(int objectIdx = 0; objectIdx < AAPLNumObjects; objectIdx++)
{
// Store the mesh metadata in the `params` buffer.

params[objectIdx].numVertices = tempMeshes[objectIdx].numVerts;

size_t meshSize = sizeof(AAPLVertex) * tempMeshes[objectIdx].numVerts;

params[objectIdx].startVertex = currentStartVertex;

// Pack the current mesh data in the combined vertex buffer.

AAPLVertex* meshStartAddress = ((AAPLVertex*)g_vertexBuffer.GetContents()) + currentStartVertex;

memcpy(meshStartAddress, tempMeshes[objectIdx].vertices, meshSize);

currentStartVertex += tempMeshes[objectIdx].numVerts;

free(tempMeshes[objectIdx].vertices);

// Set the other culling and mesh rendering parameters.

// Set the position of each object to a unique space in a grid.
vector_float2 gridPos = (vector_float2){float(objectIdx % AAPLGridWidth), float(objectIdx / AAPLGridWidth)};
params[objectIdx].position = gridPos * AAPLObjecDistance;

params[objectIdx].boundingRadius = AAPLObjectSize / 2.0;
}
}

free(tempMeshes);


for (int i = 0; i < AAPLMaxFramesInFlight; i++) {
  g_frameStateBuffer[i] = g_device.NewBuffer(
      sizeof(AAPLFrameState), mtlpp::ResourceOptions::StorageModeShared);

  AAPLFrameState* frameState = (AAPLFrameState*)g_frameStateBuffer[i].GetContents();
  g_aspectScale.x = 800.0 / 600.0;
  g_aspectScale.y = 1.0;

  frameState->aspectScale = g_aspectScale;

  const vector_float2 translation = {0.0, 0.0};

  frameState->translation = translation;
}

mtlpp::IndirectCommandBufferDescriptor indirectCommandBufferDesc;
indirectCommandBufferDesc.SetCommandTypes(
mtlpp::IndirectCommandType::IndirectCommandTypeDraw);
indirectCommandBufferDesc.SetInheritBuffers(false);
indirectCommandBufferDesc.SetMaxVertexBufferBindCount(3);
indirectCommandBufferDesc.SetMaxFragmentBufferBindCount(0);
indirectCommandBufferDesc.SetInheritPipelineState(true);

g_indirectCommandBuffer = g_device.NewIndirectCommandBuffer(indirectCommandBufferDesc, AAPLNumObjects,mtlpp::ResourceOptions::CpuCacheModeDefaultCache);

g_argumentEncoder = GPUCommandEncodingKernel.NewArgumentEncoderWithBufferIndex(AAPLKernelBufferIndex::AAPLKernelBufferIndexCommandBufferContainer);

g_icbArgumentBuffer = g_device.NewBuffer(g_argumentEncoder.GetEncodedLength(), mtlpp::ResourceOptions::StorageModeShared);

g_icbArgumentBuffer.SetLabel("ICB Argument Buffer");

g_argumentEncoder.SetArgumentBuffer(g_icbArgumentBuffer,0);

g_argumentEncoder.SetIndirectCommandBuffer(g_indirectCommandBuffer,AAPLArgumentBufferIDCommandBuffer);

Window win(g_device, &Render, 800, 600);
Window::Run();

return 0;
}
