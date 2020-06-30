//-
// ==========================================================================
// Copyright 2015 Autodesk, Inc.  All rights reserved.
//
// Use of this software is subject to the terms of the Autodesk
// license agreement provided at the time of installation or download,
// or which otherwise accompanies this software in either electronic
// or hard copy form.
// ==========================================================================
//+

////////////////////////////////////////////////////////////////////////
// 
// DESCRIPTION:
//
// Produces the dependency graph node "offsetNode".
//
// This plug-in demonstrates how to create a user-defined weighted deformer
// with an associated shape. A deformer is a node which takes any number of
// input geometries, deforms them, and places the output into the output
// geometry attribute. This example plug-in defines a new deformer node
// that offsets vertices according to their CV's weights. The weights are set
// using the set editor or the percent command.
//
// To use this node: 
//	- create a plane or some other object
//	- type: "deformer -type offset" 
//	- a locator is created by the command, and you can use this locator
//	  to control the direction of the offset. The object's CV's will be offset
//	  by the value of the weights of the CV's (the default will be the weight * some constant)
//	  in the direction of the y-vector of the locator 
//	- you can edit the weights using either the component editor or by using
//	  the percent command (eg. percent -v .5 offset1;) 
//
// Use this script to create a simple example with the offset node:
// 
//	loadPlugin offsetNode;
//	polyTorus -r 1 -sr 0.5 -tw 0 -sx 50 -sy 50 -ax 0 1 0 -cuv 1 -ch 1;
//	deformer -type "offset";
//	setKeyframe -v 0 -at rotateZ -t 1 transform1;
//	setKeyframe -v 180 -at rotateZ -t 60 transform1;
//	select -cl;
//
////////////////////////////////////////////////////////////////////////

#include <memory>
#include <cl_common.h>
#include <string.h>
#include <maya/MIOStream.h>
#include <maya/MStringArray.h>
#include <math.h>

#include <maya/MGlobal.h>

#include <maya/MPxDeformerNode.h>
#include <maya/MItGeometry.h>
#include <maya/MPxLocatorNode.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnPlugin.h>
#include <maya/MFnDependencyNode.h>

#include <maya/MTypeId.h>
#include <maya/MPlug.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>

#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>

#include <maya/MDagModifier.h>

#include <maya/MPxGPUDeformer.h>
#include <maya/MGPUDeformerRegistry.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MViewport2Renderer.h>
#include <maya/MFnMesh.h>
#include <maya/MString.h>

#include <vector>

#include "mayaApiUtils/miostream.h"
#include <boost/compute/program.hpp>

namespace bcompute = boost::compute;

using BoostCommandQueue_T = bcompute::command_queue;
using VexContext_T = vex::Context;
using BoostCommandQueueUPtr_T = std::unique_ptr<BoostCommandQueue_T>;
using VexCtontextUPtr_T = std::unique_ptr<VexContext_T>;

class offset : public MPxDeformerNode
{
public:
  offset();
  ~offset() override;

  static void* creator();
  static MStatus initialize();

  // deformation function
  //
  MStatus deform(
    MDataBlock& block,
    MItGeometry& iter,
    const MMatrix& mat,
    unsigned int multiIndex
  ) override;

  // when the accessory is deleted, this node will clean itself up
  //
  MObject& accessoryAttribute() const override;

  // create accessory nodes when the node is created
  //
  MStatus accessoryNodeSetup(MDagModifier& cmd) override;

public:
  // local node attributes

  static MObject offsetMatrix; // offset center and axis

  static MTypeId id;

private:
};

MTypeId offset::id(0x8000c);

// local attributes
//
MObject offset::offsetMatrix;


offset::offset() {}
offset::~offset() {}

void* offset::creator()
{
  return new offset();
}

MStatus offset::initialize()
{
  // local attribute initialization

  MFnMatrixAttribute mAttr;
  offsetMatrix = mAttr.create("locateMatrix", "lm");
  mAttr.setStorable(false);
  mAttr.setConnectable(true);

  //  deformation attributes
  addAttribute(offsetMatrix);

  attributeAffects(offset::offsetMatrix, offset::outputGeom);

  return MStatus::kSuccess;
}


MStatus
offset::deform(
  MDataBlock& block,
  MItGeometry& iter,
  const MMatrix& /*m*/,
  unsigned int multiIndex
)
//
// Method: deform
//
// Description:   Deform the point with a squash algorithm
//
// Arguments:
//   block		: the datablock of the node
//	 iter		: an iterator for the geometry to be deformed
//   m    		: matrix to transform the point into world space
//	 multiIndex : the index of the geometry that we are deforming
//
//
{
  MStatus returnStatus;

  // Envelope data from the base class.
  // The envelope is simply a scale factor.
  //
  MDataHandle envData = block.inputValue(envelope, &returnStatus);
  if (MS::kSuccess != returnStatus) return returnStatus;
  float env = envData.asFloat();

  // Get the matrix which is used to define the direction and scale
  // of the offset.
  //
  MDataHandle matData = block.inputValue(offsetMatrix, &returnStatus);
  if (MS::kSuccess != returnStatus) return returnStatus;
  MMatrix omat = matData.asMatrix();
  MMatrix omatinv = omat.inverse();

  // iterate through each point in the geometry
  //
  for (; !iter.isDone(); iter.next()) {
    MPoint pt = iter.position();
    pt *= omatinv;

    float weight = weightValue(block, multiIndex, iter.index());

    // offset algorithm
    //
    pt.y = pt.y + env * weight;
    //
    // end of offset algorithm

    pt *= omat;
    iter.setPosition(pt);
  }
  return returnStatus;
}


/* override */
MObject&
offset::accessoryAttribute() const
//
//	Description:
//	  This method returns a the attribute to which an accessory	
//    shape is connected. If the accessory shape is deleted, the deformer
//	  node will automatically be deleted.
//
//    This method is optional.
//
{
  return offset::offsetMatrix;
}

/* override */
MStatus
offset::accessoryNodeSetup(MDagModifier& cmd)
//
//	Description:
//		This method is called when the deformer is created by the
//		"deformer" command. You can add to the cmds in the MDagModifier
//		cmd in order to hook up any additional nodes that your node needs
//		to operate.
//
//		In this example, we create a locator and attach its matrix attribute
//		to the matrix input on the offset node. The locator is used to
//		set the direction and scale of the random field.
//
//	Description:
//		This method is optional.
//
{
  MStatus result;

  // hook up the accessory node
  //
  MObject objLoc = cmd.createNode(
    MString("locator"),
    MObject::kNullObj,
    &result
  );

  if (MS::kSuccess == result) {
    MFnDependencyNode fnLoc(objLoc);
    MString attrName;
    attrName.set("matrix");
    MObject attrMat = fnLoc.attribute(attrName);

    result = cmd.connect(objLoc, attrMat, this->thisMObject(), offset::offsetMatrix);
  }
  return result;
}

// the GPU override implementation of the offsetNode
// 

class offsetGPUDeformer : public MPxGPUDeformer
{
public:
  // Virtual methods from MPxGPUDeformer
  offsetGPUDeformer();
  ~offsetGPUDeformer() override;

  inline static MString pluginLoadPath;

  MPxGPUDeformer::DeformerStatus evaluate(
    MDataBlock& block,
    const MEvaluationNode&,
    const MPlug& plug,
    unsigned int numElements,
    const MAutoCLMem,
    const MAutoCLEvent,
    MAutoCLMem,
    MAutoCLEvent&
  ) override;
  void terminate() override;

  static MGPUDeformerRegistrationInfo* getGPUDeformerInfo();
  static bool validateNodeInGraph(
    MDataBlock& block,
    const MEvaluationNode&,
    const MPlug& plug,
    MStringArray* messages
  );
  static bool validateNodeValues(
    MDataBlock& block,
    const MEvaluationNode&,
    const MPlug& plug,
    MStringArray* messages
  );

private:
  // helper methods
  void extractWeightArray(
    MDataBlock& block,
    const MEvaluationNode& evaluationNode,
    const MPlug& plug
  );
  void extractOffsetMatrix(
    MDataBlock& block,
    const MEvaluationNode& evaluationNode,
    const MPlug& plug
  );

  // Storage for data on the GPU
  MAutoCLMem fCLWeights;
  MAutoCLMem fOffsetMatrix;
  unsigned int fNumElements;

  MAutoCLMem fFinalPosition;
  // Kernel
  MAutoCLKernel fKernel;

  #ifdef USE_VEXCL
  // std::unique_ptr<bcompute::command_queue> boost_command_queue_p;
  // BoostCommandQueueUPtr_T boost_command_queue_p;
  VexCtontextUPtr_T vexCtx_p;

  #endif
};

class offsetNodeGPUDeformerInfo : public MGPUDeformerRegistrationInfo
{
public:
  offsetNodeGPUDeformerInfo() {}
  ~offsetNodeGPUDeformerInfo() override {}

  MPxGPUDeformer* createGPUDeformer() override
  {
    return new offsetGPUDeformer();
  }

  bool validateNodeInGraph(
    MDataBlock& block,
    const MEvaluationNode& evaluationNode,
    const MPlug& plug,
    MStringArray* messages
  ) override
  {
    return offsetGPUDeformer::validateNodeInGraph(block, evaluationNode, plug, messages);
  }

  bool validateNodeValues(
    MDataBlock& block,
    const MEvaluationNode& evaluationNode,
    const MPlug& plug,
    MStringArray* messages
  ) override
  {
    return offsetGPUDeformer::validateNodeValues(block, evaluationNode, plug, messages);
  }
};

MGPUDeformerRegistrationInfo* offsetGPUDeformer::getGPUDeformerInfo()
{
  static offsetNodeGPUDeformerInfo theOne;
  return &theOne;
}

offsetGPUDeformer::offsetGPUDeformer()
{
  // Remember the ctor must be fast.  No heavy work should be done here.
  // Maya may allocate one of these and then never use it.
}

offsetGPUDeformer::~offsetGPUDeformer()
{
  terminate();
}

/* static */
bool offsetGPUDeformer::validateNodeInGraph(
  MDataBlock& block,
  const MEvaluationNode& evaluationNode,
  const MPlug& plug,
  MStringArray* messages
)
{
  // offsetGPUDeformer supports everything on the offset node except envelope
  // envelope is handled in validateNodeValues because we support some values
  // but not others.
  return true;
}

/* static */
bool offsetGPUDeformer::validateNodeValues(
  MDataBlock& block,
  const MEvaluationNode& evaluationNode,
  const MPlug& plug,
  MStringArray* messages
)
{
  // As an example offsetGPUDeformer has conditional support for the envelop
  // attribute.  offsetGPUDeformer supports the underlying depend node if
  // envelope is exactly 1.0f.  Otherwise, the underlying node is not supported.
  // Note that in additional to testing the value of the envelope attribute here
  // we have also registered the envelope attribute as a conditional attribute in
  // offsetNodeGPUDeformerInfo.

  MObject node = plug.node();
  MFnDependencyNode fnNode(node);

  // Now that I know the envelope value is not changing, check to see if it is 1.0f
  MPlug envelopePlug(node, MPxDeformerNode::envelope);
  MDataHandle envData;
  envelopePlug.getValue(envData);
  if (envData.asFloat() != 1.0f) {
    MOpenCLInfo::appendMessage(
      messages,
      "Offset %s not supported by deformer evaluator because envelope is not exactly 1.0.",
      fnNode.name().asChar()
    );
    return false;
  }

  return true;
}

MPxGPUDeformer::DeformerStatus offsetGPUDeformer::evaluate(
  MDataBlock& block,
  // data block for "this" node
  const MEvaluationNode& evaluationNode,
  // evaluation node representing "this" node
  const MPlug& plug,
  // the multi index we're working on.  There will be a separate instance created per multi index
  unsigned int numElements,
  // the number of float3 elements in inputBuffer and outputBuffer
  const MAutoCLMem inputBuffer,
  // the input positions we are going to deform
  const MAutoCLEvent inputEvent,
  // the input event we need to wait for before we start reading the input positions
  MAutoCLMem outputBuffer,
  // the output positions we should write to.  This may or may not be the same buffer as inputBuffer.
  MAutoCLEvent& outputEvent
) // the event a downstream deformer will wait for before reading from output buffer
{
  // evaluate has two main pieces of work.  I need to transfer any data I care about onto the GPU, and I need to run my OpenCL Kernel.
  // First, transfer the data.  offset has two pieces of data I need to transfer to the GPU, the weight array and the offset matrix.
  // I don't need to transfer down the input position buffer, that is already handled by the deformer evaluator, the points are in inputBuffer.
  fNumElements = numElements;

  MObject node = plug.node();

  extractWeightArray(block, evaluationNode, plug);
  extractOffsetMatrix(block, evaluationNode, plug);

  #ifdef USE_VEXCL

  MINFO("vexcl branch")


  if (!vexCtx_p) {
    MINFO("Creating VexCL context")
    vexCtx_p = std::make_unique<VexContext_T>(
      std::vector<bcompute::context>{bcompute::context(MOpenCLInfo::getOpenCLContext())},
      std::vector<BoostCommandQueue_T>{
        BoostCommandQueue_T(MOpenCLInfo::getMayaDefaultOpenCLCommandQueue())
      }
    );
  }

  // vex::Context ctx(
  // 	std::vector<VexContext_T>{VexContext_T(MOpenCLInfo::getOpenCLContext())},
  // 	std::vector<BoostCommandQueue_T>{BoostCommandQueue_T(MOpenCLInfo::getMayaDefaultOpenCLCommandQueue())}
  // );

  auto& inputBufferVV = vex::vector(
    vexCtx_p->queue()[0],
    vex::device_vector<cl_mem>(
      boost::compute::buffer(inputBuffer.get())
    )
  ).reinterpret<cl_float>();


  auto& outputBufferVV = vex::vector(
    vexCtx_p->queue()[0],
    vex::device_vector<cl_mem>(
      boost::compute::buffer(outputBuffer.get())
    )
  ).reinterpret<cl_float>();


  auto& matricesVV = vex::vector(
    vexCtx_p->queue()[0],
    vex::device_vector<cl_mem>(
      boost::compute::buffer(fOffsetMatrix.get())
    )
  ).reinterpret<cl_float4>();

  auto& weightVV = vex::vector(
    vexCtx_p->queue()[0],
    vex::device_vector<cl_mem>(
      boost::compute::buffer(fCLWeights.get())
    )
  ).reinterpret<cl_float>();

  vex::vector<cl_float4> initialPositionVV(
    *vexCtx_p,
    fNumElements
  );

  vex::vector<cl_float4> finalPositionVV(
    *vexCtx_p,
    fNumElements
  );

  VEX_FUNCTION(
    void,
    phase1,
    (const int, idx)
    (cl_float4*, initialPos)
    (const float*, inputPos)
    (cl_float4*, finalPos),
    //

    const int offsetIdx = idx * 3;

    initialPos[idx] = (float4)(
      inputPos[offsetIdx + 0],
      inputPos[offsetIdx + 1],
      inputPos[offsetIdx + 2],
      1.0f
    );

    finalPos[idx] = (float4)(
      0.0f,
      0.0f,
      0.0f,
      1.0f
    );

  );


  try {
    vex::eval(
      phase1(
        vex::element_index(0, fNumElements),
        vex::raw_pointer(initialPositionVV),
        vex::raw_pointer(inputBufferVV),
        vex::raw_pointer(finalPositionVV)
      ),
      initialPositionVV.queue_list(),
      initialPositionVV.partition()
    );

    // MINFO(initialPositionVV)
  }
  catch (std::exception e) {
    MERR(e.what())
  }

  MINFO(matricesVV)

  // 1 matrix = 4 * float4, total length = 8
  // vex::slicer<1> slice(vex::extents[4 * 2]);

  // auto matrix = matricesVV[1];
  // auto matrixInverse = matricesVV[1];

  //first matrix is offset matrix, second matrix is offset matrix inverse
   VEX_FUNCTION(
     void, phase2,
     (const int, idx)
     (const cl_float4, initialPos)
     (cl_float4, finalPos)
     (const cl_float4*, matrices)
     (const cl_float, weight)
     (cl_float*, outputPos),

    // todo: global specifier
    finalPos.x = dot(initialPos, matrices[4+0]);
    finalPos.y = dot(initialPos, matrices[4+1]) + weight;
    finalPos.z = dot(initialPos, matrices[4+2]);
  
  	const int offsetIdx = idx * 3;
  	outputPos[offsetIdx + 0] = dot(finalPos, matrices[0]);
  	outputPos[offsetIdx + 1] = dot(finalPos, matrices[1]);
  	outputPos[offsetIdx + 2] = dot(finalPos, matrices[2]);
  );
  
  	MINFO(outputBufferVV[0])
  	MINFO(matricesVV[0])
  	MINFO(matricesVV[1])
  	MINFO(matricesVV[2])
  	MINFO(matricesVV[3])
  
  try {
     vex::eval(
       phase2(
  			vex::element_index(0, fNumElements),
         initialPositionVV, 
         finalPositionVV,
  			vex::raw_pointer(matricesVV),
  			weightVV,
  			vex::raw_pointer(outputBufferVV)
       )
     );
  
  
  } catch(std::exception e) {
    MERR(e.what())
  }

  #else


	// Now that all the data we care about is on the GPU, setup and run the OpenCL Kernel
	if (!fKernel.get())
	{
		char *maya_location = getenv( "MAYA_LOCATION" );


		MString openCLKernelFile(pluginLoadPath);
		openCLKernelFile +="/offset.cl";
		MString openCLKernelName("offset");
		fKernel = MOpenCLInfo::getOpenCLKernel(openCLKernelFile, openCLKernelName);

		if (!fKernel) return MPxGPUDeformer::kDeformerFailure;
	}

	cl_int err = CL_SUCCESS;
	
	// Set all of our kernel parameters.  Input buffer and output buffer may be changing every frame
	// so always set them.
	unsigned int parameterId = 0;
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)outputBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)inputBuffer.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)fCLWeights.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_mem), (void*)fOffsetMatrix.getReadOnlyRef());
	MOpenCLInfo::checkCLErrorStatus(err);
	err = clSetKernelArg(fKernel.get(), parameterId++, sizeof(cl_uint), &fNumElements);
	MOpenCLInfo::checkCLErrorStatus(err);

	// Figure out a good work group size for our kernel.
	size_t workGroupSize;
	size_t retSize;
	err = clGetKernelWorkGroupInfo(
		fKernel.get(),
		MOpenCLInfo::getOpenCLDeviceId(),
		CL_KERNEL_WORK_GROUP_SIZE,
		sizeof(size_t),
		&workGroupSize,
		&retSize);
	MOpenCLInfo::checkCLErrorStatus(err);

	size_t localWorkSize = 256;
	if (retSize > 0) localWorkSize = workGroupSize;
	size_t globalWorkSize = (localWorkSize - fNumElements % localWorkSize) + fNumElements; // global work size must be a multiple of localWorkSize

  // set up our input events.  The input event could be NULL, in that case we need to pass
	// slightly different parameters into clEnqueueNDRangeKernel
	unsigned int numInputEvents = 0;
	if (inputEvent.get())
	{
		numInputEvents = 1;
	}

	// run the kernel
	err = clEnqueueNDRangeKernel(
		MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
		fKernel.get(),
		1,
		NULL,
		&globalWorkSize,
		&localWorkSize,
		numInputEvents,
		numInputEvents ? inputEvent.getReadOnlyRef() : 0,
		outputEvent.getReferenceForAssignment() );
	MOpenCLInfo::checkCLErrorStatus(err);

  #endif

  return MPxGPUDeformer::kDeformerSuccess;
}

void offsetGPUDeformer::terminate()
{
  MHWRender::MRenderer::theRenderer()->releaseGPUMemory(fNumElements * sizeof(float));
  fCLWeights.reset();
  fOffsetMatrix.reset();
  fFinalPosition.reset();
  MOpenCLInfo::releaseOpenCLKernel(fKernel);
  fKernel.reset();
}

void offsetGPUDeformer::extractWeightArray(
  MDataBlock& block,
  const MEvaluationNode& evaluationNode,
  const MPlug& plug
)
{
  // if we've already got a weight array and it is not changing then don't bother copying it
  // to the GPU again

  MStatus status;
  // Note that right now dirtyPlugExists takes an attribute, so if any element in the multi is changing we think it is dirty...
  // To avoid false dirty issues here you'd need to only use one element of the MPxDeformerNode::input multi attribute for each
  // offset node.
  if ((fCLWeights.get() && !evaluationNode.dirtyPlugExists(MPxDeformerNode::weightList, &status)) ||
    !status) {
    return;
  }

  // Maya might do some tricky stuff like not store the weight array at all for certain weight
  // values so we can't count on an array existing in the weightList.  For the OpenCL Kernel
  // we want an array with one weight in it per vertex, we need to build it carefully here.
  std::vector<float> temp;
  temp.reserve(fNumElements);

  // Two possibilities: we could have a sparse array in weightList[multiIndex] or there could be nothing in weightList[multiIndex].
  // if nothing is there then all the weights at 1.0f.

  // Get a handle to the weight array we want.
  MArrayDataHandle weightList = block.outputArrayValue(MPxDeformerNode::weightList, &status);
  if (!status) return; // we should always be able to get a weightList
  status = weightList.jumpToElement(plug.logicalIndex());
  // it is possible that the jumpToElement fails.  In that case all weights are 1.
  if (!status) {
    for (unsigned int i = 0; i < fNumElements; i++)
      temp.push_back(1.0f);
  }
  else {
    MDataHandle weightsStructure = weightList.inputValue(&status);
    if (!status) return;
    MArrayDataHandle weights = weightsStructure.child(MPxDeformerNode::weights);
    if (!status) return;

    // number of non-zero weights
    unsigned int numWeights = weights.elementCount(&status);
    if (!status) return;

    // we're building a list with a weight per vertex, even if the weight is zero
    unsigned int weightIndex = 0;
    for (unsigned int i = 0; i < numWeights; i++, weights.next()) {
      unsigned int weightsElementIndex = weights.elementIndex(&status);
      while (weightIndex < weightsElementIndex) {
        temp.push_back(0.0f); // weights could be sparse, fill in zero weight if no data
        weightIndex++;
      }
      MDataHandle value = weights.inputValue(&status);
      temp.push_back(value.asFloat());
      weightIndex++;
    }
    // now we have written the last non-zero weight into temp, but the last non-zero weight
    // doesn't have to be for the last vertex in the buffer.  Add more zero values if necessary.
    while (weightIndex < fNumElements) {
      temp.push_back(0.0f); // weights could be sparse, fill in zero weight if no data
      weightIndex++;
    }
  }

  // Two possibilities, we could be updating an existing OpenCL buffer or allocating a new one.
  cl_int err = CL_SUCCESS;
  if (!fCLWeights.get()) {
    MHWRender::MRenderer::theRenderer()->holdGPUMemory(fNumElements * sizeof(float));
    fCLWeights.attach(
      clCreateBuffer(
        MOpenCLInfo::getOpenCLContext(),
        CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
        fNumElements * sizeof(float),
        (void*)&temp[0],
        &err
      )
    );
  }
  else {
    // I use a blocking write here, non-blocking could be faster...  need to manage the lifetime of temp, and have the kernel wait until the write finishes before running
    // I'm also assuming that the weight buffer is not growing.
    err = clEnqueueWriteBuffer(
      MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
      fCLWeights.get(),
      CL_TRUE,
      0,
      fNumElements * sizeof(float),
      (void*)&temp[0],
      0,
      NULL,
      NULL
    );
  }
}

void offsetGPUDeformer::extractOffsetMatrix(
  MDataBlock& block,
  const MEvaluationNode& evaluationNode,
  const MPlug& plug
)
{
  // I pass the offset matrix to OpenCL using a buffer as well.  I also send down the inverse matrix to avoid calculating it many times on the GPU
  MStatus status;
  if ((fOffsetMatrix.get() && !evaluationNode.dirtyPlugExists(offset::offsetMatrix, &status)) || !
    status) {
    return;
  }

  MDataHandle matData = block.inputValue(offset::offsetMatrix, &status);
  if (MS::kSuccess != status) return;
  MMatrix omat = matData.asMatrix();
  MMatrix omatinv = omat.inverse();

  // Convert the matrix from Maya format to the format the OpenCL kernel expects
  omat = omat.transpose();
  omatinv = omatinv.transpose();

  // MMatrix stores double values, but I want floating point values on the GPU so convert them here.
  unsigned int numFloat = 32;
  float* temp = new float[numFloat];
  unsigned int curr = 0;

  for (unsigned int row = 0; row < 4; row++) {
    for (unsigned int column = 0; column < 4; column++) {
      temp[curr++] = (float)omat(row, column);
    }
  }

  for (unsigned int row = 0; row < 4; row++) {
    for (unsigned int column = 0; column < 4; column++) {
      temp[curr++] = (float)omatinv(row, column);
    }
  }

  // Two possibilities, we could be updating an existing OpenCL buffer or allocating a new one.
  cl_int err = CL_SUCCESS;
  if (!fOffsetMatrix.get()) {
    fOffsetMatrix.attach(
      clCreateBuffer(
        MOpenCLInfo::getOpenCLContext(),
        CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY,
        numFloat * sizeof(float),
        (void*)temp,
        &err
      )
    );
  }
  else {
    // I use a blocking write here, non-blocking could be faster...  need to manage the lifetime of temp, and have the kernel wait until the write finishes before running
    err = clEnqueueWriteBuffer(
      MOpenCLInfo::getMayaDefaultOpenCLCommandQueue(),
      fOffsetMatrix.get(),
      CL_TRUE,
      0,
      numFloat * sizeof(float),
      (void*)temp,
      0,
      NULL,
      NULL
    );
  }

  delete [] temp;
}

// standard initialization procedures
//

MStatus initializePlugin(MObject obj)
{
  MStatus result;
  MFnPlugin plugin(obj, PLUGIN_COMPANY, "3.0", "Any");

  offsetGPUDeformer::pluginLoadPath = plugin.loadPath();

  result = plugin.registerNode(
    "offset",
    offset::id,
    offset::creator,
    offset::initialize,
    MPxNode::kDeformerNode
  );

  MString nodeClassName("offset");
  MString registrantId("mayaPluginExample");
  MGPUDeformerRegistry::registerGPUDeformerCreator(
    nodeClassName,
    registrantId,
    offsetGPUDeformer::getGPUDeformerInfo()
  );

  MGPUDeformerRegistry::addConditionalAttribute(
    nodeClassName,
    registrantId,
    MPxDeformerNode::envelope
  );

  return result;
}

MStatus uninitializePlugin(MObject obj)
{
  MStatus result;
  MFnPlugin plugin(obj);
  result = plugin.deregisterNode(offset::id);

  MString nodeClassName("offset");
  MString registrantId("mayaPluginExample");
  MGPUDeformerRegistry::deregisterGPUDeformerCreator(
    nodeClassName,
    registrantId
  );

  return result;
}
