#include "ISRReconstructionEngine_GPU.h"
#include "ISRReconstructionEngine_DA.h"

#include "../../Utils/ISRCUDAUtils.h"

#include "../../../LibISRUtils/IOUtil.h"
#include "../../../ORUtils/CUDADefines.h"

using namespace LibISR;
using namespace LibISR::Engine;

__global__ void evolve3DShape_device(float* tmpsdfptr, float* sdfptr, float* pinptr, Vector3i voldim);


void LibISR::Engine::ISRReconstructionEngine_GPU::evolve3DShape(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRVol_ptr inpinvol, int iterno)
{
	float* pinptr = inpinvol->getVoxelData();
	float* sdfptr = inshape->getSDFVoxel();
	float* tmpsdfptr = tmpshape->getSDFVoxel();

	Vector3i voldim = inshape->volSize;

	dim3 blockSize(8, 8, 8);
	dim3 gridSize((int)ceil((float)voldim.x / (float)blockSize.x), (int)ceil((float)voldim.y / (float)blockSize.y), (int)ceil((float)voldim.z / (float)blockSize.z));

	for (int iter = 0; iter < iterno; iter++)
	{
		ORcudaSafeCall(cudaMemcpy(tmpsdfptr, sdfptr, inshape->allocatedSize*sizeof(float), cudaMemcpyDeviceToDevice));
		evolve3DShape_device << <gridSize, blockSize >> >(tmpsdfptr, sdfptr, pinptr, voldim);
		ORcudaSafeCall(cudaMemcpy(sdfptr, tmpsdfptr, inshape->allocatedSize*sizeof(float), cudaMemcpyDeviceToDevice));
	}
}

void LibISR::Engine::ISRReconstructionEngine_GPU::reinitializeSDF(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRShape_ptr tmpshape2)
{

}


//////////////////////////////////////////////////////////////////////////
// device functions
//////////////////////////////////////////////////////////////////////////

__global__ void evolve3DShape_device(float* tmpsdfptr, float* sdfptr, float* pinptr, Vector3i voldim)
{
	int i = blockIdx.x * 8 + threadIdx.x;
	int j = blockIdx.y * 8 + threadIdx.y;
	int k = blockIdx.z * 8 + threadIdx.z;

	if (i<3 || j <3 || k<3 || i>voldim.x - 3 || j>voldim.y - 3 || k>voldim.z - 3) return;

	evolveVoxelOneStep(tmpsdfptr, sdfptr, pinptr, i, j, k);
}
