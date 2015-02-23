#include "ISRReconstructionEngine_CPU.h"
#include "ISRReconstructionEngine_DA.h"


using namespace LibISR;
using namespace LibISR::Engine;


void LibISR::Engine::ISRReconstructionEngine_CPU::evolve3DShape(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRVol_ptr inpinvol, int iterno)
{
	float* pinptr = inpinvol->getVoxelData();
	float* sdfptr = inshape->getSDFVoxel();
	float* tmpsdfptr = tmpshape->getSDFVoxel();
	

	Vector3i voldim = inshape->volSize;

	float DTDIV = 0.25f;

	bool dtfound;

	for (int iter = 0; iter < iterno; iter++)
	{
		int j = 2, k = 2;
		
		memcpy(tmpsdfptr, sdfptr, inshape->allocatedSize*sizeof(float));
		#pragma omp parallel
		{
			#pragma  omp for private(j,k)
			for (int i = 2; i <= voldim.x - 3; i++) for (j = 2; j <= voldim.y - 3; j++) for (k = 2; k <= voldim.z - 3; k++)
			{
				evolveVoxelOneStep(tmpsdfptr, sdfptr, pinptr, i, j, k);
			}
		}
		memcpy(sdfptr, tmpsdfptr, inshape->allocatedSize*sizeof(float));
	}
}
