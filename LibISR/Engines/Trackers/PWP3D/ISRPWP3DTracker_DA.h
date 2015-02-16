#pragma once

#include "../../../Utils/LibISRDefine.h"
#include "../../Lowlevel/ISRVoxelAccess_DA.h"

#include "../../../Objects/Basic/ISRPose.h"
#include "../../../Objects/Basic/ISRShape.h"

//_CPU_AND_GPU_CODE_ inline float computePerPixelEnergy(const Vector4f &inpt, LibISR::Objects::ISRShape_ptr shape, LibISR::Objects::ISRPose_ptr pose, int numObj)
//{
//	if (inpt.w > 0)
//	{
//		float dt = MAX_SDF, partdt = MAX_SDF;
//		int idx;
//		float *voxelBlocks;
//
//		for (int i = 0; i < numObj; i++)
//		{
//			Vector3f objpt = poses[i].getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
//			idx = pt2IntIdx(objpt);
//			if (idx >= 0)
//			{
//				voxelBlocks = shapes[i].getSDFVoxel();
//				partdt = voxelBlocks[idx];
//				dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
//			}
//		}
//
//		if (dt == MAX_SDF) return -1.0f;
//
//		float exp_dt = expf(-dt * DTUNE);
//		float deto = exp_dt + 1.0f;
//		float sheaviside = 1.0f / deto;
//		float sdelta = 4.0f* exp_dt * sheaviside * sheaviside;
//		float e = inpt.w * sdelta*TMP_WEIGHT + (1 - inpt.w)*sheaviside*(2 - TMP_WEIGHT);
//		return e;
//	}
//	else return 0.0f;
//}
