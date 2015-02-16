#include "ISRPWP3DTracker_CPU.h"
#include "ISRPWP3DTracker_DA.h"
#include "../../Lowlevel/ISRLowlevelEngine_DA.h"


void LibISR::Engine::ISRPWP3DTracker_CPU::evaluateEnergy(float *energy, Objects::ISRPose_ptr pose)
{
	//int count = this->frame->ptCloud->dataSize;
	//Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);

	//float e = 0, es = 0;
	//int totalpix = 0;
	//int totalpfpix = 0;

	//for (int i = 0; i < count; i++)
	//{
	//	es = computePerPixelEnergy(ptcloud_ptr[i], shapes, poses, objCount);
	//	if (es > 0)
	//	{
	//		e += es; totalpix++;
	//		if (ptcloud_ptr[i].w > 0.5) totalpfpix++;
	//	}

	//}

	//energy[0] = totalpfpix > 100 ? e / totalpix : 0.0f;
}

void LibISR::Engine::ISRPWP3DTracker_CPU::computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRPose_ptr pose) const
{

}

void LibISR::Engine::ISRPWP3DTracker_CPU::prepareDataForPWP3D(ISRFloat4Image* outrefptlist, ISRFloat4Image* inimg, Objects::ISRHistogram *histogram, const Vector4i &boundingbox)
{
	if (inimg->noDims != outrefptlist->noDims) outrefptlist->ChangeDims(inimg->noDims);

	int w = inimg->noDims.width;
	int h = inimg->noDims.height;

	int noBins = histogram->noBins;

	Vector4f *inimg_ptr = inimg->GetData(false);
	Vector4f* ptcloud_ptr = outrefptlist->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		if (j < boundingbox.x || j >= boundingbox.z || i < boundingbox.y || i >= boundingbox.w)
			ptcloud_ptr[idx] = Vector4f(0, 0, 0, -1);
		else
			ptcloud_ptr[idx].w = getPf(inimg_ptr[idx], histogram->posterior, noBins);
	}
}
