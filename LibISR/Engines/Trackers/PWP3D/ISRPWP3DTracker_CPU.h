#pragma once

#include "ISRPWP3DTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRPWP3DTracker_CPU :public ISRPWP3DTracker
		{
		protected:

			void prepareDataForPWP3D(ISRFloat4Image* outrefptlist, ISRFloat4Image* inimg, Objects::ISRHistogram *histogram, const Vector4i &boundingbox);

			void evaluateEnergy(float *energy, Objects::ISRPose_ptr pose);

			void computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRPose_ptr pose) const;

		public:
			ISRPWP3DTracker_CPU(int nObjs);
			~ISRPWP3DTracker_CPU();


		};

	}
}