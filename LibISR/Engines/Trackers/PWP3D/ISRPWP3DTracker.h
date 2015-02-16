#pragma once
#include "../ISRTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRPWP3DTracker : public ISRTracker
		{
		protected:

			Objects::ISRPose_ptr acceptedPose;

			Objects::ISRPose_ptr tmpPose;

			Objects::ISRShape_ptr shape;

			Objects::ISRFrame *frame;

			// size of the gradient
			int ATb_Size; // (6*nObjects)

			// size of the Hessian
			int ATA_size; // (Atb_size^2)

			// Hessian approximated with JTJ
			float* ATA_host;

			// gradient
			float* ATb_host;

			virtual void prepareDataForPWP3D(ISRFloat4Image* ptWithPf, ISRFloat4Image* dtddt, ISRFloat4Image* inimg, ISRFloatImage* dt2d, Objects::ISRHistogram *histogram, const Vector4i &boundingbox) = 0;

			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			virtual void evaluateEnergy(float *energy, Objects::ISRPose_ptr pose) = 0;

			// compute the Hessian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			virtual void computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRPose_ptr pose) const = 0;

		public:

			int numParameters() const { return ATb_Size; }

			void TrackObjects(Objects::ISRFrame *frame, Objects::ISRShapeUnion *shapeUnion, Objects::ISRTrackingState *trackerState, bool updateappearance = false);

			ISRPWP3DTracker(int nObjs, bool useGPU);
			~ISRPWP3DTracker();
		};

	}
}

