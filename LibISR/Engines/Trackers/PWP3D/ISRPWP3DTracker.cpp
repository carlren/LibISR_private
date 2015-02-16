#include "ISRPWP3DTracker.h"

#include <math.h>
#include <stdio.h>

#include "../../../../ORUtils/Cholesky.h"
#include "../../../../LibISRUtils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;



LibISR::Engine::ISRPWP3DTracker::ISRPWP3DTracker(int nObjs, bool useGPU)
{
	ATb_Size = 6;
	ATA_size = ATb_Size*ATb_Size;

	ATb_host = new float[ATb_Size];
	ATA_host = new float[ATA_size];

	acceptedPose = new Objects::ISRPose();
	tmpPose = new Objects::ISRPose();
	
}

LibISR::Engine::ISRPWP3DTracker::~ISRPWP3DTracker()
{
	delete[] ATb_host;
	delete[] ATA_host;

	delete acceptedPose;
	delete tmpPose;
}


void computeSingleStep(float *step, float *ATA, float *ATb, float lambda, int dim)
{
	float *tmpATA = new float[dim*dim];
	for (int i = 0; i < dim*dim; i++) tmpATA[i] = ATA[i];

	for (int i = 0; i < dim * dim; i += (dim + 1))
	{
		float &ele = tmpATA[i];
		if (!(fabs(ele) < 1e-15f)) ele *= (1.0f + lambda); else ele = lambda*1e-10f;
	}

	ORUtils::Cholesky cholA(tmpATA, dim);
	cholA.Backsub(step, ATb);
}


void LibISR::Engine::ISRPWP3DTracker::TrackObjects(ISRFrame *frame, ISRShapeUnion *shapeUnion, ISRTrackingState *trackerState, bool updateappearance)
{
	this->frame = frame;
	this->shape = shapeUnion->getShape(0);
	this->acceptedPose->setFromH(trackerState->getPose(0)->getH());
	this->tmpPose->setFromH(this->acceptedPose->getH());

	float *cacheNabla = new float[ATb_Size];

	float lastenergy = 0;
	float currentenergy = 0;

	bool converged = false;
	float lambda = 1000.0f;

	// These are some sensible default parameters for Levenberg Marquardt.
	// The first three control the convergence criteria, the others might
	// impact convergence speed.
	static const int MAX_STEPS = 100;
	static const float MIN_STEP = 0.00005f;
	static const float MIN_DECREASE = 0.0001f;
	static const float TR_REGION_INCREASE = 0.10f;
	static const float TR_REGION_DECREASE = 10.0f;

	{// minimalist LM main loop
		evaluateEnergy(&lastenergy, acceptedPose);

		if (lastenergy < 0.1f) { trackerState->energy = 0; return; }

		for (int iter = 0; iter < MAX_STEPS; iter++)
		{
			computeJacobianAndHessian(ATb_host, ATA_host, tmpPose);

			while (true)
			{
				computeSingleStep(cacheNabla, ATA_host, ATb_host, lambda, ATb_Size);

				// check if step size is very small, if so, converge.
				float MAXnorm = 0.0;
				for (int i = 0; i<ATb_Size; i++) { float tmp = fabs(cacheNabla[i]); if (tmp>MAXnorm) MAXnorm = tmp; }
				if (MAXnorm < MIN_STEP) { converged = true; break; }

				tmpPose->applyIncrementalChangeToH(cacheNabla);

				evaluateEnergy(&currentenergy, tmpPose);

				if (currentenergy > lastenergy)
				{
					// check if energy decrease is too small, if so, converge.
					if (abs(currentenergy - lastenergy) / abs(lastenergy) < MIN_DECREASE) { converged = true; }
					lastenergy = currentenergy;
					lambda *= TR_REGION_INCREASE;
					acceptedPose->setFromH(tmpPose->getH());
					break;
				}
				else
				{
					lambda *= TR_REGION_DECREASE;
					tmpPose->setFromH(acceptedPose->getH());
				}
			}
			if (converged) break;
		}

	}


	trackerState->getPose(0)->setFromH(acceptedPose->getH());
	trackerState->energy = lastenergy;
}
