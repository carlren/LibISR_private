#pragma once
#include "ISRReconstructionEngine.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRReconstructionEngine_CPU:public ISRReconstructionEngine
		{
			public:
				void evolve3DShape(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRVol_ptr inpinvol, int iterno);
			//virtual void allocatePinVolumn() = 0; 
			//virtual void intergratePinVolumn() = 0;
		//public:
		//	ISRReconstructionEngine_CPU(){};
		//	~ISRReconstructionEngine_CPU(){};
		};
	}
}