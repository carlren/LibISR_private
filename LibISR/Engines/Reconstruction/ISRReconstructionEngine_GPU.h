#pragma once
#include "ISRReconstructionEngine.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRReconstructionEngine_GPU : public ISRReconstructionEngine
		{
		public:
			void evolve3DShape(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRVol_ptr inpinvol, int iterno);
			void reinitializeSDF(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRShape_ptr tmpshape2);
		};
	}
}