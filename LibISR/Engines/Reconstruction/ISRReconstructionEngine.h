#pragma once

#include "../../Utils/LibISRDefine.h"

#include "../../Objects/Basic/ISRVol.h"
#include "../../Objects/Basic/ISRShape.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRReconstructionEngine
		{
		public:
			
			virtual void evolve3DShape(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRVol_ptr inpinvol, int iterno) = 0;
			virtual void reinitializeSDF(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRShape_ptr tmpshape2) = 0;

		
			ISRReconstructionEngine(){};
			~ISRReconstructionEngine(){};
		};
	}
}