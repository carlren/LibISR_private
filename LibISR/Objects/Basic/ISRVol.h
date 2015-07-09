#pragma once
#include "../../Utils/LibISRDefine.h"
#include "../../../ORUtils/CUDADefines.h"

namespace LibISR
{
	namespace Objects
	{
		class ISRVol
		{
		private:

			float *data;

		public:

			Vector3i volSize;
			int allocatedSize;

			bool useGPU;
			
			_CPU_AND_GPU_CODE_ float* getVoxelData(){ return data; }
			_CPU_AND_GPU_CODE_ const float* getVoxelData() const	{ return data; }


			void initialize(Vector3i vsize = Vector3i(DT_VOL_SIZE), bool usegpu = false)
			{			
				useGPU = usegpu;
				volSize = vsize;
				allocatedSize = volSize.x*volSize.y*volSize.z;

				useGPU = usegpu;
				if (useGPU) ORcudaSafeCall(cudaMalloc((void**)&data, allocatedSize*sizeof(float)));
				else data = (float*)malloc(sizeof(float)*allocatedSize);

			}

			void  loadShapeFromFile(const char* fileName, Vector3i size = Vector3i(DT_VOL_SIZE))
			{
				volSize = size;
				allocatedSize = size.x*size.y*size.z;

				float *dt_host = new float[allocatedSize];

				FILE* f;
				f = fopen(fileName, "rb");
				fread(dt_host, sizeof(float) * this->allocatedSize, 1, f);
				fclose(f);

				if (useGPU)
				{
					ORcudaSafeCall(cudaMalloc((void**)&data, allocatedSize*sizeof(float)));
					ORcudaSafeCall(cudaMemcpy(data, dt_host, allocatedSize*sizeof(float), cudaMemcpyHostToDevice));
					delete[] dt_host;
				}
				else
				{
					data = dt_host;
				}
			}

			ISRVol(Vector3i vsize=Vector3i(DT_VOL_SIZE), bool usegpu=false)
			{
				initialize(vsize, usegpu);
			}

			~ISRVol()
			{
					if (useGPU) ORcudaSafeCall(cudaFree(data));
					else free(data);
			}
		};

		typedef ISRVol* ISRVol_ptr;
	}
}
