#include "ISRReconstructionEngine_CPU.h"
#include "ISRReconstructionEngine_DA.h"

#include "../../../LibISRUtils/IOUtil.h"

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

void LibISR::Engine::ISRReconstructionEngine_CPU::reinitializeSDF(Objects::ISRShape_ptr inshape, Objects::ISRShape_ptr tmpshape, Objects::ISRShape_ptr tmpshape2)
{
	float* sdfptr = inshape->getSDFVoxel();
	float* tmpsdfptr = tmpshape->getSDFVoxel();
	float* tmpsdfptr2 = tmpshape2->getSDFVoxel();

	Vector3i voldim = inshape->volSize;
	bool dtfound;

	float tmpdt;
	int idx;

	// first find the boundary, reading from sdfptr, writing to tmpsdfptr

#pragma omp parallel
	{
#pragma omp for
		for (int i = 0; i < voldim.x; i++)for (int j = 0; j < voldim.y; j++)for (int k = 0; k < voldim.z; k++)
		{
			setSDFValue(Vector3i(i, j, k), tmpsdfptr2, LONG_INFTY);


			tmpdt = getSDFValue(Vector3i(i - 1, j, k), sdfptr, dtfound);
			tmpdt *= getSDFValue(Vector3i(i + 1, j, k), sdfptr, dtfound);
			if (!dtfound){setSDFValue(Vector3i(i, j, k), tmpsdfptr, 0.0f); continue;}
			if (tmpdt <= 0.0f){ setSDFValue(Vector3i(i, j, k), tmpsdfptr, 1.0f); continue; }

			tmpdt = getSDFValue(Vector3i(i, j - 1, k), sdfptr, dtfound);
			tmpdt *= getSDFValue(Vector3i(i, j + 1, k), sdfptr, dtfound);
			if (!dtfound){ setSDFValue(Vector3i(i, j, k), tmpsdfptr, 0.0f); continue; }
			if (tmpdt <= 0.0f){ setSDFValue(Vector3i(i, j, k), tmpsdfptr, 1.0f); continue; }


			tmpdt = getSDFValue(Vector3i(i, j, k - 1), sdfptr, dtfound);
			tmpdt *= getSDFValue(Vector3i(i, j, k + 1), sdfptr, dtfound);
			if (!dtfound){ setSDFValue(Vector3i(i, j, k), tmpsdfptr, 0.0f); continue; }
			if (tmpdt <= 0.0f){ setSDFValue(Vector3i(i, j, k), tmpsdfptr, 1.0f); continue; }
			setSDFValue(Vector3i(i, j, k), tmpsdfptr, 0.0f);
		}

		
		// do x scan, reading from tmpsdfptr, writing to tmpsdfptr2
#pragma omp for
		for (int z = 0; z < voldim.z; z++) for (int y = 0; y < voldim.y; y++)
		{
			setSDFValue(Vector3i(0, y, z), tmpsdfptr2, LONG_INFTY);

			// Forward scan
			for (int x = 1; x < voldim.x; x++)
			{
				if (getSDFValue(Vector3i(x, y, z), tmpsdfptr, dtfound) == 1.0f) setSDFValue(Vector3i(x, y, z), tmpsdfptr2, 0.0f);
				else setSDFValue(Vector3i(x, y, z), tmpsdfptr2, sum_isr(1.0f, getSDFValue(Vector3i(x - 1, y, z), tmpsdfptr2, dtfound)));
			}

			// Backward scan
			for (int x = voldim.x - 2; x >= 0; x--)
			{
				tmpdt = getSDFValue(Vector3i(x + 1, y, z), tmpsdfptr2, dtfound);
				if (tmpdt < getSDFValue(Vector3i(x, y, z), tmpsdfptr2, dtfound)) setSDFValue(Vector3i(x, y, z), tmpsdfptr2, sum_isr(1.0f, tmpdt));
			}
		}

		int q, w;

		int s[DT_VOL_SIZE], t[DT_VOL_SIZE], buff[DT_VOL_SIZE];


		// do y scan, reading from tmpsdfptr2, witting to tmpsdfptr
#pragma omp for
		for (int z = 0; z < voldim.z; z++) for (int x = 0; x < voldim.x; x++)
		{
			q = 0; s[0] = 0; t[0] = 0;
			for (int u = 0; u < voldim.y; u++)  buff[u] = getSDFValue(Vector3i(x, u, z), tmpsdfptr2, dtfound);

			//Forward Scan
			for (int u = 1; u < voldim.y; u++)
			{
				float val1 = prod_isr(buff[u], buff[u]), val2 = LONG_INFTY;

				while (q >= 0)
				{
					val2 = prod_isr(buff[s[q]], buff[s[q]]);
					if (F(t[q], s[q], val2) <= F(t[q], u, val1)) break;

					q--;
				}

				if (q < 0) { q = 0; s[0] = u; }
				else { w = 1 + Sep(s[q], u, val2, val1); if (w < voldim.y) { q++; s[q] = u; t[q] = w; } }
			}

			//Backward Scan
			for (int u = voldim.y - 1; u >= 0; --u)
			{
				setSDFValue(Vector3i(x, u, z), tmpsdfptr, sqrtf(F(u, s[q], prod_isr(buff[s[q]], buff[s[q]]))));
				if (u == t[q]) q--;
			}
		}

		//PrintArrayToFile("e:/libisr/debug/scany.txt", tmpsdfptr, inshape->allocatedSize);

		// do z scan, reading from tmpsdfptr, witting to sdfptr
#pragma omp for
		for (int y = 0; y < voldim.y; y++) for (int x = 0; x < voldim.x; x++)
		{
			q = 0; s[0] = 0; t[0] = 0;

			for (int u = 0; u < voldim.z; u++) buff[u] = getSDFValue(Vector3i(x, y, u), tmpsdfptr, dtfound);

			//Forward Scan
			for (int u = 1; u < voldim.z; u++)
			{
				while ((q >= 0) && (F(t[q], s[q], buff[s[q]]) > F(t[q], u, buff[u]))) q--;

				if (q < 0) { q = 0; s[0] = u; }
				else { w = 1 + Sep(s[q], u, buff[s[q]], buff[u]); if (w < voldim.z) { q++; s[q] = u; t[q] = w; } }
			}

			//Backward Scan
			for (int u = voldim.z - 1; u >= 0; --u)
			{
				setSDFValue(Vector3i(x, y, u), tmpsdfptr2, sqrtf(F(u, s[q], prod_isr(buff[s[q]], buff[s[q]]))));
				if (u == t[q]) q--;
			}
		}

		PrintArrayToFile("e:/libisr/debug/scanz.txt", tmpsdfptr2, inshape->allocatedSize);
		

#pragma omp for
		for (int i = 0; i < voldim.x; i++)for (int j = 0; j < voldim.y; j++)for (int k = 0; k < voldim.z; k++)
		{
			if (getSDFValue(Vector3i(i, j, k), sdfptr, dtfound)<0.0f)
				setSDFValue(Vector3i(i, j, k), sdfptr, -(getSDFValue(Vector3i(i, j, k), tmpsdfptr2, dtfound)));
			else setSDFValue(Vector3i(i, j, k), sdfptr, (getSDFValue(Vector3i(i, j, k), tmpsdfptr2, dtfound)));
		}
	}
}
