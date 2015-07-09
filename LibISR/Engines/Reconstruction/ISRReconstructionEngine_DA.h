#pragma once

#include "../Lowlevel/ISRVoxelAccess_DA.h"

#ifndef LONG_INFTY
#define LONG_INFTY 100000000
#endif

#ifndef sum_isr
#define sum_isr(a,b) ((((a)==LONG_INFTY) || ((b)==LONG_INFTY)) ? LONG_INFTY : (a)+(b))
#endif

#ifndef prod_isr
#define prod_isr(a,b) ((((a)==LONG_INFTY) || ((b)==LONG_INFTY)) ? LONG_INFTY : (a) * (b))
#endif
//
#ifndef opp_isr
#define opp_isr(a) (((a) == LONG_INFTY) ? LONG_INFTY : -(a))
#endif

#ifndef intdivint_isr
#define intdivint_isr(divid, divis) (((divis) == 0 || (divid) == LONG_INFTY) ? LONG_INFTY : (long)((divid) / (divis)))
#endif

_CPU_AND_GPU_CODE_ inline float F(int x, int i, float gi2) { return sum_isr((x - i)*(x - i), gi2); }
_CPU_AND_GPU_CODE_ inline float Sep(int i, int u, float gi2, float gu2) { return intdivint_isr(sum_isr(sum_isr((float)(u*u - i*i), gu2), opp_isr(gi2)), 2 * (u - i)); }



_CPU_AND_GPU_CODE_ inline void evolveVoxelOneStep(float* outsdfptr, float* sdfptr, float* pinptr, int i, int j, int k)
{

	bool dtfound;
	float dt = getSDFValue(Vector3i(i, j, k), sdfptr, dtfound);
	float DTDIV = 0.25f;

	Vector3f dtdm2, dtdp2, dtdm1, dtdp1, middt(dt);

	dtdm2.x = getSDFValue(Vector3i(i - 2, j, k), sdfptr, dtfound);
	dtdm2.y = getSDFValue(Vector3i(i, j - 2, k), sdfptr, dtfound);
	dtdm2.z = getSDFValue(Vector3i(i, j, k - 2), sdfptr, dtfound);

	dtdp2.x = getSDFValue(Vector3i(i + 2, j, k), sdfptr, dtfound);
	dtdp2.y = getSDFValue(Vector3i(i, j + 2, k), sdfptr, dtfound);
	dtdp2.z = getSDFValue(Vector3i(i, j, k + 2), sdfptr, dtfound);

	Vector3f dddt = (dtdp2 + dtdm2 - 2.0f*middt)*0.25f;
	float grad2 = (dddt.x + dddt.y + dddt.z);

	Vector3f tmpvec;

	tmpvec = getSDFNormal(Vector3i(i - 1, j, k), sdfptr, dtfound); tmpvec = tmpvec.normalised(); dtdm1.x = tmpvec.x;
	tmpvec = getSDFNormal(Vector3i(i, j - 1, k), sdfptr, dtfound); tmpvec = tmpvec.normalised(); dtdm1.y = tmpvec.y;
	tmpvec = getSDFNormal(Vector3i(i, j, k - 1), sdfptr, dtfound); tmpvec = tmpvec.normalised(); dtdm1.z = tmpvec.z;

	tmpvec = getSDFNormal(Vector3i(i + 1, j, k), sdfptr, dtfound); tmpvec = tmpvec.normalised(); dtdp1.x = tmpvec.x;
	tmpvec = getSDFNormal(Vector3i(i, j + 1, k), sdfptr, dtfound); tmpvec = tmpvec.normalised(); dtdp1.y = tmpvec.y;
	tmpvec = getSDFNormal(Vector3i(i, j, k + 1), sdfptr, dtfound); tmpvec = tmpvec.normalised(); dtdp1.z = tmpvec.z;

	//dtdm1.x = getSDFNormal(Vector3i(i - 1, j, k), sdfptr, dtfound).normalised().x;
	//dtdm1.y = getSDFNormal(Vector3i(i, j - 1, k), sdfptr, dtfound).normalised().y;
	//dtdm1.z = getSDFNormal(Vector3i(i, j, k - 1), sdfptr, dtfound).normalised().z;

	//dtdp1.x = getSDFNormal(Vector3i(i + 1, j, k), sdfptr, dtfound).normalised().x;
	//dtdp1.y = getSDFNormal(Vector3i(i, j + 1, k), sdfptr, dtfound).normalised().y;
	//dtdp1.z = getSDFNormal(Vector3i(i, j, k + 1), sdfptr, dtfound).normalised().z;


	Vector3f dddt_nml = (dtdp1 - dtdm1)*0.5f;
	float divg = (dddt_nml.x + dddt_nml.y + dddt_nml.z);

	float p_in = getSDFValue(Vector3i(i, j, k), pinptr, dtfound);

	float exp_dt = expf(dt * DTDIV);
	float deto = exp_dt + 1.0f;
	float sheaviside = 1.0f / deto;
	float d_heaviside_dt = exp_dt * sheaviside * sheaviside;

	float energyterm = (2.0f*p_in - 1.0f)*(d_heaviside_dt) / (p_in*sheaviside + (1 - p_in)*(1 - sheaviside) + 0.0001f);
	energyterm = energyterm > 1 ? 1 : energyterm;
	energyterm = energyterm < -1 ? -1 : energyterm;

	float constraintterm = 0.25*(grad2 - divg);
	float updateterm = constraintterm - energyterm;

	int idx = pt2IntIdx(Vector3i(i, j, k));
	outsdfptr[idx] += updateterm;
} 