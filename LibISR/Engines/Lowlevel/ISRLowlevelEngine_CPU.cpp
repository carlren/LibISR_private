#include "ISRLowlevelEngine_CPU.h"
#include "ISRLowlevelEngine_DA.h"

#include "../../../LibISRUtils/IOUtil.h"

using namespace LibISR;
using namespace LibISR::Engine;
using namespace LibISR::Objects;

//////////////////////////////////////////////////////////////////////////
// Below are the functions that are currently used
//////////////////////////////////////////////////////////////////////////

void LibISR::Engine::ISRLowlevelEngine_CPU::subsampleImageRGBDImage(ISRFloat4Image *outimg, ISRFloat4Image *inimg)
{
	Vector2i oldDims = inimg->noDims;
	Vector2i newDims; newDims.x = inimg->noDims.x / 2; newDims.y = inimg->noDims.y / 2;

	outimg->ChangeDims(newDims);

	const Vector4f *imageData_in = inimg->GetData(false);
	Vector4f *imageData_out = outimg->GetData(false);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void LibISR::Engine::ISRLowlevelEngine_CPU::prepareAlignedRGBDData(ISRFloat4Image *outimg, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home)
{
	int w = raw_depth_in->noDims.width;
	int h = raw_depth_in->noDims.height;

	short* depth_ptr = raw_depth_in->GetData(false);
	Vector4u* rgb_in_ptr = rgb_in->GetData(false);
	Vector4f* rgbd_out_ptr = outimg->GetData(false);

	bool alreadyAligned = home->T == Vector3f(0, 0, 0);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		ushort rawdepth = depth_ptr[idx];
		float z = rawdepth == 65535 ? 0 : ((float)rawdepth) / 1000.0f;

		if (alreadyAligned)
		{
			rgbd_out_ptr[idx].x = rgb_in_ptr[idx].r;
			rgbd_out_ptr[idx].y = rgb_in_ptr[idx].g;
			rgbd_out_ptr[idx].z = rgb_in_ptr[idx].b;
			rgbd_out_ptr[idx].w = z;
		}
		else
		{
			mapRGBDtoRGB(rgbd_out_ptr[idx], Vector3f(j*z, i*z, z), rgb_in_ptr, raw_depth_in->noDims, home->H, home->T);
			rgbd_out_ptr[idx].w = z;
		}

	}

}

void LibISR::Engine::ISRLowlevelEngine_CPU::preparePointCloudFromAlignedRGBDImage(ISRFloat4Image *ptcloud_out, ISRFloat4Image *inimg, Objects::ISRHistogram *histogram, const Vector4f &intrinsic, const Vector4i &boundingbox)
{
	if (inimg->noDims != ptcloud_out->noDims) ptcloud_out->ChangeDims(inimg->noDims);
	
	int w = inimg->noDims.width;
	int h = inimg->noDims.height;

	int noBins = histogram->noBins;

	Vector4f *inimg_ptr = inimg->GetData(false);
	Vector4f* ptcloud_ptr = ptcloud_out->GetData(false);
	
	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		if (j < boundingbox.x || j >= boundingbox.z || i < boundingbox.y || i >= boundingbox.w)
		{
			ptcloud_ptr[idx] = Vector4f(0, 0, 0, -1);
		}
		else
		{
			float z = inimg_ptr[idx].w;
			unprojectPtWithIntrinsic(intrinsic, Vector3f(j*z, i*z, z), ptcloud_ptr[idx]);
			ptcloud_ptr[idx].w = getPf(inimg_ptr[idx], histogram->posterior, noBins);
		}
	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::computepfImageFromHistogram(ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram)
{
	int w = rgb_in->noDims.width;
	int h = rgb_in->noDims.height;
	int noBins = histogram->noBins;
	float pf = 0;

	Vector4u *inimg_ptr = rgb_in->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		pf = getPf(inimg_ptr[idx], histogram->posterior, noBins);
		if (pf > 0.5f)
		{
			inimg_ptr[idx].r = 255;
			inimg_ptr[idx].g = 0;
			inimg_ptr[idx].b = 0;
		}
		else if (pf == 0.5f)
		{
			inimg_ptr[idx].r = 0;
			inimg_ptr[idx].g = 0;
			inimg_ptr[idx].b = 255;
		}

	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::convertNormalizedRGB(ISRUChar4Image* inrgb, ISRUChar4Image* outrgb)
{
	int w = inrgb->noDims.width;
	int h = inrgb->noDims.height;

	Vector4u *inimg_ptr = inrgb->GetData(false);
	Vector4u *outimg_ptr = outrgb->GetData(false);

	float r, g, b, nm, nr, ng, nb;

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		r = inimg_ptr[idx].r;
		g = inimg_ptr[idx].g;
		b = inimg_ptr[idx].b;
		
		if (r == 0, g == 0, b == 0) outimg_ptr[idx] = Vector4u((uchar)0);
		else
		{
			nm = 1/sqrtf(r*r + g*g + b*b);
			nr = r*nm; ng = g*nm; nb = b*nm;
			outimg_ptr[idx].r = (uchar)(nr*255);
			outimg_ptr[idx].g = (uchar)(ng*255);
			outimg_ptr[idx].b = (uchar)(nb*255);
		}
	}
}


#ifndef SDFsum
#define  LONG_INFTY 100000001 
#define SDFsum(a,b) ((((a)==LONG_INFTY) || ((b)==LONG_INFTY)) ? LONG_INFTY : (a)+(b))
#define SDFprod(a,b) ((((a)==LONG_INFTY) || ((b)==LONG_INFTY)) ? LONG_INFTY : (a) * (b))
#define SDFintdivint(divid, divis) (((divis) == 0 || (divid) == LONG_INFTY) ? LONG_INFTY : (float)((divid) / (divis)))
#define SDFopp(a) (((a) == LONG_INFTY) ? LONG_INFTY : -(a))
#endif

float F(int x, int i, float gi2) { return SDFsum((x - i)*(x - i), gi2); }
float Sep(int i, int u, float gi2, long gu2) { return SDFintdivint(SDFsum(SDFsum((float)(u*u - i*i), gu2), SDFopp(gi2)), 2 * (u - i)); }


void dt2d_passX(float* outsdf, uchar* inmask, Vector4i bb, Vector2i imgsize, int masktype)
{
	for (int y = bb.y; y < bb.w; y++)
	{
		// set max and min
		if (inmask[0 + y * imgsize.x] == masktype) outsdf[0 + y * imgsize.x] = 0;
		else outsdf[0 + y * imgsize.x] = LONG_INFTY;

		// Forward scan
		for (int x = bb.x+1; x < bb.z; x++)
			if (inmask[x + y * imgsize.x] == masktype) outsdf[x + y * imgsize.x] = 0;
			else outsdf[x + y * imgsize.x] = SDFsum(1, outsdf[x - 1 + y * imgsize.x]);

		//Backward scan
			for (int x = bb.z - 2; x >= 0; x--)
				if (outsdf[x + 1 + y * imgsize.x] < outsdf[x + y *  imgsize.x])
					outsdf[x + y * imgsize.x] = SDFsum(1, outsdf[x + 1 + y * imgsize.x]);
	}
}


void dt2d_passY(float *sdt_xy, float *sdt_x, Vector4i bb, Vector2i imgsize)
{
	int q, w;

	int s[480], t[480], buff[480];

	for (int x = bb.x; x < bb.z; x++)
	{
		q = 0; s[0] = 0; t[0] = 0;

		for (int y = bb.y; y < bb.w; y++) {	buff[y] = sdt_x[x + y * imgsize.x];	}

		//Forward Scan
		for (int y = bb.y+1; y < bb.w; y++)
		{
			float val1 = SDFprod(buff[y], buff[y]), val2 = LONG_INFTY;

			while (q >= 0)
			{
				val2 = SDFprod(buff[s[q]], buff[s[q]]);
				if (F(t[q], s[q], val2) <= F(t[q], y, val1)) break;
				q--;
			}

			if (q < 0) 
			{ q = 0; s[0] = y; }
			else {
				w = 1 + Sep(s[q], y, val2, val1); 
				if (w < bb.w) 
				{ 
					q++; s[q] = y; t[q] = w; 
				} 
			}
		}

		//Backward Scan
		for (int y = bb.w - 1; y >= bb.y; --y)
		{
			sdt_xy[x + y * imgsize.x] = sqrtf(F(y, s[q], SDFprod(buff[s[q]], buff[s[q]])));
			if (y == t[q]) q--;
		}
	}
}


void LibISR::Engine::ISRLowlevelEngine_CPU::computeSDFFromMask(ISRFloatImage* outsdf, ISRUCharImage* inmask, Vector4i bb)
{
	float* sdf_ptr = outsdf->GetData(false);
	uchar*  mask_ptr = inmask->GetData(false);
	Vector2i imgSize = inmask->noDims;

	ISRFloatImage* outsdf_x = new ISRFloatImage(imgSize, false);
	ISRFloatImage* tmpsdf = new ISRFloatImage(imgSize, false);

	outsdf_x->Clear(0);

	dt2d_passX(outsdf_x->GetData(false), mask_ptr, bb, imgSize,0);
	dt2d_passY(tmpsdf->GetData(false), outsdf_x->GetData(false), bb, imgSize);

	dt2d_passX(outsdf_x->GetData(false), mask_ptr, bb, imgSize,1);
	dt2d_passY(sdf_ptr, outsdf_x->GetData(false), bb, imgSize);

	for (int i = 0; i < outsdf->dataSize;i++)
		if (mask_ptr[i] == 1) sdf_ptr[i] = -tmpsdf->GetData(false)[i];
}

