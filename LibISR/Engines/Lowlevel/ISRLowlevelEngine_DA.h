#pragma once

#include "../../Utils/LibISRDefine.h"

_CPU_AND_GPU_CODE_ inline void unprojectPtWithIntrinsic(const Vector4f intrinsic, const Vector3f &inpt, Vector4f &outpt)
{
	outpt.x = (inpt.x - inpt.z*intrinsic.z) / intrinsic.x;
	outpt.y = (inpt.y - inpt.z*intrinsic.w) / intrinsic.y;
	outpt.z = inpt.z;
	outpt.w = 1.0f;
}

template<class T>
_CPU_AND_GPU_CODE_ inline float getPf(const T &pixel, float* histogram, int noBins)
{
	int dim = noBins*noBins*noBins;

	int ru = pixel.r / noBins;
	int gu = pixel.g / noBins;
	int bu = pixel.b / noBins;

	int pidx = ru*noBins*noBins + gu * noBins + bu;

	return histogram[pidx];
}

template<class T>
_CPU_AND_GPU_CODE_ inline void mapRGBDtoRGB(T &rgb_out, const Vector3f& inpt, const Vector4u *rgb_in, const Vector2i& imgSize, const Matrix3f &H, const Vector3f &T)
{
	if (inpt.z>0)
	{
		Vector3f imgPt = H*inpt + T;
		int ix = (int)(imgPt.x / imgPt.z);
		int iy = (int)(imgPt.y / imgPt.z);

		if (ix >= 0 && ix < imgSize.x && iy >= 0 && imgSize.y)
		{
			rgb_out.x = rgb_in[iy * imgSize.x + ix].x;
			rgb_out.y = rgb_in[iy * imgSize.x + ix].y;
			rgb_out.z = rgb_in[iy * imgSize.x + ix].z;

			return;
		}
	}
	rgb_out.r = rgb_out.g = rgb_out.b = 0;
}


_CPU_AND_GPU_CODE_ inline void filterSubsampleWithHoles(Vector4f *imageData_out, int x, int y, Vector2i newDims, const Vector4f *imageData_in, Vector2i oldDims)
{
	int src_pos_x = x * 2, src_pos_y = y * 2;
	Vector4f pixel_out = 0.0f, pixel_in; float no_good_pixels = 0.0f;

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in.w > 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 0) * oldDims.x];
	if (pixel_in.w > 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 0) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in.w > 0) { pixel_out += pixel_in; no_good_pixels++; }

	pixel_in = imageData_in[(src_pos_x + 1) + (src_pos_y + 1) * oldDims.x];
	if (pixel_in.w > 0) { pixel_out += pixel_in; no_good_pixels++; }

	if (no_good_pixels > 0) pixel_out /= no_good_pixels;
	else { pixel_out.w = -1.0f; }

	imageData_out[x + y * newDims.x] = pixel_out;
}


_CPU_AND_GPU_CODE_ inline void normalizeRGB(const Vector4u &pix_in, Vector4u& pix_out)
{
	float r, g, b, nm, sm, nr, ng, nb;

	if (pix_in.r == 0 && pix_in.g == 0 && pix_in.b == 0) pix_out = pix_in;
	else
	{
		//nm = 1 / sqrtf(r*r + g*g + b*b);
		r = pix_in.r;
		g = pix_in.g;
		b = pix_in.b;

		nm = 1 / (r + g + b);
		nr = r*nm; ng = g*nm; nb = b*nm;
		pix_out.r = (uchar)(nr * 255);
		pix_out.g = (uchar)(ng * 255);
		pix_out.b = (uchar)(nb * 255);
	}
}



//_CPU_AND_GPU_CODE_ inline void normalizeRGB_brightness(const Vector4u &pix_in, Vector4f& pix_out)
//{
//	float r, g, b, nm, sm, nr, ng, nb;
//
//	if (pix_in.r == 0 && pix_in.g == 0 && pix_in.b == 0) pix_out = pix_in;
//	else
//	{
//		r = logf(pix_in.r);
//		g = logf(pix_in.g);
//		b = logf(pix_in.b);
//		sm = (r + g + b)*0.3333f;
//		r -= sm; g -= sm; b -= sm;
//		pix_out.r = (uchar)expf(r);
//		pix_out.g = (uchar)expf(g);
//		pix_out.b = (uchar)expf(b);
//	}
//}