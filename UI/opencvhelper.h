#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "../LibISR/LibISR.h"

void inline updateHistogramFromRendering(ISRUChar4Image* rendering, ISRUChar4Image* rgb, LibISR::Objects::ISRHistogram* hist)
{
	Vector4u* imgptr = rendering->GetData(false);
	Vector4u bpix((uchar)0);
	for (int i = 0; i < rendering->dataSize; i++)
		if (imgptr[i] != bpix) imgptr[i] = Vector4u(255, 255, 255, 255);
		else imgptr[i] = Vector4u(100, 100, 100, 100);

		hist->buildHistogram(rgb, rendering);

}

void inline copydataISR2OpenCV(IplImage* outimg, ISRUChar4Image* inimg)
{
	uchar* outimg_ptr = (uchar*)outimg->imageData;
	Vector4u* inimg_ptr = inimg->GetData(false);

	for (int i = 0; i < inimg->dataSize; i++)
	{
		outimg_ptr[i * 4 + 0] = inimg_ptr[i].b;
		outimg_ptr[i * 4 + 1] = inimg_ptr[i].g;
		outimg_ptr[i * 4 + 2] = inimg_ptr[i].r;
	}
}