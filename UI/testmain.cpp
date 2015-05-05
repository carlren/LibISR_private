
#include "../LibISR/LibISR.h"

#include "ImageSourceEngine.h"
#include "OpenNIEngine.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#pragma comment( lib, "opencv_core2410.lib" )
#pragma comment( lib, "opencv_highgui2410.lib" )


#include "../LibISRUtils/IOUtil.h"
#include "../LibISRUtils/Timer.h"
#include "../LibISRUtils/NVTimer.h"

#include "opencvhelper.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;
using namespace LibISRUtils;




void main___(int argc, char** argv)
{
	const char *tarsdfFile = "../Data/couch200.bin";
	const char *sdfFile = "../Data/initdt.bin";

	//////////////////////////////////////////////////////////////////////////
	// test stuff
	//////////////////////////////////////////////////////////////////////////
	bool usegpu = false;

	ISRShape_ptr initdt = new ISRShape(); initdt->initialize(usegpu, 0); initdt->loadShapeFromFile(sdfFile);
	ISRShape_ptr tmpshape = new ISRShape(); tmpshape->initialize(usegpu, 0); tmpshape->loadShapeFromFile(sdfFile);
	ISRShape_ptr tmpshape2 = new ISRShape(); tmpshape2->initialize(usegpu, 0); tmpshape2->loadShapeFromFile(sdfFile);
	ISRVol_ptr pinvol = new ISRVol(Vector3i(DT_VOL_SIZE),usegpu);  pinvol->loadShapeFromFile(tarsdfFile);
	
	
	ISRReconstructionEngine* myreco;
	ISRVisualisationEngine* myvengine;

	if (usegpu)
	{
		myvengine = new ISRVisualisationEngine_GPU();
		myreco = new ISRReconstructionEngine_GPU();
		
	}
	else
	{
		myvengine = new ISRVisualisationEngine_CPU();
		myreco = new ISRReconstructionEngine_CPU();
	}


	ISRPose_ptr pose = new ISRPose(); float pv[6] = { 0.0f, 0.0f, 0.3f, 1.0f, 0.0f, 1.0f };
	pose->setHFromParam(pv);
	ISRIntrinsics intrinsic; intrinsic.SetFrom(504, 504, 352, 272);


	ISRUShortImage* surfimg = new ISRUShortImage(Vector2i(640, 480),usegpu);
	ISRUChar4Image* normimg = new ISRUChar4Image(Vector2i(640, 480),usegpu);
	ISRVisualisationState* vstate = new ISRVisualisationState(Vector2i(640, 480), usegpu);
	myvengine->updateMinmaxmImage(vstate->minmaxImage, pose->getH(),intrinsic.A, Vector2i(640, 480));
	vstate->minmaxImage->UpdateDeviceFromHost();

	cvNamedWindow("evolution", 0);
	IplImage* depthFrame = cvCreateImage(cvSize(640, 480), 8, 4);

	StopWatchInterface *timer;
	sdkCreateTimer(&timer);

	for (int i = 1; i < 1000;i++)
	{
		sdkResetTimer(&timer); sdkStartTimer(&timer);
		myreco->evolve3DShape(initdt,tmpshape, pinvol, 1);
		sdkStopTimer(&timer); printf("\rEvolution time:[%.2f] \t", sdkGetTimerValue(&timer));

		if (i%2==0)
		{
			sdkResetTimer(&timer); sdkStartTimer(&timer);
			myreco->reinitializeSDF(initdt, tmpshape, tmpshape2);
			sdkStopTimer(&timer); printf("Reinitialization time:[%.2f]", sdkGetTimerValue(&timer));

			PrintArrayToFile("e:/libisr/debug/dt.txt", initdt->getSDFVoxel(), initdt->allocatedSize);
		}
		

		myvengine->renderDepthNormalAndObject(surfimg, normimg, vstate, pose->getInvH(), initdt, intrinsic.getParam());
		copydataISR2OpenCV(depthFrame, normimg);
		//copydataISR2OpenCV(depthFrame, vstate->outputImage);

		cvShowImage("evolution", depthFrame);
		cvWaitKey(10);
	}

	cvDestroyAllWindows();



}