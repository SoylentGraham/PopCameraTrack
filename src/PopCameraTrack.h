#pragma once
#include <ofxSoylent.h>
#include <SoyApp.h>
#include <TJob.h>
#include <TChannel.h>

#include "SlamSystem.h"
#include "IOWrapper/Output3DWrapper.h"

class SlamOutput : public lsd_slam::Output3DWrapper
{
public:
};


class TPopCameraTrack : public TJobHandler, public TChannelManager
{
public:
	TPopCameraTrack();
	
	virtual void	AddChannel(std::shared_ptr<TChannel> Channel) override;

	void			OnExit(TJobAndChannel& JobAndChannel);
	void			OnGetFeature(TJobAndChannel& JobAndChannel);
	void			OnNewFrame(TJobAndChannel& JobAndChannel);
	
	bool			UpdateSlam(SoyPixelsImpl& Pixels,std::stringstream& Error);
	
public:
	Soy::Platform::TConsoleApp	mConsoleApp;
	std::shared_ptr<lsd_slam::SlamSystem>	mSlam;
	SlamOutput					mSlamOutput;
	
	std::mutex		mSlamLock;
	unsigned int	mSlamFrameCounter;
	float			mSlamTimestamp;
};





