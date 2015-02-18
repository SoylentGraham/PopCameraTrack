#pragma once
#include <ofxSoylent.h>
#include <SoyApp.h>
#include <TJob.h>
#include <TChannel.h>

#include "TJobEventSubscriber.h"
#include "TCameraPose.h"
#include <TFeatureBinRing.h>

#if defined(ENABLE_LSDSLAM)
#include "SlamSystem.h"
#include "IOWrapper/Output3DWrapper.h"
#endif


#if defined(ENABLE_LSDSLAM)
class SlamOutput : public lsd_slam::Output3DWrapper
{
public:
	virtual void	publishKeyframeGraph(lsd_slam::KeyFrameGraph* graph) override;
	
	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void	publishKeyframe(lsd_slam::Frame* kf)  override;
	
	// published a tracked frame that did not become a keyframe (yet; i.e. has no depth data)
	virtual void	publishTrackedFrame(lsd_slam::Frame* kf) override;
	
	// publishes graph and all constraints, as well as updated KF poses.
	virtual void	publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier) override;
	virtual void	publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier) override;
	
	virtual void	publishDebugInfo(Eigen::Matrix<float, 20, 1> data) override;

public:
	SoyEvent<TCameraPose>	mOnNewCameraPose;
};
#endif


class TTrackerState
{
public:
	Array<TFeatureMatch>		mFeatures;	//	base/original features we're matching
	SoyTime						mFeaturesTimestamp;	//	timestamp for mFeatures
	SoyPixels					mLastImage;
};

//	persistent feature tracker
class TFeatureTracker
{
public:
	void						UpdateFeatures(const ArrayBridge<TFeatureMatch>&& NewFeatures,SoyTime Timestamp,const SoyPixels& Image);

	bool						HasBaseFeatures() const	{	return !mBase.mFeatures.IsEmpty();	}
	
public:
	TTrackerState				mBase;
	std::string					mSerial;	//	serial of the camera we're tracking
	SoyEvent<TTrackerState>		mOnNewState;
	
	ofMutexM<bool>				mPendingFeatures;
};




class TPopCameraTrack : public TJobHandler, public TChannelManager
{
public:
	TPopCameraTrack(TJobParams& Params);
	
	virtual void	AddChannel(std::shared_ptr<TChannel> Channel) override;

	void			OnExit(TJobAndChannel& JobAndChannel);
	void			OnGetFeature(TJobAndChannel& JobAndChannel);
	void			OnNewFrame(TJobAndChannel& JobAndChannel);
	void			SubscribeNewCameraPose(TJobAndChannel& JobAndChannel);
	void			OnResetSlam(TJobAndChannel& JobAndChannel);
	void			OnFoundInterestingFeatures(TJobAndChannel& JobAndChannel);
	void			OnTrackedFeatures(TJobAndChannel& JobAndChannel);
	void			SubscribeNewFeatures(TJobAndChannel& JobAndChannel);

	bool			UpdateSlam(SoyPixelsImpl& Pixels,std::stringstream& Error);
	
	bool			OnNewFeatureStateCallback(TEventSubscriptionManager& SubscriptionManager,TJobChannelMeta Client,TTrackerState& State);

public:
	Soy::Platform::TConsoleApp	mConsoleApp;
	TSubscriberManager	mSubcriberManager;

#if defined(ENABLE_LSDSLAM)
	std::shared_ptr<lsd_slam::SlamSystem>	mSlam;
	SlamOutput					mSlamOutput;
	
	std::recursive_mutex	mSlamLock;
	unsigned int	mSlamFrameCounter;
	float			mSlamTimestamp;
#endif
	
	TFeatureBinRingParams		mFeatureParams;
	
	std::shared_ptr<TChannel>		mFeatureChannel;
	std::map<std::string,TFeatureTracker>	mFeatureTrackers;	//	feature tracker for each camera serial
};





