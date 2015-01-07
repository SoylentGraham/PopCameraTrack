#pragma once
#include <ofxSoylent.h>
#include <SoyApp.h>
#include <TJob.h>
#include <TChannel.h>

#include "SlamSystem.h"
#include "IOWrapper/Output3DWrapper.h"



class TCameraPose
{
public:
	vec3f	mPosition;
	vec4f	mQuaternion;
};


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





