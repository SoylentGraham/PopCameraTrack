#include "PopCameraTrack.h"
#include <TParameters.h>
#include <SoyDebug.h>
#include <TProtocolCli.h>
#include <TProtocolHttp.h>
#include <SoyApp.h>
#include <PopMain.h>
#include <TJobRelay.h>
#include <SoyPixels.h>
#include <SoyString.h>

#if defined(ENABLE_LSDSLAM)
#include "DataStructures/Frame.h"
#endif

void TFeatureTracker::UpdateFeatures(const ArrayBridge<TFeatureMatch>&& NewFeatures,SoyTime Timestamp,const SoyPixels& Image)
{
	TTrackerState NewState;
	NewState.mFeaturesTimestamp = Timestamp;
	NewState.mFeatures = NewFeatures;
	NewState.mLastImage = Image;

	mOnNewState.OnTriggered( NewState );
}


#if defined(ENABLE_LSDSLAM)
void SlamOutput::publishKeyframeGraph(lsd_slam::KeyFrameGraph* graph)
{
	//std::Debug << __FUNCTION__ << std::endl;
}

void SlamOutput::publishKeyframe(lsd_slam::Frame* kf)
{
	std::Debug << __FUNCTION__ << std::endl;
}

void SlamOutput::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	std::Debug << __FUNCTION__ << std::endl;
}

void SlamOutput::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	std::Debug << __FUNCTION__ << std::endl;
}
	
void SlamOutput::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	std::Debug << __FUNCTION__ << std::endl;
}



void SlamOutput::publishTrackedFrame(lsd_slam::Frame* kf)
{
	std::Debug << __FUNCTION__ << std::endl;
	
	
	TCameraPose CameraPose;
	
	//	extract camera pose
	SE3 camToWorld = lsd_slam::se3FromSim3(kf->getScaledCamToWorld());
	CameraPose.mPosition.x = camToWorld.translation()[0];
	CameraPose.mPosition.y = camToWorld.translation()[1];
	CameraPose.mPosition.z = camToWorld.translation()[2];

	CameraPose.mQuaternion.x = camToWorld.so3().unit_quaternion().x();
	CameraPose.mQuaternion.y = camToWorld.so3().unit_quaternion().y();
	CameraPose.mQuaternion.z = camToWorld.so3().unit_quaternion().z();
	CameraPose.mQuaternion.w = camToWorld.so3().unit_quaternion().w();

	if ( CameraPose.mQuaternion.w < 0)
		CameraPose.mQuaternion *= -1.f;

	std::Debug << "camera: " << CameraPose.mPosition.x << "," << CameraPose.mPosition.y << "," << CameraPose.mPosition.z << std::endl;
	mOnNewCameraPose.OnTriggered( CameraPose );
}
#endif



TPopCameraTrack::TPopCameraTrack() :
	mSubcriberManager	( *this )
{
	AddJobHandler("exit", TParameterTraits(), *this, &TPopCameraTrack::OnExit );
	
	AddJobHandler("newframe", TParameterTraits(), *this, &TPopCameraTrack::OnNewFrame );
	AddJobHandler("re:getframe", TParameterTraits(), *this, &TPopCameraTrack::OnNewFrame );

	TParameterTraits SubscribeNewCameraPoseTraits;
	SubscribeNewCameraPoseTraits.mDefaultParams.PushBack( std::make_tuple(std::string("command"),std::string("newcamerapose")) );
	AddJobHandler("subscribenewcamerapose", SubscribeNewCameraPoseTraits, *this, &TPopCameraTrack::SubscribeNewCameraPose );

	
	AddJobHandler("resetslam", TParameterTraits(), *this, &TPopCameraTrack::OnResetSlam );

	AddJobHandler("re:findinterestingfeatures", TParameterTraits(), *this, &TPopCameraTrack::OnFoundInterestingFeatures );
	AddJobHandler("re:trackfeatures", TParameterTraits(), *this, &TPopCameraTrack::OnTrackedFeatures );

	TParameterTraits SubscribeNewFeatures;
	SubscribeNewFeatures.mAssumedKeys.PushBack("serial");
	SubscribeNewFeatures.mDefaultParams.PushBack( std::make_tuple(std::string("command"),std::string("newfeatures")) );
	AddJobHandler("subscribenewfeatures", SubscribeNewFeatures, *this, &TPopCameraTrack::SubscribeNewFeatures );
}

void TPopCameraTrack::AddChannel(std::shared_ptr<TChannel> Channel)
{
	TChannelManager::AddChannel( Channel );
	if ( !Channel )
		return;
	TJobHandler::BindToChannel( *Channel );
}

void TPopCameraTrack::OnExit(TJobAndChannel& JobAndChannel)
{
	mConsoleApp.Exit();
	
	//	should probably still send a reply
	TJobReply Reply( JobAndChannel );
	Reply.mParams.AddDefaultParam(std::string("exiting..."));
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}


void TPopCameraTrack::OnResetSlam(TJobAndChannel& JobAndChannel)
{
#if defined(ENABLE_LSDSLAM)
	std::lock_guard<std::recursive_mutex> Lock( mSlamLock );
	mSlam.reset();
#endif
}



void TPopCameraTrack::OnNewFrame(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();
	
	//	pull image
	auto ImageParam = Job.mParams.GetDefaultParam();

	/*
	SoyPixels Image;
	std::Debug << "Getting image from " << ImageParam.GetFormat() << std::endl;
	if ( !ImageParam.Decode( Image ) )
	{
		std::Debug << "Failed to decode image" << std::endl;
		return;
	}
*/
#if defined(ENABLE_LSDSLAM)
	std::stringstream SlamError;
	UpdateSlam( Image, SlamError );
	if ( !SlamError.str().empty() )
		std::Debug << "Update slam failed: " << SlamError.str() << std::endl;
#endif

	
	//	get features from feature channel
	if ( mFeatureChannel )
	{
		//	update feature tracker
		auto SerialParam = Job.mParams.GetParam("serial");
		if ( !SerialParam.IsValid() )
		{
			std::Debug << "New frame missing serial, can't update feature tracker" << std::endl;
			return;
		}

		auto Serial = SerialParam.Decode<std::string>();
		auto& FeatureTracker = mFeatureTrackers[Serial];
		
		//	get initial interesting features if we haven't got any yet
		TJob GetFeaturesJob;
		if ( !FeatureTracker.HasBaseFeatures() )
		{
			GetFeaturesJob.mParams.mCommand = "findinterestingfeatures";
		}
		else
		{
			GetFeaturesJob.mParams.mCommand = "trackfeatures";
			GetFeaturesJob.mParams.AddParam("sourcefeatures", FeatureTracker.mBase.mFeatures );
		}
		
		GetFeaturesJob.mParams.AddDefaultParam( ImageParam.mSoyData );
		GetFeaturesJob.mParams.AddParam( SerialParam );
		GetFeaturesJob.mParams.AddParam("asbinary",true);
		
		GetFeaturesJob.mChannelMeta.mChannelRef = mFeatureChannel->GetChannelRef();
		mFeatureChannel->SendCommand( GetFeaturesJob );
	}
}

void TPopCameraTrack::OnTrackedFeatures(TJobAndChannel& JobAndChannel)
{
	const TJob& Job = JobAndChannel;
	
	//	print out any error
	std::string Error;
	if ( Job.mParams.GetParamAs( TJobParam::Param_Error, Error ) )
		std::Debug << Job.mParams.mCommand << " error: " << Error << std::endl;

	//	update feature tracker
	std::string Serial;
	if ( !Job.mParams.GetParamAs("serial", Serial ) )
	{
		std::Debug << "New interesting features missing serial, can't update feature tracker" << std::endl;
		std::Debug << Job.mParams << std::endl;
		return;
	}
	
	//	get feature tracker
	auto& FeatureTracker = mFeatureTrackers[Serial];
	
	std::Debug << Job.mParams.mCommand << " params: " << Job.mParams << std::endl;
	
	//	read features
	auto FeaturesParam = Job.mParams.GetDefaultParam();
	
	if ( FeaturesParam.GetFormat().HasContainer( Soy::StringToLower(Soy::GetTypeName<TFeatureMatchesAndImage>()) ) )
	{
		SoyData_Stack<TFeatureMatchesAndImage> MaiData;
		if ( !FeaturesParam.Decode( MaiData ) )
		{
			auto FeaturesAsString = FeaturesParam.Decode<std::string>();
			if ( FeaturesAsString.length() > 100 )
				FeaturesAsString = FeaturesAsString.substr( 0, 100 );
			std::Debug << "Failed to get features from " << Job.mParams.mCommand << " (" << FeaturesParam.GetFormat() << ") " << FeaturesAsString << std::endl;
			return;
		}
		
		//	do update which should trigger event
		auto Timecode = Job.mParams.GetParamAs<SoyTime>("timecode");
		auto Image = Job.mParams.GetParam("image");
		FeatureTracker.UpdateFeatures( GetArrayBridge(MaiData.mValue.mFeatureMatches), Timecode, MaiData.mValue.mImage );
	}
	else
	{
		//	explicit decode as it's not a known type
		SoyData_Stack<Array<TFeatureMatch>> FeaturesData;
		if ( !FeaturesParam.Decode( FeaturesData ) )
		{
			auto FeaturesAsString = FeaturesParam.Decode<std::string>();
			std::Debug << "Failed to get features from " << Job.mParams.mCommand << " (" << FeaturesParam.GetFormat() << ") " << FeaturesAsString << std::endl;
			return;
		}
	
		/*
		//	do update which should trigger event
		auto Timecode = Job.mParams.GetParamAs<SoyTime>("timecode");
		auto Image = Job.mParams.GetParam("image");
		FeatureTracker.UpdateFeatures( GetArrayBridge(FeaturesData.mValue), Timecode, Image );
		 */
	}
	
}

void TPopCameraTrack::OnFoundInterestingFeatures(TJobAndChannel& JobAndChannel)
{
	const TJob& Job = JobAndChannel;
	
	//	print out any error
	std::string Error;
	if ( Job.mParams.GetParamAs( TJobParam::Param_Error, Error ) )
		std::Debug << Job.mParams.mCommand << " error: " << Error << std::endl;

	//	update feature tracker
	std::string Serial;
	if ( !Job.mParams.GetParamAs("serial", Serial ) )
	{
		std::Debug << "New interesting features missing serial, can't update feature tracker" << std::endl;
		std::Debug << Job.mParams << std::endl;
		return;
	}
	
	//	get feature tracker
	auto& FeatureTracker = mFeatureTrackers[Serial];

	//	already have base features!
	if ( FeatureTracker.HasBaseFeatures() )
	{
		std::Debug << "FeatureTracker for " << Serial << " already has base features, ignored new interesting features" << std::endl;
		return;
	}
	
	//	read features
	auto FeaturesParam = Job.mParams.GetDefaultParam();
	//	explicit decode as it's not a known type
	
	if ( FeaturesParam.GetFormat().HasContainer( Soy::StringToLower(Soy::GetTypeName<TFeatureMatchesAndImage>()) ) )
	{
		SoyData_Stack<TFeatureMatchesAndImage> MaiData;
		if ( !FeaturesParam.Decode( MaiData ) )
		{
			auto FeaturesAsString = FeaturesParam.Decode<std::string>();
			if ( FeaturesAsString.length() > 100 )
				FeaturesAsString = FeaturesAsString.substr( 0, 100 );
			std::Debug << "Failed to get features from " << Job.mParams.mCommand << " (" << FeaturesParam.GetFormat() << ") " << FeaturesAsString << std::endl;
			return;
		}

		FeatureTracker.mBase.mFeatures = MaiData.mValue.mFeatureMatches;
		FeatureTracker.mBase.mLastImage = MaiData.mValue.mImage;
	}
	else
	{
		SoyData_Impl<Array<TFeatureMatch>> FeaturesData( FeatureTracker.mBase.mFeatures );
		if ( !FeaturesParam.Decode( FeaturesData ) )
		{
			auto FeaturesAsString = FeaturesParam.Decode<std::string>();
			std::Debug << "Failed to get features from " << Job.mParams.mCommand << " (" << FeaturesParam.GetFormat() << ") " << FeaturesAsString << std::endl;
			return;
		}
	}
	
	std::Debug << "got interesting features for " << Serial << std::endl;
}

void TPopCameraTrack::SubscribeNewCameraPose(TJobAndChannel& JobAndChannel)
{
	const TJob& Job = JobAndChannel;
	TJobReply Reply( JobAndChannel );
	
	std::stringstream Error;
	
	//	gr: determine if this already exists!
	auto EventName = Job.mParams.GetParamAs<std::string>("command");
	
#if defined(ENABLE_LSDSLAM)
	//	create new subscription for it
	auto Event = mSubcriberManager.AddEvent( mSlamOutput.mOnNewCameraPose, EventName, Error );
	if ( !Event )
	{
		std::stringstream ReplyError;
		ReplyError << "Failed to create new event " << EventName << ". " << Error.str();
		Reply.mParams.AddErrorParam( ReplyError.str() );
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	//	subscribe this caller
	if ( !Event->AddSubscriber( Job.mChannelMeta, Error ) )
	{
		std::stringstream ReplyError;
		ReplyError << "Failed to add subscriber to event " << EventName << ". " << Error.str();
		Reply.mParams.AddErrorParam( ReplyError.str() );
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	
	std::stringstream ReplyString;
	ReplyString << "OK subscribed to " << EventName;
	Reply.mParams.AddDefaultParam( ReplyString.str() );
#else
	
	Error << "Slam not enabled";

#endif
	
	if ( !Error.str().empty() )
		Reply.mParams.AddErrorParam( Error.str() );
	Reply.mParams.AddParam("eventcommand", EventName);
	
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}




bool TPopCameraTrack::OnNewFeatureStateCallback(TEventSubscriptionManager& SubscriptionManager,TJobChannelMeta Client,TTrackerState& State)
{
	TJob OutputJob;
	auto& Reply = OutputJob;

	//	send pixels
	Reply.mParams.AddParam("image", State.mLastImage);
	
	//	as json for now
	bool AsJson = true;
	
	auto& FeatureMatches = State.mFeatures;
	if ( AsJson )
	{
		//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
		std::shared_ptr<SoyData_Impl<json::Object>> FeatureMatchesJsonData( new SoyData_Stack<json::Object>() );
		if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
		{
			std::shared_ptr<SoyData> FeatureMatchesJsonDataGen( FeatureMatchesJsonData );
			Reply.mParams.AddDefaultParam( FeatureMatchesJsonDataGen );
		}
	}
	if ( !Reply.mParams.HasDefaultParam() )
		Reply.mParams.AddDefaultParam( FeatureMatches );
	
	if ( !SubscriptionManager.SendSubscriptionJob( Reply, Client ) )
		return false;
	
	return true;
}



void TPopCameraTrack::SubscribeNewFeatures(TJobAndChannel& JobAndChannel)
{
	const TJob& Job = JobAndChannel;
	TJobReply Reply( JobAndChannel );
	
	std::stringstream Error;

	//	get serial so we know what feature tracker to subscribe to
	std::string Serial;
	if ( !Job.mParams.GetParamAs("serial", Serial ) )
	{
		Reply.mParams.AddErrorParam("Couldn't decode serial");
		JobAndChannel.GetChannel().OnJobCompleted(Reply);
		return;
	}
	
	//	get/create feature tracker
	auto& FeatureTracker = mFeatureTrackers[Serial];
	
	//	gr: determine if this already exists!
	auto EventName = Job.mParams.GetParamAs<std::string>("command");
	
	//	create new subscription for it
	auto Event = mSubcriberManager.AddEvent( FeatureTracker.mOnNewState, EventName, Error );
	if ( !Event )
	{
		std::stringstream ReplyError;
		ReplyError << "Failed to create new event " << EventName << ". " << Error.str();
		Reply.mParams.AddErrorParam( ReplyError.str() );
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	//	make a lambda to recieve the event
	auto Client = Job.mChannelMeta;
	TEventSubscriptionCallback<TTrackerState> ListenerCallback = [this,Client](TEventSubscriptionManager& SubscriptionManager,TTrackerState& Value)
	{
		return this->OnNewFeatureStateCallback( SubscriptionManager, Client, Value );
	};
	
	//	subscribe this caller
	if ( !Event->AddSubscriber( Job.mChannelMeta, ListenerCallback, Error ) )
	{
		std::stringstream ReplyError;
		ReplyError << "Failed to add subscriber to event " << EventName << ". " << Error.str();
		Reply.mParams.AddErrorParam( ReplyError.str() );
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	
	std::stringstream ReplyString;
	ReplyString << "OK subscribed to " << EventName;
	Reply.mParams.AddDefaultParam( ReplyString.str() );

	
	if ( !Error.str().empty() )
		Reply.mParams.AddErrorParam( Error.str() );
	Reply.mParams.AddParam("eventcommand", EventName);
	
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}


bool TPopCameraTrack::UpdateSlam(SoyPixelsImpl& CameraPixels,std::stringstream& Error)
{
	if ( !Soy::Assert(CameraPixels.IsValid(),"Expected valid pixels") )
	{
		Error << "Invalid pixels";
		return false;
	}

#if defined(ENABLE_LSDSLAM)
	//	if slam already processing, drop this camera frame
	if ( !mSlamLock.try_lock() )
	{
		Error << "slam processing old frame, dropping frame" << std::endl;
		return false;
	}
	
	//	take another lock and unlock our try_lock so we don't ever lose the lock
	std::lock_guard<std::recursive_mutex> Lock(mSlamLock);
	mSlamLock.unlock();

	
	//	slam needs grey pixels as multiple of 16
	SoyPixels GreyPixels;
	GreyPixels.Copy( CameraPixels );
	if ( !GreyPixels.SetFormat( SoyPixelsFormat::Greyscale ) )
	{
		Error << "Failed to convert image to greyscale";
		return false;
	}
	int NewWidth = GreyPixels.GetWidth() - (GreyPixels.GetWidth() % 16);
	int NewHeight = GreyPixels.GetHeight() - (GreyPixels.GetHeight() % 16);
	GreyPixels.ResizeClip( NewWidth, NewHeight );
	auto& Pixels = GreyPixels;
		
	//	iphone 4/5 camera
	double f = 4.1;
	double resX = Pixels.GetWidth();
	double resY = Pixels.GetHeight();
	double sensorSizeX = 4.89;
	double sensorSizeY = 3.67;
	double fx = f * resX / sensorSizeX;
	double fy = f * resY / sensorSizeY;
	double cx = resX/2.;
	double cy = resY/2.;
	
	Eigen::Matrix3f CameraMatrix;
	CameraMatrix << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	
	bool EnableSlam = true;
	
	if ( !mSlam )
	{
		mSlam.reset( new lsd_slam::SlamSystem( Pixels.GetWidth(), Pixels.GetHeight(), CameraMatrix, EnableSlam ) );
		mSlam->setVisualization( &mSlamOutput );
		mSlamFrameCounter = 0;
	}
	
	uchar* PixelsArray = reinterpret_cast<uchar*>( GreyPixels.GetPixelsArray().GetArray() );
	
	
	bool Block = true;
	mSlamTimestamp = SoyTime(true).GetTime() / 1000.f;

	//	first run
	if ( mSlamFrameCounter == 0 )
	{
		mSlam->randomInit( PixelsArray, mSlamTimestamp, mSlamFrameCounter );
	}
	else
	{
		mSlam->trackFrame( PixelsArray, mSlamFrameCounter, Block, mSlamTimestamp );
	}
	mSlamFrameCounter++;
	
	return true;
#else
	Error << "Slam not enabled";
	return false;
#endif
}


class TChannelLiteral : public TChannel
{
public:
	TChannelLiteral(SoyRef ChannelRef) :
	TChannel	( ChannelRef )
	{
	}
	
	virtual void				GetClients(ArrayBridge<SoyRef>&& Clients)
	{

	}
	
	bool				FixParamFormat(TJobParam& Param,std::stringstream& Error) override
	{
		return true;
	}
	void		Execute(std::string Command)
	{
		TJobParams Params;
		Execute( Command, Params );
	}
	void		Execute(std::string Command,const TJobParams& Params)
	{
		auto& Channel = *this;
		TJob Job;
		Job.mParams = Params;
		Job.mParams.mCommand = Command;
		Job.mChannelMeta.mChannelRef = Channel.GetChannelRef();
		Job.mChannelMeta.mClientRef = SoyRef("x");
		
		//	send job to handler
		Channel.OnJobRecieved( Job );
	}
	
	//	we don't do anything, but to enable relay, we say it's "done"
	virtual bool				SendJobReply(const TJobReply& Job) override
	{
		OnJobSent( Job );
		return true;
	}
	virtual bool				SendCommandImpl(const TJob& Job) override
	{
		OnJobSent( Job );
		return true;
	}
};



//	horrible global for lambda
std::shared_ptr<TChannel> gStdioChannel;
std::shared_ptr<TChannel> gCaptureChannel;



TPopAppError::Type PopMain(TJobParams& Params)
{
	std::cout << Params << std::endl;
	
	TPopCameraTrack App;

	auto CommandLineChannel = std::shared_ptr<TChan<TChannelLiteral,TProtocolCli>>( new TChan<TChannelLiteral,TProtocolCli>( SoyRef("cmdline") ) );
	
	//	create stdio channel for commandline output
	auto StdioChannel = CreateChannelFromInputString("std:", SoyRef("stdio") );
	gStdioChannel = StdioChannel;
	auto HttpChannel = CreateChannelFromInputString("http:8080-8090", SoyRef("http") );
	auto WebSocketChannel = CreateChannelFromInputString("ws:json:9030-9039", SoyRef("websock") );
	//auto WebSocketChannel = CreateChannelFromInputString("ws:cli:9090-9099", SoyRef("websock") );
	auto SocksChannel = CreateChannelFromInputString("cli:7080-7089", SoyRef("socks") );
	App.mFeatureChannel = CreateChannelFromInputString("cli://localhost:7090", SoyRef("Feature") );
	
	App.AddChannel( CommandLineChannel );
	App.AddChannel( StdioChannel );
//	App.AddChannel( HttpChannel );
	App.AddChannel( WebSocketChannel );
	App.AddChannel( SocksChannel );
	App.AddChannel( App.mFeatureChannel );

	//	when the commandline SENDs a command (a reply), send it to stdout
	auto RelayFunc = [](TJobAndChannel& JobAndChannel)
	{
		if ( !gStdioChannel )
			return;
		TJob Job = JobAndChannel;
		Job.mChannelMeta.mChannelRef = gStdioChannel->GetChannelRef();
		Job.mChannelMeta.mClientRef = SoyRef();
		gStdioChannel->SendCommand( Job );
	};
	CommandLineChannel->mOnJobSent.AddListener( RelayFunc );
	
	//	connect to another app, and subscribe to frames
	bool CreateCaptureChannel = true;
	if ( CreateCaptureChannel )
	{
		auto CaptureChannel = CreateChannelFromInputString("cli://localhost:7070", SoyRef("capture") );
		gCaptureChannel = CaptureChannel;
		CaptureChannel->mOnJobRecieved.AddListener( RelayFunc );
		App.AddChannel( CaptureChannel );
		
		//	send commands from stdio to new channel
		auto SendToCaptureFunc = [](TJobAndChannel& JobAndChannel)
		{
			TJob Job = JobAndChannel;
			Job.mChannelMeta.mChannelRef = gStdioChannel->GetChannelRef();
			Job.mChannelMeta.mClientRef = SoyRef();
			gCaptureChannel->SendCommand( Job );
		};
		gStdioChannel->mOnJobRecieved.AddListener( SendToCaptureFunc );
		
		
		//	gr: need to do this in subscribe features!
		auto StartSubscription = [](TChannel& Channel)
		{
			TJob GetFrameJob;
			GetFrameJob.mChannelMeta.mChannelRef = Channel.GetChannelRef();
			GetFrameJob.mParams.mCommand = "subscribenewframe";
			GetFrameJob.mParams.AddParam("serial", "face" );
			GetFrameJob.mParams.AddParam("memfile", "1" );
			Channel.SendCommand( GetFrameJob );
		};
		
		CaptureChannel->mOnConnected.AddListener( StartSubscription );
	}
	
	//	run
	App.mConsoleApp.WaitForExit();

	gStdioChannel.reset();
	return TPopAppError::Success;
}




