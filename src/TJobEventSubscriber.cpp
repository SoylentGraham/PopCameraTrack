#include "TJobEventSubscriber.h"
#include <TChannel.h>
#include "PopCameraTrack.h"



bool TEventSubscriptionManager::SendSubscriptionJob(TJob& Job,TJobChannelMeta Client )
{
	//	modify job
	Soy::Assert( Job.mParams.mCommand.empty(), "Expected job not to have a command yet" );
	Job.mParams.mCommand = mEventName;
	Job.mChannelMeta = Client;
	
	//	get channel
	auto Channel = mParent.GetChannel( Client.mChannelRef );
	if ( !Channel )
	{
		std::Debug << "Unknown channel for " << Client << std::endl;
		return false;
	}
	
	//	send to client
	if ( !Channel->SendCommand( Job ) )
	{
	//	std::Debug << "Failed to send job " << Job.mParams.mCommand << " to " << Client << std::endl;
		return false;
	}
	
	//std::Debug << "Sending subscription job(" << Job.mParams.mCommand << ") to " << Client << "; " << Job.mParams << std::endl;
	
	return false;
}


std::shared_ptr<TChannel> TSubscriberManager::GetChannel(SoyRef Channel)
{
	auto pChannel = mChannelManager.GetChannel( Channel );
	return pChannel;
}

