using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;


public struct TJobInterface
{
	public System.IntPtr	pJob;
	public System.IntPtr	sCommand;
	public System.IntPtr	sError;
	public UInt32			ParamCount;
	//public System.IntPtr[10]	sParamName;
};



public class PopJob
{
	public TJobInterface	mInterface;
	public String			Command = "";
	public String			Error = null;

	public PopJob(TJobInterface Interface)
	{
		//	gr: strings aren't converting
		mInterface = Interface;
		Command = Marshal.PtrToStringAuto( Interface.sCommand );
		Error = Marshal.PtrToStringAuto( Interface.sError );
	}

	public int GetParam(string Param,int DefaultValue)
	{
		return PopUnity.GetJobParam_int( ref mInterface, Param, DefaultValue );
	}

	public string GetParam(string Param,string DefaultValue)
	{
		System.IntPtr stringPtr = PopUnity.GetJobParam_string( ref mInterface, Param, DefaultValue );
		return Marshal.PtrToStringAuto( stringPtr );
	}

	public bool GetParam(string Param,Texture2D texture)
	{
		return PopUnity.GetJobParam_texture( ref mInterface, Param, texture.GetNativeTextureID() );
	}
}



public class PopUnity
{
	public delegate void TJobHandler(PopJob Job);
	private static Dictionary<string,TJobHandler> mJobHandlers = new Dictionary<string,TJobHandler>();

	public static void AssignJobHandler(string JobName,TJobHandler Delegate)
	{
		mJobHandlers[JobName] = Delegate;
	}

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	private delegate void DebugLogDelegate(string str);
	
	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	private delegate void OnJobDelegate(ref TJobInterface Job);

	[DllImport("PopUnity", CallingConvention = CallingConvention.Cdecl)]
	public static extern UInt64 CreateChannel(string ChannelSpec);

	[DllImport("PopUnity", CallingConvention = CallingConvention.Cdecl)]
	public static extern bool SendJob(UInt64 ChannelRef,string Command);

	[DllImport("PopUnity")]
	public static extern void FlushDebug (System.IntPtr FunctionPtr);
	
	[DllImport("PopUnity")]
	public static extern bool PopJob (System.IntPtr FunctionPtr);

	[DllImport("PopUnity", CallingConvention = CallingConvention.Cdecl)]
	public static extern int GetJobParam_int(ref TJobInterface JobInterface,string Param,int DefaultValue);
	
	[DllImport("PopUnity", CallingConvention = CallingConvention.Cdecl)]
	public static extern System.IntPtr GetJobParam_string(ref TJobInterface JobInterface,string Param,string DefaultValue);

	[DllImport("PopUnity", CallingConvention = CallingConvention.Cdecl)]
	public static extern bool GetJobParam_texture(ref TJobInterface JobInterface,string Param,int Texture);

	[DllImport("PopUnity")]
	private static extern void Cleanup ();

	static private DebugLogDelegate	mDebugLogDelegate = new DebugLogDelegate( Log );
	static private OnJobDelegate	mOnJobDelegate = new OnJobDelegate( OnJob );

	public PopUnity()
	{
#if UNITY_EDITOR
		UnityEditor.EditorApplication.playmodeStateChanged += OnAppStateChanged;
#endif
	}

	public void OnAppStateChanged()
	{
#if UNITY_EDITOR
		if (UnityEditor.EditorApplication.isPlayingOrWillChangePlaymode && UnityEditor.EditorApplication.isPlaying)
			Cleanup();
#endif
	}

	static void Log(string str)
	{
		UnityEngine.Debug.Log("PopUnity: " + str);
	}

	static void OnJob(ref TJobInterface JobInterface)
	{
		//	turn into the more clever c# class
		PopJob Job = new PopJob( JobInterface );
		UnityEngine.Debug.Log ("job! " + Job.Command );
		UnityEngine.Debug.Log ("(error: " + Job.Error);

		//	send job to handler
		try
		{
			TJobHandler Handler = mJobHandlers[Job.Command];
			Handler( Job );
		}
		catch ( KeyNotFoundException e )
		{
			UnityEngine.Debug.Log ("Unhandled job " + Job.Command);
		}
	}

	static public void Update()
	{
		FlushDebug (Marshal.GetFunctionPointerForDelegate (mDebugLogDelegate));

		//	pop all jobs
		bool More = PopJob (Marshal.GetFunctionPointerForDelegate (mOnJobDelegate));

	}
};
	

public class PopUnityChannel
{
	public UInt64	mChannel = 0;

	public PopUnityChannel(string Channel)
	{
		mChannel = PopUnity.CreateChannel(Channel);
	}

	public bool SendJob(string Command)
	{
		return PopUnity.SendJob (mChannel, Command);
	}
};


public class Debug : MonoBehaviour {

	private String	ServerAddress = "cli://localhost:7070";
	private PopUnityChannel	mChannel = null;
	private String	JobString = "subscribenewframe serial=isight memfile=false";
	public Texture2D mTexture;
	public Material MaterialForTexture;

	void Start()
	{
		//	need to CREATE a texture. overwriting an existing one doesn't work...
		if (mTexture == null) {
			mTexture = new Texture2D (1280, 1024, TextureFormat.BGRA32, false);

			//	need a material...
			if (MaterialForTexture != null)
				MaterialForTexture.mainTexture = mTexture;
		}

		PopUnity.AssignJobHandler("re:getframe", ((Job) => this.OnGetFrameReply(Job)) );
		PopUnity.AssignJobHandler("newframe", ((Job) => this.OnGetFrameReply(Job)) );
	}

	void Update () {
		PopUnity.Update ();
	}

	void OnGetFrameReply(PopJob Job)
	{
		Job.GetParam("default",mTexture);
	}

	Rect StepRect(Rect rect)
	{
		rect.y += rect.height + (rect.height*0.30f);
		return rect;
	}

	void OnGUI()
	{
		Rect rect = new Rect (10, 0, Screen.width - 20, 20);

		if (mChannel == null) {
			rect = StepRect (rect);
			ServerAddress = GUI.TextField (rect, ServerAddress);

			rect = StepRect (rect);
			if (GUI.Button (rect, "Connect to channel")) {
					mChannel = new PopUnityChannel (ServerAddress);
			}			

		} else {
			rect = StepRect (rect);
			GUI.Label (rect, "Channel id: " + mChannel.mChannel);

			rect = StepRect (rect);
			JobString = GUI.TextField (rect, JobString);

			rect = StepRect (rect);
			if (GUI.Button (rect, "Send job")) {
				mChannel.SendJob( JobString );
			}			
		}

		rect = StepRect (rect);
		//	fill
		rect.height = Screen.height - rect.y;

		//string Text = "Hello ";
		//GUI.Label (rect, Text);
		//GUI.DrawTexture (rect, mTexture);
	}

	void OnPostRender()
	{
		GL.IssuePluginEvent (0);
	}
}
