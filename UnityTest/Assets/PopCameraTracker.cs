using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System;

public class PopCameraTracker : MonoBehaviour {

	public string ServerAddress = "cli://localhost:7080";
	private PopUnityChannel	mChannel = null;
	private String	JobString = "subscribenewcamerapose";
	private Vector3 NewPosition;
	private Quaternion NewQuaternion;

	public void Start()
	{
		PopUnity.AssignJobHandler("newcamerapose", ((Job) => this.OnNewCameraPose(Job)) );
		NewPosition = transform.position;
		NewQuaternion = transform.rotation;
	}

	void Connect()
	{
		if ( mChannel != null )
			return;

		mChannel = new PopUnityChannel (ServerAddress);
		mChannel.SendJob( JobString );
	}

	void Update () {
		Connect ();
		PopUnity.Update ();
		transform.localPosition = NewPosition;
		transform.localRotation = NewQuaternion;
	}
	
	void OnNewCameraPose(PopJob Job)
	{
		float posx = Job.GetParam("posx",0.0f);
		float posy = Job.GetParam("posy",0.0f);
		float posz = Job.GetParam("posz",0.0f);
		float quatx = Job.GetParam("quatx",0.0f);
		float quaty = Job.GetParam("quaty",0.0f);
		float quatz = Job.GetParam("quatz",0.0f);
		float quatw = Job.GetParam("quatw",0.0f);
		float PosScalar = 10.0f;
		NewPosition.Set (posx*PosScalar, posy*PosScalar, posz*PosScalar);
		NewQuaternion.Set (quatx, quaty, quatz, quatw);
	}
	
	void OnGUI()
	{
		if (GUI.Button (new Rect (10, 10, 100, 20), "Reset slam"))
			mChannel.SendJob ("resetslam");
	}
	
	void OnPostRender()
	{
		GL.IssuePluginEvent (0);
	}
}