﻿//
// Mecanimのアニメーションデータが、原点で移動しない場合の Rigidbody付きコントローラ
// サンプル
// 2014/03/13 N.Kobyasahi
//
using UnityEngine;
using System.Collections;
using running;
namespace UnityChan
{
// 必要なコンポーネントの列記
	[RequireComponent(typeof(Animator))]
	[RequireComponent(typeof(CapsuleCollider))]
	[RequireComponent(typeof(Rigidbody))]

	public class UnityChanControlScriptWithRgidBody : MonoBehaviour
	{

		public float animSpeed = 1.5f;				// アニメーション再生速度設定
		public float lookSmoother = 3.0f;			// a smoothing setting for camera motion
		public bool useCurves = true;				// Mecanimでカーブ調整を使うか設定する
		// このスイッチが入っていないとカーブは使われない
		public float useCurvesHeight = 0.5f;		// カーブ補正の有効高さ（地面をすり抜けやすい時には大きくする）

		// 以下キャラクターコントローラ用パラメタ
		// 前進速度
		public float forwardSpeed = 7.0f;
		// 後退速度
		public float backwardSpeed = 2.0f;
		// 旋回速度
		public float rotateSpeed = 2.0f;
		// ジャンプ威力
		public float jumpPower = 2.0f; 
		// キャラクターコントローラ（カプセルコライダ）の参照
		private CapsuleCollider col;
		private Rigidbody rb;
		// キャラクターコントローラ（カプセルコライダ）の移動量
		private Vector3 velocity;
		// CapsuleColliderで設定されているコライダのHeiht、Centerの初期値を収める変数
		private float orgColHight;
		private Vector3 orgVectColCenter;
		private Animator anim;							// キャラにアタッチされるアニメーターへの参照
		private AnimatorStateInfo currentBaseState;			// base layerで使われる、アニメーターの現在の状態の参照

		private GameObject cameraObject;	// メインカメラへの参照
		
		// アニメーター各ステートへの参照
		static int idleState = Animator.StringToHash ("Base Layer.Idle");
		static int locoState = Animator.StringToHash ("Base Layer.Locomotion");
		static int jumpState = Animator.StringToHash ("Base Layer.Jump");
		static int restState = Animator.StringToHash ("Base Layer.Rest");

		// 初期化
		void Start ()
		{
			// Animatorコンポーネントを取得する
			anim = GetComponent<Animator> ();
			// CapsuleColliderコンポーネントを取得する（カプセル型コリジョン）
			col = GetComponent<CapsuleCollider> ();
			rb = GetComponent<Rigidbody> ();
			//メインカメラを取得する
			cameraObject = GameObject.FindWithTag ("MainCamera");
			// CapsuleColliderコンポーネントのHeight、Centerの初期値を保存する
			orgColHight = col.height;
			orgVectColCenter = col.center;
		}
	
	
		// 以下、メイン処理.リジッドボディと絡めるので、FixedUpdate内で処理を行う.
		void FixedUpdate ()
		{
			float h = Input.GetAxis ("Horizontal");				// 入力デバイスの水平軸をhで定義
			float v = Input.GetAxis ("Vertical");				// 入力デバイスの垂直軸をvで定義
			anim.SetFloat ("Speed", v);							// Animator側で設定している"Speed"パラメタにvを渡す
			anim.SetFloat ("Direction", h); 						// Animator側で設定している"Direction"パラメタにhを渡す
			anim.speed = animSpeed;								// Animatorのモーション再生速度に animSpeedを設定する
			currentBaseState = anim.GetCurrentAnimatorStateInfo (0);	// 参照用のステート変数にBase Layer (0)の現在のステートを設定する
			rb.useGravity = true;//ジャンプ中に重力を切るので、それ以外は重力の影響を受けるようにする
		
		
		    int receivedData;
			if (int.TryParse(TcpListenerManager.dataReceived, out receivedData) && receivedData >= 30 && receivedData <= 50)
				{
					// If so, multiply the value by 0.1 and assign it to v for walking speed
					v = receivedData * 0.06f;
				}
			// 以下、キャラクターの移動処理
			velocity = new Vector3 (0, 0, v);		// 上下のキー入力からZ軸方向の移動量を取得
			// キャラクターのローカル空間での方向に変換
			velocity = transform.TransformDirection (velocity);
			//以下のvの閾値は、Mecanim側のトランジションと一緒に調整する
			if (v > 0.1) {
				velocity *= forwardSpeed;
				Debug.Log("walking");		// 移動速度を掛ける
			} else if (v < -0.1) {
				velocity *= backwardSpeed;	// 移動速度を掛ける
			}
		
			if (Input.GetButtonDown ("Jump")||  (TcpListenerManager.dataReceived=="150" && !anim.GetBool("Jump") ) ) {	// スペースキーを入力したら

				Debug.Log("Jump condition met");

    // Simulate pressing the space bar for 1 second
    float simulatedJumpDuration = 1.0f;  // Adjust the duration as needed
    float simulatedJumpForce = jumpPower / simulatedJumpDuration;

    rb.AddForce(Vector3.up * simulatedJumpForce, ForceMode.Impulse);
    anim.SetBool("Jump", true);

    // Use a coroutine to reset the Jump state after the simulated duration
    StartCoroutine(ResetJumpState(simulatedJumpDuration));
	TcpListenerManager.dataReceived="0";
						// Debug.Log("this one");
						// rb.AddForce (Vector3.up * jumpPower, ForceMode.Impulse);
						
						// anim.SetBool ("Jump", true);
					
			}
		

			
			transform.localPosition += velocity * Time.fixedDeltaTime;

			// 左右のキー入力でキャラクタをY軸で旋回させる
			transform.Rotate (0, h * rotateSpeed, 0);	
	

			// 以下、Animatorの各ステート中での処理
			// Locomotion中
			// 現在のベースレイヤーがlocoStateの時
			if (currentBaseState.nameHash == locoState) {
				//カーブでコライダ調整をしている時は、念のためにリセットする
				if (useCurves) {
					resetCollider ();
				}
			}
		// JUMP中の処理
		// 現在のベースレイヤーがjumpStateの時
		else if (currentBaseState.nameHash == jumpState ) {
				//cameraObject.SendMessage ("setCameraPositionJumpView");	// ジャンプ中のカメラに変更
				// ステートがトランジション中でない場合
				if (!anim.IsInTransition (0)) {
				
					// 以下、カーブ調整をする場合の処理
					if (useCurves) {
					
						float jumpHeight = anim.GetFloat ("JumpHeight");
						//Debug.Log("jumpheight"+jumpHeight);
						float gravityControl = anim.GetFloat ("GravityControl"); 
						if (gravityControl > 0)
							rb.useGravity = false;	//ジャンプ中の重力の影響を切る
					    //Debug.Log("gravity"+ gravityControl);
						// レイキャストをキャラクターのセンターから落とす
						Ray ray = new Ray (transform.position + Vector3.up*0.5f, -Vector3.up);
						Debug.Log("chan jumped");
						RaycastHit hitInfo = new RaycastHit ();
						Debug.Log(hitInfo.distance);
						if (Physics.Raycast (ray, out hitInfo)) {
							if (hitInfo.distance > useCurvesHeight) {
								col.height = orgColHight - jumpHeight;			// 調整されたコライダーの高さ
								float adjCenterY = orgVectColCenter.y + jumpHeight;
								col.center = new Vector3 (0, adjCenterY, 0);	
								// 調整されたコライダーのセンター
							} else {
								// 閾値よりも低い時には初期値に戻す（念のため）					
								resetCollider ();
							}
						}
					}
					// Jump bool値をリセットする（ループしないようにする）				
					anim.SetBool ("Jump", false);
				}
			}
		// IDLE中の処理
		// 現在のベースレイヤーがidleStateの時
		else if (currentBaseState.nameHash == idleState) {
				//カーブでコライダ調整をしている時は、念のためにリセットする
				if (useCurves) {
					resetCollider ();
				}
				// スペースキーを入力したらRest状態になる
				if (Input.GetButtonDown ("Jump")) {
					anim.SetBool ("Rest", true);
				}
			}
		// REST中の処理
		// 現在のベースレイヤーがrestStateの時
		else if (currentBaseState.nameHash == restState) {
				//cameraObject.SendMessage("setCameraPositionFrontView");		// カメラを正面に切り替える
				// ステートが遷移中でない場合、Rest bool値をリセットする（ループしないようにする）
				if (!anim.IsInTransition (0)) {
					anim.SetBool ("Rest", false);
				}
			}
		}

		// void OnGUI ()
		// {
		// 	GUI.Box (new Rect (Screen.width - 260, 10, 250, 150), "Interaction");
		// 	GUI.Label (new Rect (Screen.width - 245, 30, 250, 30), "Up/Down Arrow : Go Forwald/Go Back");
		// 	GUI.Label (new Rect (Screen.width - 245, 50, 250, 30), "Left/Right Arrow : Turn Left/Turn Right");
		// 	GUI.Label (new Rect (Screen.width - 245, 70, 250, 30), "Hit Space key while Running : Jump");
		// 	GUI.Label (new Rect (Screen.width - 245, 90, 250, 30), "Hit Spase key while Stopping : Rest");
		// 	GUI.Label (new Rect (Screen.width - 245, 110, 250, 30), "Left Control : Front Camera");
		// 	GUI.Label (new Rect (Screen.width - 245, 130, 250, 30), "Alt : LookAt Camera");
		// }


	
		void resetCollider ()
		{
			// コンポーネントのHeight、Centerの初期値を戻す
			col.height = orgColHight;
			col.center = orgVectColCenter;
		}

IEnumerator ResetJumpState(float delay)
{
    yield return new WaitForSeconds(delay);

    // Reset the Jump state after the simulated duration
    anim.SetBool("Jump", false);
}
	}
}