using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class GemCollect : MonoBehaviour
{
    public GameObject scoreBox;
    public AudioSource collectSound;

    void OnTriggerEnter()
    {   

        GlobalScore.currentScore+=1000;
        collectSound.Play();
        Destroy(gameObject);
    }
}
