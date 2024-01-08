using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class MorsePassword : MonoBehaviour
{
    public GameObject bookPage;
    public TextMeshProUGUI messageText;

    public int requiredPoints = 3000;

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            Debug.Log(GlobalScore.currentScore);
            if (GlobalScore.currentScore == 3000)
            {
                
                messageText.text = "The rest of the passcode is: ..---  ...--  ....-";}
            else
            {   
                
                messageText.text = "Check ATM machine";
            }

            LeanTween.scale(bookPage, Vector3.one, 2);
        }
    }
    
    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            Debug.Log("Exited");
            LeanTween.scale(bookPage, Vector3.zero, 2);
        }
    }
}
