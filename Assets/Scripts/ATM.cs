using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class ATM : MonoBehaviour
{
    public GameObject atmScreen;
    public TextMeshProUGUI messageText;

    public int requiredPoints = 3000;

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            Debug.Log(GlobalScore.currentScore);
            if (GlobalScore.currentScore == 3000)
            {
                
                messageText.text = "Find a book in this room for your next clue";
            }
            else
            {   
                
                messageText.text = "Gain 3000 points through collectibles before you get your first clue";
            }

            LeanTween.scale(atmScreen, Vector3.one, 2);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            Debug.Log("Exited");
            LeanTween.scale(atmScreen, Vector3.zero, 2);
        }
    }
}