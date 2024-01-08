using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class BookClue : MonoBehaviour
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
                
                messageText.text = "Count the number blue potion bottles in both rooms";}
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
