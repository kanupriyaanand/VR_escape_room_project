using System.Collections;
using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class ATMRoom2 : MonoBehaviour
{
    public GameObject atmScreen;
    public TextMeshProUGUI screenText;
    public TextMeshProUGUI messageText;

    private string passcode = "It starts with 1"; // Replace with your desired passcode

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            Debug.Log("Entered");
            screenText.text = "How many blue potion bottles did you find?";
            messageText.text = "Enter the correct number:";
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

    public void Update()
    {
        // Check for user input continuously
        CheckUserInput();
    }

    private void CheckUserInput()
    {
        // Check if the player pressed the "4" key
        if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            // Display the message for the correct answer
            screenText.text = "It starts with 1";
            Debug.Log("correct!");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha0) || Input.GetKeyDown(KeyCode.Alpha1) || Input.GetKeyDown(KeyCode.Alpha2) || Input.GetKeyDown(KeyCode.Alpha3) ||
                 Input.GetKeyDown(KeyCode.Alpha5) || Input.GetKeyDown(KeyCode.Alpha6) || Input.GetKeyDown(KeyCode.Alpha7) || Input.GetKeyDown(KeyCode.Alpha8) || Input.GetKeyDown(KeyCode.Alpha9))
        {
            // Display the message for an incorrect answer
            screenText.text = "Wrong answer, try again";
            Debug.Log("try again sis");
        }
    }
}
