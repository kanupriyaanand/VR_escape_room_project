using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System.Linq;
public class CheckPassword : MonoBehaviour
{

    public string theName;
    public InputField inputField;

    public Text textDisplay;
    
    public Button btn;

    public GameObject door; 
    public Collider doorCollider; 
    public float endGameXCoordinate = 8.95f;
    private bool isGameEnded = false;

    private void Start()
    {
        btn.onClick.AddListener(StoreName);
    }
    public void StoreName()
    {
        theName= inputField.text;
        if (theName=="1234")
        {Debug.Log("correct");
        textDisplay.text="Good job! Exit game through the open door.";
        RotateDoor();
        isGameEnded = true;
        
    }
    else 
    {
        Debug.Log("incorrect");
        textDisplay.text="Incorrect password! Try again!";
    }
}

private void RotateDoor()
    {
        // Add code to rotate the door (adjust the rotation angles as needed)
        door.transform.Rotate(new Vector3(0f, 90f, 0f), Space.Self);
    }
void Update()
    {
        // Find the player GameObject using its tag
        GameObject player = GameObject.FindGameObjectWithTag("Player");

        // Check if the player GameObject exists and its X coordinate is greater than or equal to the specified value
        if (player != null && player.transform.position.x >= endGameXCoordinate)
        {
            EndGame();
        }
    }
 private void EndGame()
    {
        Debug.Log("Game Ended");
        
        Application.Quit();
    }


}
 