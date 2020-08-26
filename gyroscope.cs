using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Uduino;
using System;
public class gyroscope : MonoBehaviour
{
    GameObject cube;
    UduinoManager u;
    int xpos;
    int zpos;


    // Start is called before the first frame update
    void Start()
    {
        cube = GameObject.Find("Capsule");

        UduinoManager.Instance.OnDataReceived += ReadIMU;
        //UduinoManager.Instance.pinMode(AnalogPin.A0, PinMode.Input);
        //UduinoManager.Instance.pinMode(AnalogPin.A1, PinMode.Input);
    }

    // Update is called once per frame
    void Update()
    {

    }
    public void ReadIMU(string data, UduinoDevice device)
    {
        try
        {
            string[] values = data.Split('/');
            float x = float.Parse(values[0].Trim());
            float y = float.Parse(values[1].Trim());
            float z = float.Parse(values[2].Trim());
            Debug.Log(x + ", " + y + ", " + z);
            cube.transform.rotation = Quaternion.Lerp(cube.transform.localRotation, new Quaternion(x, y, z, 0), Time.deltaTime * 500f);
        }
        catch { }

    }

}
//joystick controls 
        //xpos = UduinoManager.Instance.analogRead(AnalogPin.A0, "PinRead");
       // zpos = UduinoManager.Instance.analogRead(AnalogPin.A1, "PinRead");
        /*
        if (xpos > 900)
        {
            //left
            cube.transform.Translate(Vector3.left * Time.deltaTime * 5);
            /*forward
            if (zpos> 650)
            {
                movingObject.transform.Translate(Vector3.forward * Time.deltaTime * 5);

            }   //back
            if (zpos< 400)
            {
                movingObject.transform.Translate(Vector3.back * Time.deltaTime * 5);

            }
            
        }

        if (xpos < 100)
        { //right

            cube.transform.Translate(Vector3.right * Time.deltaTime * 5);
            //forward
            /*
            if (zpos > 650)
                movingObject.transform.Translate(Vector3.forward * Time.deltaTime * 5);

            //back
            if (zpos < 400)
                movingObject.transform.Translate(Vector3.back * Time.deltaTime * 5);

            
        }

        if (zpos > 900)
        {    //forward
            cube.transform.Translate(Vector3.forward * Time.deltaTime * 5);
            //left
            /*
            if (xpos > 650)
                movingObject.transform.Translate(Vector3.left * Time.deltaTime * 5);

            //right
            if (xpos < 350)
                movingObject.transform.Translate(Vector3.right * Time.deltaTime * 5);
            
        }
        if (zpos < 100)
        {  //back
            cube.transform.Translate(Vector3.back * Time.deltaTime * 5);
            //left
            /*
            if (xpos > 650)
                movingObject.transform.Translate(Vector3.left * Time.deltaTime * 5);

            //right
            if (xpos< 350)
                movingObject.transform.Translate(Vector3.right * Time.deltaTime * 5);
        }
        

        }

        UduinoManager.Instance.SendBundle("PinRead");
    }
    */



    
