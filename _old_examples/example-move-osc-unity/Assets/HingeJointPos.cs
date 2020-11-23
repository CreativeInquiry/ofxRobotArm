using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HingeJointPos : MonoBehaviour
{
    public OSC osc;
	public HingeJoint[] joints;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
         OscMessage message = new OscMessage();
         message.address = "/joints";
        for(int i = 0; i < joints.Length; i++){
            if(i == 1 || i == 3){
              message.values.Add(Mathf.Deg2Rad*(-90f+joints[i].angle)); 
            }else{
              message.values.Add(Mathf.Deg2Rad*(joints[i].angle));
            }
            
        }
        osc.Send(message);
    }
}

