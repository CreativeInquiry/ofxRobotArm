using System.Collections;
using System.Collections.Generic;
using UnityEngine;

 
 public class IgnoreCollision : MonoBehaviour
 {
 	public string objectToIgnore;
     void OnCollisionEnter(Collision collision)
    {
         if (collision.gameObject.tag == objectToIgnore)
        {
            Physics.IgnoreCollision(collision.gameObject.GetComponent<Collider>(), GetComponent<Collider>());
        }
    }

    void OnCollisionStay(Collision collision)
    {
    	     if (collision.gameObject.tag == objectToIgnore)
        {
            Physics.IgnoreCollision(collision.gameObject.GetComponent<Collider>(), GetComponent<Collider>());
        }
    }
 }