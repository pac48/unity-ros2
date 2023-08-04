using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SawyerController : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject baseLink;
    public GameObject l0;
    public GameObject l1;
    public GameObject l2;
    public GameObject l3;
    public GameObject l4;
    public GameObject l5;
    public GameObject l6;
    public GameObject head;

    void Start()
    {
        // Physics.IgnoreCollision(baseLink.GetComponent<Collider>(),l0.GetComponent<Collider>());
        // Physics.IgnoreCollision(l0.GetComponent<Collider>(),head.GetComponent<Collider>());
        // Physics.IgnoreCollision(l0.GetComponent<Collider>(),l1.GetComponent<Collider>());
        // Physics.IgnoreCollision(l1.GetComponent<Collider>(),l2.GetComponent<Collider>());
        // Physics.IgnoreCollision(l2.GetComponent<Collider>(),l3.GetComponent<Collider>());
        // Physics.IgnoreCollision(l3.GetComponent<Collider>(),l4.GetComponent<Collider>());
        // Physics.IgnoreCollision(l4.GetComponent<Collider>(),l5.GetComponent<Collider>());
        // Physics.IgnoreCollision(l5.GetComponent<Collider>(),l6.GetComponent<Collider>());
        //
        //
        // Physics.IgnoreCollision(l1.GetComponent<Collider>(),l0.GetComponent<Collider>());
        // Physics.IgnoreCollision(l2.GetComponent<Collider>(),l1.GetComponent<Collider>());
    }

    void OnCollisionEnter(Collision collision)
    {
        int o = 0;

    }

    // Update is called once per frame
    void Update()
    {

        
    }
}
