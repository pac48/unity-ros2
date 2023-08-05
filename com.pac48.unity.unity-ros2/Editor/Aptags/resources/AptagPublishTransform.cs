using System;
using UnityEngine;

public class AptagPublishTransform : MonoBehaviour
{
    public GameObject obj;
    public bool publish_loc_aptag_in_world = false;
    void Start()
    {
    }

    void Update()
    {   if (publish_loc_aptag_in_world){
            ROSInterface.SendTransform(obj.transform);
        }

    }
}