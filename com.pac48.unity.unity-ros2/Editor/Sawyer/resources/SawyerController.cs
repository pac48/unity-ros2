using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SawyerController : MonoBehaviour
{
    public ArticulationBody articulatedBody;
    public string topicName;

    private Hashtable name2ind;
    private Hashtable ind2name;
    private List<float> positions;
    private sensor_msgs_JointState msg;

    void Start()
    {
        foreach (var joint in articulatedBody.GetComponentsInChildren<ArticulationBody>())
        {
            name2ind.Add(joint.name, joint.index);
            ind2name.Add(joint.index, joint.name);
        }

        msg = new sensor_msgs_JointState();
        // msg.
    }

    // Update is called once per frame
    void Update()
    {
        articulatedBody.GetJointPositions(positions);
        int ind = 0;
        foreach (var angle in positions)
        {
            
            ind++;
        }
        
        ROSInterface.SetROSTime(ref msg.header.stamp);
        ROSInterface.PublishROS(ref msg, topicName);
    }
}