using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class SawyerJointStatePublisher : MonoBehaviour
{
    public ArticulationBody articulatedBody;
    public Transform base_link;
    public string topicName;
    public float publishMessageFrequency = 1.0f / 60.0f;

    private Hashtable name2ind = new Hashtable();
    private Hashtable ind2name = new Hashtable();
    private double[] positionsArr;
    private int[] indexArr;
    private sensor_msgs_JointState msg;
    private float timeElapsed;
    // Update is called once per frame

    void Start()
    {
        var linksArr = articulatedBody.GetComponentsInChildren<ArticulationBody>();
        var indexes = new List<int>();
        var names = new List<string>();
        foreach (var joint in linksArr)
        {
            if (joint.jointType == ArticulationJointType.PrismaticJoint ||
                joint.jointType == ArticulationJointType.RevoluteJoint)
            {
                names.Add(joint.name);
                indexes.Add(joint.index - 1);
            }
        }

        indexArr = indexes.ToArray();
        positionsArr = new double[indexArr.Length];
        msg = new sensor_msgs_JointState();
        msg.name = ROSInterface.AllocateStringArray(names.ToArray());
        msg.position = ROSInterface.AllocateDoubleArray(positionsArr);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            var positions = new List<float>();
            articulatedBody.GetJointPositions(positions);
            var posArr = positions.ToArray();
            int count = 0;
            int offset = 0;
            if (!articulatedBody.immovable)
            {
                offset += 5;
            }

            foreach (var ind in indexArr)
            {
                positionsArr[count] = -posArr[ind+offset];
                count++;
            }

            Marshal.Copy(positionsArr, 0, msg.position.ptr, (int)msg.position.length);
            ROSInterface.SetROSTime(ref msg.header.stamp);
            ROSInterface.PublishROS(ref msg, topicName);
            ROSInterface.SendTransform(base_link);

            timeElapsed = 0;
        }
    }
}