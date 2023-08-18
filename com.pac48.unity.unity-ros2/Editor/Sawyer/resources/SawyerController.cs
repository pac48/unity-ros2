using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SawyerController : MonoBehaviour
{
    public ArticulationBody articulatedBody;
    public string topicName = "/robot/limb/right/joint_command";
    public float UpdateFrequency = 1.0f / 60.0f;

    private Hashtable name2ind = new Hashtable();
    private Hashtable ind2name = new Hashtable();
    private List<float> positionsArr;
    private float timeElapsed;
    private int[] indexArr;
    private float[] command;
    private float[] posArr;

    void Start()
    {
        var indexes = new List<int>();
        foreach (var joint in articulatedBody.GetComponentsInChildren<ArticulationBody>())
        {
            indexes.Add(joint.index);
            name2ind.Add(joint.name, joint.index);
            ind2name.Add(joint.index, joint.name);
        }

        int offset = 0;
        if (!articulatedBody.immovable)
        {
            offset = 6;
        }

        indexArr = indexes.ToArray();
        command = new float[indexArr.Length - 1 + offset];
        var pos = new List<float>();
        articulatedBody.GetDriveTargets(pos);
        posArr = pos.ToArray();
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        for (int i = 0; i < command.Length; i++)
        {
            posArr[i] += command[i] * Time.deltaTime;
        }

        if (timeElapsed > UpdateFrequency)
        {
            var msg = ROSInterface.ReceiveROS<intera_core_msgs_JointCommand>(topicName);
            var velocityArr = ROSInterface.GetDoubleArray(msg.velocity);
            var positionArr = ROSInterface.GetDoubleArray(msg.position);
            var namesArr = ROSInterface.GetStringArray(msg.names);

            int offset = 0;
            if (!articulatedBody.immovable)
            {
                offset += 6;
            }

            for (int i = 0; i < namesArr.Length; i++)
            {
                var jointName = namesArr[i];
                if (name2ind.Contains(jointName))
                {
                    var tmp = (int)name2ind[jointName];
                    if (msg.mode == 2)
                    {
                        if (Math.Abs(velocityArr[i]) < 0.0001)
                        {
                            velocityArr[i] = 0;
                        }

                        command[tmp - 1 + offset] = (float)-velocityArr[i];
                    }
                    else if (msg.mode == 1)
                    {
                        command[tmp - 1 + offset] = 0;
                        posArr[tmp - 1 + offset] = (float)-positionArr[i];
                    }
                }
            }

            articulatedBody.SetDriveTargetVelocities(command.ToList());
            articulatedBody.SetDriveTargets(posArr.ToList());

            msg.Delete();
            timeElapsed = 0;
        }
    }
}