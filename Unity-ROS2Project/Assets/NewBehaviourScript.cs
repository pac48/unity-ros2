using System;
using UnityEngine;

public class NewBehaviourScript : MonoBehaviour
{
    public GameObject obj;
    void Start()
    {
    }

    void Update()
    {
        geometry_msgs_PoseArray val4 = new geometry_msgs_PoseArray();
        geometry_msgs_Pose[] poses = new geometry_msgs_Pose[2];
        poses[0].position.x = 3.1;
        poses[0].position.y = 2.2;
        poses[0].position.z = 1.3;
        poses[1].position.x = 1.1;
        poses[1].position.y = 2.2;
        poses[1].position.z = 3.3;
        val4.poses = ROSInterface.AllocateStructArray(poses);
        ROSInterface.PublishROS(ref val4, "pose_array");
        val4.poses = ROSInterface.AllocateStructArray(poses);
        var poses_arr = ROSInterface.GetStructArray<geometry_msgs_Pose>(val4.poses);

        std_msgs_Float32MultiArray val2 = new std_msgs_Float32MultiArray();
        float[] data_vals = { 1, 2, 3 };
        val2.data = ROSInterface.AllocateFloatArray(data_vals);
        std_msgs_MultiArrayDimension[] mulit = new std_msgs_MultiArrayDimension[2];
        mulit[0].label = ROSInterface.AllocateString("sad");
        mulit[0].size = 3;
        mulit[1].label = ROSInterface.AllocateString("no");
        val2.layout.dim = ROSInterface.AllocateStructArray(mulit);
        ROSInterface.PublishROS(ref val2, "array");

        sensor_msgs_JointState val3 = new sensor_msgs_JointState();
        string[] name_arr = { "joint_1", "joint_2", "joint_3" };
        val3.name = ROSInterface.AllocateStringArray(name_arr);
        double[] position_arr = { 1.1, 2.2, 3.3 };
        val3.position = ROSInterface.AllocateDoubleArray(position_arr);
        ROSInterface.PublishROS(ref val3, "joint_state");

        sensor_msgs_JointState val33 = ROSInterface.ReceiveROS<sensor_msgs_JointState>("joint_state");
        var names = ROSInterface.GetStringArray(val33.name);
        var positions = ROSInterface.GetDoubleArray(val33.position);

        std_msgs_String val5 = ROSInterface.ReceiveROS<std_msgs_String>("chatter");
        var str = ROSInterface.GetString(val5.data);
        // Debug.Log(str);
        
        val2.Delete();
        val3.Delete();
        val33.Delete();
        val4.Delete();
        val5.Delete();
    }
}