using System;
using UnityEngine;

public class NewBehaviourScript : MonoBehaviour
{
    private ROSInterface ros;
        
    void Start()
    {
        ros = GetComponent<ROSInterface>();
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
        ROSInterface.allocateStructArray(ref val4.poses, poses);
        ros.PublishROS(ref val4, "pose_array");
        ROSInterface.allocateStructArray(ref val4.poses, poses);
        var poses_arr = ROSInterface.getStructArray<geometry_msgs_Pose>(val4.poses);
      
        std_msgs_Float32MultiArray val2 = new std_msgs_Float32MultiArray();
        float[] data_vals = { 1, 2, 3 };
        ROSInterface.allocateFloatArray(ref val2.data, data_vals);
        std_msgs_MultiArrayDimension[] mulit = new std_msgs_MultiArrayDimension[2];
        ROSInterface.allocateString(ref mulit[0].label, "sad");
        mulit[0].size = 3;
        ROSInterface.allocateString(ref mulit[1].label, "no");
        ROSInterface.allocateStructArray(ref val2.layout.dim, mulit);
        ros.PublishROS(ref val2, "array");
      
        sensor_msgs_JointState val3 = new sensor_msgs_JointState();
        string[] name_arr = { "joint_1", "joint_2", "joint_3" };
        ROSInterface.allocateStringArray(ref val3.name, name_arr);
        double[] position_arr = { 1.1, 2.2, 3.3 };
        ROSInterface.allocateDoubleArray(ref val3.position, position_arr);
        ros.PublishROS(ref val3, "joint_state");

        sensor_msgs_JointState val33 = ros.ReceiveROS<sensor_msgs_JointState>("joint_state");
        var names = ROSInterface.getStringArray(val33.name);
        var positions = ROSInterface.getDoubleArray(val33.position);

        std_msgs_String val5 = ros.ReceiveROS<std_msgs_String>("chatter");
        var str = ROSInterface.getString(val5.data);
        Debug.Log(str);

        ROSInterface.FreeAll();
    }
}