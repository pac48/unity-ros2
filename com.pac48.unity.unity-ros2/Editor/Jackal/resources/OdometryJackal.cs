using System;
using System.Runtime.InteropServices;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class OdometryJackal : MonoBehaviour
{
    public Transform odom_transform;
    public float publishMessageFrequency = 1.0f / 60.0f;
    public string topicName = "odom";

    private nav_msgs_Odometry msg;
    private Vector3 prev_forward;
    private Vector3 prev_position;
    private float timeElapsed;

    void Start()
    {
        prev_forward = new Vector3();
        prev_position = new Vector3();
        prev_position = odom_transform.position;
        prev_forward = odom_transform.forward;

        msg = new nav_msgs_Odometry();
        msg.header.frame_id = ROSInterface.AllocateString("odom");
        msg.child_frame_id = Marshal.StringToHGlobalAnsi(odom_transform.name);
        msg.pose.covariance = new double[36];
        msg.pose.covariance[0] = 0.1;
        msg.pose.covariance[7] = 0.1;
        msg.pose.covariance[14] = 1000000000000.0;
        msg.pose.covariance[21] = 1000000000000.0;
        msg.pose.covariance[28] = 1000000000000.0;
        msg.pose.covariance[35] = 1000000000000.0;
        msg.twist.covariance = new double[36];
        msg.twist.covariance[0] = 1000000000000.0;
        msg.twist.covariance[7] = 1000000000000.0;
        msg.twist.covariance[14] = 1000000000000.0;
        msg.twist.covariance[21] = 1000000000000.0;
        msg.twist.covariance[28] = 1000000000000.0;
        msg.twist.covariance[35] = 1000000000000.0;
    }

    void FixedUpdate()
    {
        var velocity = (odom_transform.position - prev_position) / Time.deltaTime;
        var tmp = Vector3.Dot(odom_transform.forward, prev_forward);
        tmp = Math.Max(tmp, 0.0f);
        tmp = Math.Min(tmp, 1.0f);
        var omega = Math.Acos(tmp) / Time.deltaTime;
        var angular_velocity = Vector3.Cross(odom_transform.forward, prev_forward);
        angular_velocity.Normalize();

        Vector3 tmp2 = new Vector3();
        tmp2.x = Vector3.Dot(odom_transform.right, velocity);
        tmp2.y = Vector3.Dot(odom_transform.up, velocity);
        tmp2.z = Vector3.Dot(odom_transform.forward, velocity);
        var flu = tmp2.To<FLU>();
        msg.twist.twist.linear.x = flu.x;
        msg.twist.twist.linear.y = flu.y;
        msg.twist.twist.linear.z = flu.z;

        Vector3 tmp3 = new Vector3();
        tmp3.x = (float)omega * angular_velocity.x;
        tmp3.y = (float)omega * angular_velocity.y;
        tmp3.z = (float)omega * angular_velocity.z;
        flu = tmp3.To<FLU>();
        msg.twist.twist.angular.x = flu.x;
        msg.twist.twist.angular.y = flu.y;
        msg.twist.twist.angular.z = flu.z;

        prev_position = odom_transform.position;
        prev_forward = odom_transform.forward;
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            var pos = odom_transform.position.To<FLU>();
            msg.pose.pose.position.x = pos.x;
            msg.pose.pose.position.y = pos.y;
            msg.pose.pose.position.z = pos.z;
            var rot = odom_transform.rotation.To<FLU>();
            msg.pose.pose.orientation.w = rot.w;
            msg.pose.pose.orientation.x = rot.x;
            msg.pose.pose.orientation.y = rot.y;
            msg.pose.pose.orientation.z = rot.z;
            ROSInterface.SendTransform(odom_transform);
            ROSInterface.SetROSTime(ref msg.header.stamp);
            ROSInterface.PublishROS(ref msg, topicName);
            
            geometry_msgs_TransformStamped tf = new geometry_msgs_TransformStamped();
            tf.transform.rotation.w = 1.0;
            tf.child_frame_id = ROSInterface.AllocateString("unity");
            tf.header.frame_id = ROSInterface.AllocateString("odom");
            ROSInterface.SendTransform(tf);
            tf.Delete();
            
            timeElapsed = 0;
        }
    }
}