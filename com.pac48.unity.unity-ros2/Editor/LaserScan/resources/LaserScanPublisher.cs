using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Burst;
using UnityEngine.Jobs;
using Unity.Jobs;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

[RequireComponent(typeof(RotateLidar))]
public class LaserScanPublisher : MonoBehaviour
{
    [SerializeField] public string _topicName = "scan";
    [SerializeField] public Transform scan_link;
    public float publishMessageFrequency = 1.0f/10.0f;

    public float angle_min;
    public float angle_max;
    public float angle_increment;
    public float time_increment;
    public float scan_time;
    public float range_min;
    public float range_max;
    private float timeElapsed;

    
    private float[] ranges;
    public float[] intensities;

    private sensor_msgs_LaserScan msg = new ();
    private JobHandle _handle;
    private float _timeElapsed = 0f;
    private double _timeStamp = 0f;

    private RotateLidar _lidar;

    float Deg2Rad(float deg)
    {
        return deg * Mathf.PI / 180f;
    }

    void Start()
    {
        // Get Rotate Lidar
        this._lidar = GetComponent<RotateLidar>();
        this._lidar.Init();

        angle_min = Deg2Rad(this._lidar.minAzimuthAngle);
        angle_max = Deg2Rad(this._lidar.maxAzimuthAngle);
        var deg_increment = (this._lidar.maxAzimuthAngle - this._lidar.minAzimuthAngle) /
                            (float)this._lidar.numOfIncrements;
        angle_increment = Deg2Rad(deg_increment);
        time_increment = 1f / this._lidar.scanRate / (float)this._lidar.numOfIncrements;
        scan_time = 1f / this._lidar.scanRate;
        range_min = this._lidar.minRange;
        range_max = this._lidar.maxRange;
        
        msg.header.frame_id = ROSInterface.AllocateString(scan_link.name);
        msg.angle_min = angle_min;
        msg.angle_max = angle_max;
        msg.angle_increment = angle_increment;
        msg.time_increment = time_increment;
        msg.scan_time = scan_time;
        msg.range_min = range_min;
        msg.range_max = range_max;
        ranges = new float[_lidar.numOfIncrements * _lidar.numOfLayers];
        intensities = new float[_lidar.numOfIncrements * _lidar.numOfLayers];
        msg.ranges = ROSInterface.AllocateFloatArray(ranges);
        msg.intensities = ROSInterface.AllocateFloatArray(intensities);
    }

    void OnDisable()
    {
        this._handle.Complete();
        this._lidar.Dispose();
    }


    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            _handle.Complete();
            // Update ROS Message

            ranges = this._lidar.distances.ToArray();
            intensities = this._lidar.intensities.ToArray();

            Marshal.Copy(ranges, 0, msg.ranges.ptr, (int) msg.ranges.length); 
            Marshal.Copy(intensities, 0, msg.intensities.ptr, (int) msg.intensities.length);
            ROSInterface.SetROSTime(ref msg.header.stamp);
            ROSInterface.PublishROS(ref msg, _topicName);
            ROSInterface.SendTransform(scan_link);
            
            // Update time
            timeElapsed = 0;
            this._timeStamp = Time.timeAsDouble;

            // Update Raycast Command
            for (int incr = 0; incr < this._lidar.numOfIncrements; incr++)
            {
                for (int layer = 0; layer < this._lidar.numOfLayers; layer++)
                {
                    int index = layer + incr * this._lidar.numOfLayers;
                    this._lidar.commands[index] =
                        new RaycastCommand(this.transform.position,
                            this.transform.rotation * this._lidar.commandDirVecs[index],
                            this._lidar.maxRange);
                }
            }

            // Update Parallel Jobs
            var raycastJobHandle = RaycastCommand.ScheduleBatch(this._lidar.commands, this._lidar.results, 360);
            // Update Distance data
            if (this._lidar.randomSeed++ == 0)
                this._lidar.randomSeed = 1;
            this._lidar.job.random.InitState(this._lidar.randomSeed);
            this._handle = this._lidar.job.Schedule(this._lidar.results.Length, 360, raycastJobHandle);
            JobHandle.ScheduleBatchedJobs();
        }
    }
}
// using RosMessageTypes.Sensor;