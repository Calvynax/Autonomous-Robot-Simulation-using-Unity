using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider WCFrontLeft;
    [SerializeField] private WheelCollider WCFrontRight;
    [SerializeField] private WheelCollider WCBackLeft;
    [SerializeField] private WheelCollider WCBackRight;

    [SerializeField] private Transform WTFrontLeft;
    [SerializeField] private Transform WTFrontRight;
    [SerializeField] private Transform WTBackLeft;
    [SerializeField] private Transform WTBackRight;

    [SerializeField] private Transform RCFR;
    [SerializeField] private Transform RCL1;
    [SerializeField] private Transform RCL2;
    [SerializeField] private Transform RCL3;
    [SerializeField] private Transform RCR1;
    [SerializeField] private Transform RCR2;
    [SerializeField] private Transform RCR3;
    [SerializeField] private Transform AGOR;

    [SerializeField] private float maxSteeringAngle = 30;
    [SerializeField] private float motorForce = 50;
    [SerializeField] private float brakeForce;
    private Rigidbody rb;
    [SerializeField] private float angle_x;
    [SerializeField] private float angle_z;
    [SerializeField] private float CarVelocity;

    private float steerAngle;
    private bool isBreaking;

    private float s1dist = 5;
    private float s2dist = 6;
    private float s3dist = 8;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        float s1x = 0; float s1y = 10; float s1z = 0;
        float s2x = 8; float s2y = 30; float s2z = 0;
        float s3x = 16; float s3y = 60; float s3z = 0;

        PositionSensors(RCFR, 20, 0, 0);
        PositionSensors(RCL1, s1x, -s1y, s1z);
        PositionSensors(RCL2, s2x, -s2y, s2z);
        PositionSensors(RCL3, s3x, -s3y, s3z);
        PositionSensors(RCR1, s1x, s1y, s1z);
        PositionSensors(RCR2, s2x, s2y, s2z);
        PositionSensors(RCR3, s3x, s3y, s3z);
        PositionSensors(AGOR, 50, 180, 0);
    }

   

    private void PositionSensors(Transform sensor, float x_angle, float y_angle, float z_angle)
    {
        sensor.transform.Rotate(x_angle, y_angle, z_angle);
    }

    private void MotorPower()
    {
        float currentAcceleration;

        currentAcceleration = isBreaking ? 0 : motorForce;
        ApplyMotorTorque(currentAcceleration);
        brakeForce = isBreaking ? 3000f : 0f;
        ApplyBrakeTorque(brakeForce);
    }

    private void ApplyMotorTorque(float torque)
    {
        WCFrontRight.motorTorque = torque;
        WCFrontLeft.motorTorque = torque;
        WCBackLeft.motorTorque = torque;
        WCBackRight.motorTorque = torque;
    }

    private void ApplyBrakeTorque(float torque)
    {
        WCFrontLeft.brakeTorque = torque;
        WCFrontRight.brakeTorque = torque;
        WCBackLeft.brakeTorque = torque;
        WCBackRight.brakeTorque = torque;
    }

    private void UpdateWheels()
    {
        UpdateWheelsPosition(WCFrontLeft, WTFrontLeft);
        UpdateWheelsPosition(WCFrontRight, WTFrontRight);
        UpdateWheelsPosition(WCBackLeft, WTBackLeft);
        UpdateWheelsPosition(WCBackRight, WTBackRight);
    }

    private void UpdateWheelsPosition(WheelCollider collider, Transform transform)
    {
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
        transform.position = position;
        transform.rotation = rotation;
    }

    private void SteeringControl(float direction)
    {
        steerAngle = direction * maxSteeringAngle;
        WCFrontLeft.steerAngle = steerAngle;
        WCFrontRight.steerAngle = steerAngle;
    }

    private bool Sense(Transform sensor, float dist, string layer)
    {
        LayerMask mask = LayerMask.GetMask(layer);
        if (Physics.Raycast(sensor.position, sensor.TransformDirection(Vector3.forward), dist, mask))
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * dist, Color.yellow);
            return true;
        }
        else
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * dist, Color.white);
            return false;
        }
    }

    private void MaintainTrack()
    {
        if (!Sense(RCL3, s3dist, "Road") || !Sense(RCR3, s3dist, "Road"))
        {
            if (!Sense(RCL3, s3dist, "Road"))
            {
                SteeringControl(1);
            }
            if (!Sense(RCR3, s3dist, "Road"))
            {
                SteeringControl(-1);
            }
        }
        else
        {
            SteeringControl(0);
        }
    }

    private void SpeedControl()
    {
        if (CarVelocity < 2 && motorForce <= 50)
        {
            motorForce = motorForce + 540f;
        }
        else if (CarVelocity > 6 && motorForce > 0)
        {
            motorForce = motorForce - 30f;
        }
        else if (CarVelocity > 12 && motorForce < 0)
        {
            motorForce = motorForce - 80f;
        }
    }

    private void ObstaclesAvoidance()
    {
        if (Sense(RCL1, s1dist, "Obs"))
        {
            SteeringControl(1);
        }
        if (Sense(RCR1, s1dist, "Obs"))
        {
            SteeringControl(-1);
        }

        if (Sense(RCL2, s2dist, "Obs"))
        {
            SteeringControl(1);
        }
        if (Sense(RCR2, s2dist, "Obs"))
        {
            SteeringControl(-1);
        }
    }

    private void FixedUpdate()
    {
        MaintainTrack();
        ObstaclesAvoidance();
        SpeedControl();
        MotorPower();
        UpdateWheels();

        angle_x = AGOR.eulerAngles.x;
        angle_z = AGOR.eulerAngles.z;

        CarVelocity = rb.velocity.magnitude;
    }
}