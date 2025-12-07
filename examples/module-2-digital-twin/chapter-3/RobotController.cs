using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

/// <summary>
/// Unity script for controlling a robot via ROS 2 cmd_vel messages.
/// Attach to robot GameObject with Rigidbody component.
/// </summary>
public class RobotController : MonoBehaviour
{
    private ROSConnection ros;
    private Rigidbody rb;

    [Header("ROS Settings")]
    public string cmdVelTopic = "cmd_vel";
    public float linearSpeed = 1.0f;
    public float angularSpeed = 1.0f;

    void Start()
    {
        // Connect to ROS TCP Endpoint
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveCmdVel);

        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("RobotController requires Rigidbody component!");
        }
    }

    void ReceiveCmdVel(TwistMsg msg)
    {
        float linear = (float)msg.linear.x * linearSpeed;
        float angular = (float)msg.angular.z * angularSpeed;

        // Apply velocities to Rigidbody
        Vector3 movement = transform.forward * linear * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + movement);

        Quaternion rotation = Quaternion.Euler(0, angular * Time.fixedDeltaTime * Mathf.Rad2Deg, 0);
        rb.MoveRotation(rb.rotation * rotation);
    }
}
