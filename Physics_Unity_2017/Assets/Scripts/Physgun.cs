using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Physgun : MonoBehaviour
{
    public enum GunState
    {
        INACTIVE,
        SEARCHING,
        CONNECTED
    }

    public Transform gunEnd = null;
    public Transform beamEnd = null;
    public LineRenderer lineRenderer = null;
    public LayerMask layerMask = new LayerMask();

    GunState currentState = GunState.INACTIVE;

    Rigidbody connectedBody = null;
    RaycastHit currentHit;
    public ConfigurableJoint joint = null;
    public int connectedSolverCount = 10;
    bool connectedCache_Gravity = false;
    int connectedCache_SolverCount = 0;
    RigidbodyInterpolation connectedCache_interp = RigidbodyInterpolation.None;
    CollisionDetectionMode connectedCache_detect = CollisionDetectionMode.Discrete;
    bool inputHeldDown = false;
    Vector3 initialHitPos = new Vector3();

    Vector3[] BeamPoints = new Vector3[16];



    public float force = 600;
    public float damping = 6;

    public Transform jointTrans;


    void Start()
    {
        if (!lineRenderer) lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.numPositions = BeamPoints.Length;
        if (!gunEnd) gunEnd = GetComponent<Transform>();
        if (!beamEnd) beamEnd = GetComponent<Transform>();
        if (!joint) joint = beamEnd.GetComponent<ConfigurableJoint>();

    }

    void Update()
    {
    }

    void FixedUpdate()
    {
        ProcessBeamState();
    }

    void OnConnectionEnter()
    {
        connectedBody = currentHit.rigidbody;
        beamEnd.position = currentHit.point;
        initialHitPos = connectedBody.transform.InverseTransformPoint(beamEnd.position);
        // Gravity
        connectedCache_Gravity = connectedBody.useGravity;
        connectedBody.useGravity = false;
        // Solver Count
        connectedCache_SolverCount = connectedBody.solverIterations;
        connectedBody.solverIterations = connectedSolverCount;
        // Interpolation
        connectedCache_interp = connectedBody.interpolation;
        connectedBody.interpolation = RigidbodyInterpolation.Interpolate;
        // Detection
        connectedCache_detect = connectedBody.collisionDetectionMode;
        connectedBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        // Joint Connection
        //joint.connectedBody = connectedBody;

        jointTrans = AttachJoint(connectedBody, beamEnd.position);

        //AttachJoint(beamEnd, beamEnd.GetComponent<Rigidbody>(), joint, connectedBody);

        //connectedDistance = Vector3.Distance(gunEnd.position, connectedPoint);
    }

    void OnConnectionStay()
    {
        //connectedBody.rotation = gunEnd.transform.rotation;
        // Vector3 endPos = connectedBody.transform.TransformPoint(connectedBody.transform.localPosition - initialHitPos);
        Vector3 targetPosition = beamEnd.transform.position;
        // connectedBody.MovePosition(endPos);

        jointTrans.position = targetPosition;
        //connectedPoint = targetPosition;
    }

    void OnConnectionExit()
    {
        connectedBody.useGravity = connectedCache_Gravity;
        connectedBody.solverIterations = connectedCache_SolverCount;
        connectedBody.interpolation = connectedCache_interp;
        connectedBody.collisionDetectionMode = connectedCache_detect;
        joint.connectedBody = null;
        connectedBody = null;


        Destroy(jointTrans.gameObject);
    }

    void LateUpdate()
    {
        // Drawing Beam
        switch (currentState)
        {
            case GunState.INACTIVE:
                if (lineRenderer.enabled)
                    lineRenderer.enabled = false;
                break;
            case GunState.SEARCHING:
                if (!lineRenderer.enabled)
                    lineRenderer.enabled = true;
                // Calculate Points
                Vector3 endPoint = gunEnd.position + gunEnd.forward * 3;
                BeamPoints[0] = gunEnd.position;
                for (int i = 1; i < BeamPoints.Length; i++)
                {
                    float percentage = i / (BeamPoints.Length - 1.0f);
                    BeamPoints[i] = Vector3.Lerp(gunEnd.position, endPoint, percentage);
                    //BeamPoints[i] = InterpolateFromCatmullRomSpline(
                    //    gunEnd.position,
                    //    Vector3.Lerp(gunEnd.position, endPoint, percentage) + (gunEnd.right * Mathf.Sin(Time.time)),
                    //    Vector3.Lerp(gunEnd.position, endPoint, percentage) - (gunEnd.right * Mathf.Sin(Time.time)),
                    //    endPoint, percentage);
                }
                lineRenderer.SetPositions(BeamPoints);
                break;
            case GunState.CONNECTED:
                if (!lineRenderer.enabled)
                    lineRenderer.enabled = true;
                // Calculate Points
                BeamPoints[0] = gunEnd.position;
                for (int i = 1; i < BeamPoints.Length; i++)
                {
                    float percentage = i / (BeamPoints.Length - 1.0f);
                    BeamPoints[i] = Vector3.Lerp(gunEnd.position, beamEnd.position, percentage);
                }
                lineRenderer.SetPositions(BeamPoints);
                break;
            default:
                break;
        }
    }

    void ProcessBeamState()
    {
        // Check Current Input
        bool input = Input.GetButton("Fire1");

        if (input)
        {
            // Input Is Down
            inputHeldDown = true;
            if (currentState == GunState.INACTIVE)
                currentState = GunState.SEARCHING;
        }
        else
        {
            // Input Is Up
            if (currentState == GunState.SEARCHING)
                currentState = GunState.INACTIVE;

            // Input Was Released
            if (inputHeldDown)
            {
                inputHeldDown = false;
                if (currentState == GunState.CONNECTED)
                {
                    // Lost Connection
                    OnConnectionExit();
                    currentState = GunState.INACTIVE;
                }
            }
        }

        // Try to Connect
        if (currentState == GunState.SEARCHING)
        {
            bool rayHit = Physics.Raycast(gunEnd.position, gunEnd.forward, out currentHit, float.MaxValue, layerMask);
            if (rayHit)
            {
                if (currentHit.rigidbody && !(currentHit.rigidbody.gameObject.isStatic))
                {
                    // Begin Connection
                    OnConnectionEnter();
                    currentState = GunState.CONNECTED;
                }
            }
        }

        // Connected
        if (currentState == GunState.CONNECTED)
        {
            OnConnectionStay();
        }
    }


    public static Vector2 InterpolateFromCatmullRomSpline(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, float t)
    {
        return 0.5f * ((2f * p2) + (-p1 + p3) * t + (2f * p1 - 5f * p2 + 4f * p3 - p4) * Mathf.Pow(t, 2f) + (-p1 + 3f * p2 - 3f * p3 + p4) * Mathf.Pow(t, 3f));
    }
    public static Vector3 InterpolateFromCatmullRomSpline(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, float t)
    {
        return 0.5f * ((2f * p2) + (-p1 + p3) * t + (2f * p1 - 5f * p2 + 4f * p3 - p4) * Mathf.Pow(t, 2f) + (-p1 + 3f * p2 - 3f * p3 + p4) * Mathf.Pow(t, 3f));
    }

    //void AttachJoint(Transform _startTransform, Rigidbody _startRb, ConfigurableJoint _joint, Rigidbody _endRb)
    //{
    //    _joint.connectedBody = _endRb;
    //    _joint.configuredInWorldSpace = true;
    //    ConfigJointDrive(_joint.xDrive, force, damping);
    //    ConfigJointDrive(_joint.yDrive, force, damping);
    //    ConfigJointDrive(_joint.zDrive, force, damping);
    //    ConfigJointDrive(_joint.slerpDrive, force, damping);
    //    _joint.rotationDriveMode = RotationDriveMode.Slerp;
    //}

    //private void ConfigJointDrive(JointDrive _drive, float force, float damping)
    //{
    //    _drive.positionSpring = force;
    //    _drive.positionDamper = damping;
    //    _drive.maximumForce = Mathf.Infinity;
    //}

    Transform AttachJoint(Rigidbody rb, Vector3 attachmentPosition)
    {
        GameObject go = new GameObject("Attachment Point");
        go.hideFlags = HideFlags.HideInHierarchy;
        go.transform.position = attachmentPosition;

        var newRb = go.AddComponent<Rigidbody>();
        newRb.isKinematic = true;

        var joint = go.AddComponent<ConfigurableJoint>();
        joint.connectedBody = rb;
        joint.configuredInWorldSpace = true;
        joint.xDrive = NewJointDrive(force, damping);
        joint.yDrive = NewJointDrive(force, damping);
        joint.zDrive = NewJointDrive(force, damping);
        joint.slerpDrive = NewJointDrive(force, damping);
        joint.rotationDriveMode = RotationDriveMode.Slerp;

        return go.transform;
    }

    private JointDrive NewJointDrive(float force, float damping)
    {
        JointDrive drive = new JointDrive();
        // drive.mode = JointDriveMode.Position;
        drive.positionSpring = force;
        drive.positionDamper = damping;
        drive.maximumForce = Mathf.Infinity;
        return drive;
    }
}
