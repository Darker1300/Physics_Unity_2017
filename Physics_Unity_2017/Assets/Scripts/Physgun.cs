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
    // public float connectedDistance = 0.0f;
    //public float sensitivity = 5.0f;

    GunState currentState = GunState.INACTIVE;

    Rigidbody connectedBody = null;
    RaycastHit currentHit;
    public Joint joint = null;
    public int connectedSolverCount = 10;
    bool connectedCache_Gravity = false;
    int connectedCache_SolverCount = 0;
    RigidbodyInterpolation connectedCache_interp = RigidbodyInterpolation.None;
    CollisionDetectionMode connectedCache_detect = CollisionDetectionMode.Discrete;
    bool heldDown = false;

    Vector3[] BeamPoints = new Vector3[16];

    void Start()
    {
        if (!lineRenderer) lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.numPositions = BeamPoints.Length;
        if (!gunEnd) gunEnd = GetComponent<Transform>();
        if (!beamEnd) beamEnd = GetComponent<Transform>();
        if (!joint) joint = beamEnd.GetComponent<Joint>();

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

        //connectedDistance = Vector3.Distance(gunEnd.position, connectedPoint);
        joint.connectedBody = connectedBody;
    }

    void OnConnectionStay()
    {
        //connectedBody.rotation = gunEnd.transform.rotation;

        //Vector3 targetPosition = (gunEnd.transform.position +
        //        gunEnd.transform.forward * connectedDistance);
        //connectedPoint = targetPosition;
        //connectedBody.MovePosition(targetPosition);

    }

    void OnConnectionExit()
    {
        connectedBody.useGravity = connectedCache_Gravity;
        connectedBody.solverIterations = connectedCache_SolverCount;
        connectedBody.interpolation = connectedCache_interp;
        connectedBody.collisionDetectionMode = connectedCache_detect;
        //connectedBody.freezeRotation = false;
        joint.connectedBody = null;
        connectedBody = null;
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
            heldDown = true;
            if (currentState == GunState.INACTIVE)
                currentState = GunState.SEARCHING;
        }
        else
        {
            // Input Is Up
            if (currentState == GunState.SEARCHING)
                currentState = GunState.INACTIVE;

            // Input Was Released
            if (heldDown)
            {
                heldDown = false;
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
}
