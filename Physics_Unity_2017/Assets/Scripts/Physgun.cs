﻿using UnityEngine;

public class Physgun : MonoBehaviour
{
    public enum GunState
    {
        INACTIVE,
        SEARCHING,
        CONNECTED
    }

    [System.Serializable]
    public class ConnectedCache
    {
        public bool useGravity;
        public int solverCount;
        public RigidbodyInterpolation interpolation;
        public CollisionDetectionMode detection;

        public int layer;

        public void Read(Rigidbody _rigidBody)
        {
            useGravity = _rigidBody.useGravity;
            solverCount = _rigidBody.solverIterations;
            interpolation = _rigidBody.interpolation;
            detection = _rigidBody.collisionDetectionMode;
            GameObject go = _rigidBody.gameObject;
            layer = go.layer;
        }

        public void Write(ref Rigidbody _rigidBody)
        {
            _rigidBody.useGravity = useGravity;
            _rigidBody.solverIterations = solverCount;
            _rigidBody.interpolation = interpolation;
            _rigidBody.collisionDetectionMode = detection;
            GameObject go = _rigidBody.gameObject;
            go.layer = layer;
        }

        public void Clear()
        {
            useGravity = false;
            solverCount = 0;
            interpolation = RigidbodyInterpolation.None;
            detection = CollisionDetectionMode.Discrete;
            layer = 0;
        }
    }

    public Transform gunEnd = null;
    public Transform beamEnd = null;
    public LineRenderer lineRenderer = null;
    public Joint joint = null;

    [Header("Configuration")]
    public string inputUseID = "Fire1";

    public LayerMask layerMask = new LayerMask();
    public float distanceSpeedModifier = 100;
    public float distanceMinimum = 1;
    public float rotationSpeedDegrees = 120;
    public MouseLook mlx;
    public MouseLook mly;

    private RaycastHit currentHit;
    private GunState currentState = GunState.INACTIVE;
    private bool inputWasHeldDown = false;
    private Rigidbody connectedBody = null;
    private Vector3 connectedLocalHit = new Vector3();
    private Vector3[] beamLinePoints = new Vector3[17];

    public ConnectedCache activeSettings = new ConnectedCache
    {
        useGravity = false,
        solverCount = 20,
        interpolation = RigidbodyInterpolation.Interpolate,
        detection = CollisionDetectionMode.ContinuousDynamic
    };

    public ConnectedCache cacheSettings = new ConnectedCache();

    private void Start()
    {
        if (!lineRenderer) lineRenderer = GetComponent<LineRenderer>();
        if (!gunEnd) gunEnd = GetComponent<Transform>();
        if (!beamEnd) beamEnd = GetComponent<Transform>();
        if (!joint) joint = beamEnd.GetComponent<Joint>();

        // Set up Line Renderer
        lineRenderer.numPositions = beamLinePoints.Length;
    }

    private void Update()
    {
        ProcessBeamState();
    }

    private void FixedUpdate()
    {
    }

    private void LateUpdate()
    {
        DrawBeamState();
    }

    private void OnConnectionEnter()
    {
        // Store connection details
        connectedBody = currentHit.rigidbody;
        connectedLocalHit = currentHit.transform.InverseTransformPoint(currentHit.point);
        beamEnd.position = currentHit.point;
        beamEnd.rotation = currentHit.transform.rotation;

        // Store RigidBody settings
        cacheSettings.Read(connectedBody);
        // Apply custom RigidBody settings
        activeSettings.Write(ref connectedBody);

        // Attach to joint
        joint.connectedBody = connectedBody;
    }

    private void OnConnectionStay()
    {
        // Distance Control
        if (Input.mouseScrollDelta.y != 0.0f)
        {
            float dist = Input.mouseScrollDelta.y * Time.deltaTime * distanceSpeedModifier;
            if (Mathf.Sign(Input.mouseScrollDelta.y) == 1.0f)
            {
                beamEnd.position += gunEnd.forward * dist;
            }
            else
            {
                Vector3 min = gunEnd.position + gunEnd.forward * distanceMinimum;
                beamEnd.position = Vector3.MoveTowards(beamEnd.position, min, Mathf.Abs(dist));
            }
        }
        // Rotation Test
        if (Input.GetKey(KeyCode.E))
        {
            mlx.enabled = false;
            mly.enabled = false;

            beamEnd.RotateAround(beamEnd.TransformPoint(joint.anchor), -gunEnd.up, Time.deltaTime * rotationSpeedDegrees * Input.GetAxis("Mouse X"));
            beamEnd.RotateAround(beamEnd.TransformPoint(joint.anchor), gunEnd.right, Time.deltaTime * rotationSpeedDegrees * Input.GetAxis("Mouse Y"));
        }
        else
        {
            mlx.enabled = true;
            mly.enabled = true;
        }
        

        #region HiddenTesting

        //if (Input.GetButton("Fire2"))
        //{
        //beamEnd.Rotate(Vector3.left * Time.fixedDeltaTime * 5.0f, Space.World);
        //rotationFromBeamEnd = Quaternion.FromToRotation(beamEnd.forward, currentHit.transform.forward);
        //connectedBody.angularVelocity = rotationFromBeamEnd.eulerAngles * Mathf.Deg2Rad;
        //Debug.Log(connectedBody.angularVelocity);
        //connectedBody.transform.rotation = beamEnd.rotation * rotationFromBeamEnd;

        //Quaternion target = Quaternion.AngleAxis(10, Vector3.up) * connectedBody.transform.rotation;
        //// Quaternion smooth = Quaternion.RotateTowards(connectedBody.transform.rotation, target, 10 * Time.fixedDeltaTime);
        //connectedBody.MoveRotation(target);
        //}

        //Quaternion CurrentDifference = Quaternion.FromToRotation(beamEnd.forward, connectedBody.transform.forward);

        // if this doesn't work try "B.transform.eulerAngles - A.transform.eulerAngles" below
        // Vector3 difference = beamEnd.eulerAngles - connectedBody.transform.eulerAngles;

        //Vector3 difference = beamEnd.eulerAngles - connectedBody.transform.eulerAngles;
        //Vector3 velocity = difference / Time.fixedDeltaTime;
        //connectedBody.angularVelocity += velocity;

        //connectedBody.rotation = gunEnd.transform.rotation;

        //Vector3 targetPosition = (gunEnd.transform.position +
        //        gunEnd.transform.forward * connectedDistance);
        //connectedPoint = targetPosition;
        //connectedBody.MovePosition(targetPosition);

        #endregion HiddenTesting
    }

    private void OnConnectionExit()
    {
        // Restore previous RigidBody settings
        cacheSettings.Write(ref connectedBody);
        cacheSettings.Clear();

        joint.connectedBody = null;
        connectedBody = null;

        beamEnd.position = gunEnd.position;

        mlx.enabled = true;
        mly.enabled = true;
    }

    private void ProcessBeamState()
    {
        // Check Current Input
        bool input = Input.GetButton(inputUseID);

        if (input)
        {
            // Input Is Down

            // Lock Cursor
            Cursor.visible = false;
            Cursor.lockState = CursorLockMode.Locked;

            inputWasHeldDown = true;
            if (currentState == GunState.INACTIVE)
                currentState = GunState.SEARCHING;
        }
        else
        {
            // Input Is Up
            if (currentState == GunState.SEARCHING)
                currentState = GunState.INACTIVE;

            // Input Was Released
            if (inputWasHeldDown)
            {
                inputWasHeldDown = false;
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
            // Raycast
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

    private void DrawBeamState()
    {
        switch (currentState)
        {
            case GunState.INACTIVE:
                if (lineRenderer.enabled)
                    lineRenderer.enabled = false;
                break;

            case GunState.SEARCHING:
                {
                    if (!lineRenderer.enabled)
                        lineRenderer.enabled = true;
                    // Calculate Points
                    beamLinePoints[0] = gunEnd.position;
                    for (int i = 1; i < beamLinePoints.Length; i++)
                    {
                        float percentage = i / (beamLinePoints.Length - 1.0f);
                        beamLinePoints[i] = Vector3.Lerp(gunEnd.position, currentHit.point, percentage);
                    }
                    lineRenderer.SetPositions(beamLinePoints);
                }
                break;

            case GunState.CONNECTED:
                {
                    if (!lineRenderer.enabled)
                        lineRenderer.enabled = true;
                    // Calculate Points
                    Vector3 endPoint = connectedBody.transform.TransformPoint(connectedLocalHit);
                    Vector3 midPoint = Vector3.Lerp(gunEnd.position, beamEnd.position, 0.5f);
                    beamLinePoints[0] = gunEnd.position;
                    for (int i = 1; i < beamLinePoints.Length; i++)
                    {
                        float percentage = i / (beamLinePoints.Length - 1.0f);
                        //BeamPoints[i] = Vector3.Lerp(gunEnd.position, beamEnd.position, percentage);
                        beamLinePoints[i] = Bezier(gunEnd.position, midPoint, endPoint, percentage);
                    }
                    lineRenderer.SetPositions(beamLinePoints);
                }
                break;

            default:
                break;
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

    public static Vector3 Bezier(Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return Vector3.Lerp(Vector3.Lerp(p1, p2, t), Vector3.Lerp(p2, p3, t), t);
    }
    private static int layermask_to_layer(LayerMask layerMask)
    {
        int layerNumber = 0;
        int layer = layerMask.value;
        while (layer > 0)
        {
            layer = layer >> 1;
            layerNumber++;
        }
        return layerNumber - 1;
    }
}