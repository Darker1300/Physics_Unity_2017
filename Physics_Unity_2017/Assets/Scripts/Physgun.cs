using UnityEngine;
using UnityEngine.Events;

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

    public string inputRotateID = "Fire2";

    public LayerMask layerMask = new LayerMask();
    public float distanceSpeedModifier = 100;
    public float distanceMinimum = 1;
    public float rotationSpeedDegrees = 360;

    private RaycastHit currentHit;
    private GunState currentState = GunState.INACTIVE;
    private Rigidbody connectedBody = null;
    private Vector3 connectedLocalHit = new Vector3();
    private Vector3[] beamLinePoints = new Vector3[17];
    private bool grabStarted = false;
    private bool rotationStarted = false;

    public ConnectedCache activeSettings = new ConnectedCache
    {
        useGravity = false,
        solverCount = 20,
        interpolation = RigidbodyInterpolation.Interpolate,
        detection = CollisionDetectionMode.ContinuousDynamic
    };

    public ConnectedCache cacheSettings = new ConnectedCache();

    public UnityEvent GrabEnter = new UnityEvent();

    [HideInInspector]
    public UnityEvent GrabStay = new UnityEvent();

    public UnityEvent GrabExit = new UnityEvent();

    public UnityEvent RotationEnter = new UnityEvent();

    [HideInInspector]
    public UnityEvent RotationStay = new UnityEvent();

    public UnityEvent RotationExit = new UnityEvent();

    private void Start()
    {
        if (!lineRenderer) lineRenderer = GetComponent<LineRenderer>();
        if (!gunEnd) gunEnd = GetComponent<Transform>();
        if (!beamEnd) beamEnd = GetComponent<Transform>();
        if (!joint) joint = beamEnd.GetComponent<Joint>();

        // Set up Line Renderer
        lineRenderer.numPositions = beamLinePoints.Length;

        GrabEnter.AddListener(OnGrabEnter);
        GrabStay.AddListener(OnGrabStay);
        GrabExit.AddListener(OnGrabExit);

        RotationEnter.AddListener(OnRotationEnter);
        RotationStay.AddListener(OnRotationStay);
        RotationExit.AddListener(OnRotationExit);
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

    private void OnGrabEnter()
    {
        // Store grab details
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

        // Ragdoll support
        Ragdoll rd = connectedBody.GetComponentInParent<Ragdoll>();
        if (rd != null) rd.RagdollState = true;
    }

    private void OnGrabStay()
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
        if (Input.GetButton(inputRotateID))
        {
            if (rotationStarted)
                RotationStay.Invoke();
            else
            {
                RotationEnter.Invoke();
                rotationStarted = true;
            }
        }
        else if (rotationStarted)
        {
            RotationExit.Invoke();
            rotationStarted = false;
        }
    }

    private void OnGrabExit()
    {
        // Ragdoll support
        Ragdoll rd = connectedBody.GetComponentInParent<Ragdoll>();
        if (rd != null)
        {
            rd.RagdollState = false;
        }

        // Restore previous RigidBody settings
        cacheSettings.Write(ref connectedBody);
        cacheSettings.Clear();

        joint.connectedBody = null;
        connectedBody = null;

        beamEnd.position = gunEnd.position;
    }

    private void OnRotationEnter()
    {
    }

    private void OnRotationStay()
    {
        beamEnd.Rotate(gunEnd.up, Time.deltaTime * rotationSpeedDegrees * Input.GetAxis("Mouse X"), Space.World);
        beamEnd.Rotate(-gunEnd.right, Time.deltaTime * rotationSpeedDegrees * Input.GetAxis("Mouse Y"), Space.World);
    }

    private void OnRotationExit()
    {
    }

    private void ProcessBeamState()
    {
        // Check Current Input
        bool input = Input.GetButton(inputUseID);

        if (input)
        {   // Input Is Down
            // Lock Cursor
            Cursor.visible = false;
            Cursor.lockState = CursorLockMode.Locked;

            grabStarted = true;
            if (currentState == GunState.INACTIVE)
                currentState = GunState.SEARCHING;
        }
        else
        {   // Input Is Up
            if (currentState == GunState.SEARCHING)
                currentState = GunState.INACTIVE;

            // Input Was Released
            if (grabStarted)
            {
                grabStarted = false;
                if (currentState == GunState.CONNECTED)
                {
                    // Lost Grab
                    GrabExit.Invoke();
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
                    // Begin Grab
                    GrabEnter.Invoke();
                    currentState = GunState.CONNECTED;
                }
            }
        }

        // Connected
        if (currentState == GunState.CONNECTED)
        {
            GrabStay.Invoke();
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
}