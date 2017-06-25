using UnityEngine;

public class MouseLookAxis : MonoBehaviour
{
    public enum Axes { MouseX = 0, MouseY = 1 }

    public Axes axis = Axes.MouseX;
    public float sensitivity = 10F;
    public float minAngleY = -60F;
    public float maxAngleY = 60F;
    private float rotationY = 0F;

    private void Start()
    {
        if (GetComponent<Rigidbody>())
            GetComponent<Rigidbody>().freezeRotation = true;
    }

    private void Update()
    {
        if (axis == Axes.MouseX)
        {
            transform.Rotate(0, Input.GetAxis("Mouse X") * sensitivity, 0);
        }
        else
        {
            rotationY += Input.GetAxis("Mouse Y") * sensitivity;
            rotationY = Mathf.Clamp(rotationY, minAngleY, maxAngleY);

            transform.localEulerAngles = new Vector3(-rotationY, transform.localEulerAngles.y, 0);
        }
    }
}