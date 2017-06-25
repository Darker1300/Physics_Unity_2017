using UnityEngine;

public class PlayerMovement : MonoBehaviour
{
    public CharacterController controller = null;
    public string HorizontalInput = "Horizontal";
    public string VerticalInput = "Vertical";
    public string JumpInput = "Jump";
    public float speed = 10f;
    public float jumpSpeed = 8.0F;
    public float gravity = 20.0F;
    private Vector3 moveDirection;

    private void Start()
    {
        if (!controller) controller = GetComponent<CharacterController>();
    }

    private void Update()
    {
        if (controller.isGrounded)
        {
            // Calculate moveDirection
            moveDirection = new Vector3(Input.GetAxis(HorizontalInput), 0, Input.GetAxis(VerticalInput));
            moveDirection = transform.TransformDirection(moveDirection);
            // Apply Speed modifier
            moveDirection *= speed;
            // Jumping
            if (Input.GetButton(JumpInput))
                moveDirection.y = jumpSpeed;
        }
    }

    private void FixedUpdate()
    {
        if (!controller) return;
        // Apply gravity
        moveDirection.y -= gravity * Time.deltaTime;
        controller.Move(moveDirection * Time.fixedDeltaTime);
    }
}