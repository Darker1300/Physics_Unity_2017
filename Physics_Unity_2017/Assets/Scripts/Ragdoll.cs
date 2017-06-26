using System.Collections.Generic;
using UnityEngine;

public class Ragdoll : MonoBehaviour
{
    public Animator animator = null;
    public List<Rigidbody> rigidbodies = new List<Rigidbody>();

    public bool RagdollState
    {
        get { return !animator.enabled; }
        set
        {
            animator.enabled = !value;
            foreach (Rigidbody r in rigidbodies)
                r.isKinematic = !value;
        }
    }

    private void Start()
    {
        if (!animator) animator = GetComponent<Animator>();

        foreach (Rigidbody r in rigidbodies)
            r.isKinematic = true;
    }

    private void Update()
    {
    }
}