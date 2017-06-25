using UnityEngine;

public class MaterialAnimator : MonoBehaviour
{
    public Material material = null;
    public string MaterialParameter = "";
    public bool activate = false;
    public float speed = 0.5f;
    public AnimationCurve curve = new AnimationCurve();
    public float evaluation = 0.0f;

    [Range(0, 2)]
    public float currentTime = 0.0f;

    private void Start()
    {
        currentTime = 0.0f;
        evaluation = 0.0f;
        currentTime = 0.0f;
    }

    private void Update()
    {
        if (activate && material)
        {
            currentTime = Mathf.Repeat(currentTime + Time.deltaTime * speed, 2.0f);
            float p = Mathf.PingPong(currentTime, 1.0f);
            evaluation = Mathf.Clamp01(curve.Evaluate(p));
            material.SetFloat(MaterialParameter, evaluation);
        }
    }
}