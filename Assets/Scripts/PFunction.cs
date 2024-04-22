using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PFunction : MonoBehaviour
{
    public GameObject arm1, arm2;
    public float theta1, theta2;
    private float l1, l2;
    public TMPro.TextMeshProUGUI effectorPos;
    // Start is called before the first frame update
    void Start()
    {
        l1 = arm1.GetComponent<MeshRenderer>().bounds.extents.y * 2;
        l2 = arm2.GetComponent<MeshRenderer>().bounds.extents.y * 2;
    }

    // Update is called once per frame
    void Update()
    {
        P();
    }

    private void P()
    {
        float degToRad = Mathf.PI / 180;

        positionArms();

        float y = (l1 * Mathf.Sin(theta1 * degToRad)) + (l2 * Mathf.Sin((theta1 + theta2) * degToRad));
        float x = (l1 * Mathf.Cos(theta1 * degToRad)) + (l2 * Mathf.Cos((theta1 + theta2) * degToRad));

        effectorPos.text = "(" + x + ", " + y + ")";
    }

    private void positionArms()
    {
        float degToRad = Mathf.PI / 180;

        arm1.transform.eulerAngles = new Vector3(
            arm1.transform.eulerAngles.x,
            arm1.transform.eulerAngles.y,
            270 + theta1);

        arm2.transform.position = new Vector3(l1 * Mathf.Cos(theta1 * degToRad), l2 * Mathf.Sin(theta1 * degToRad), 0);

        arm2.transform.eulerAngles = new Vector3(
            arm2.transform.eulerAngles.x,
            arm2.transform.eulerAngles.y,
            270 + theta1 + theta2);
    }
}
