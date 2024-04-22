using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PInverseFunction : MonoBehaviour
{
    public GameObject arm1, arm2;
    public float x, y;
    private float l1, l2;
    public TMPro.TextMeshProUGUI angles;
    public bool useValue2;

    // Start is called before the first frame update
    void Start()
    {
        l1 = arm1.GetComponent<MeshRenderer>().bounds.extents.y * 2;
        l2 = arm2.GetComponent<MeshRenderer>().bounds.extents.y * 2;
    }

    // Update is called once per frame
    void Update()
    {
        Vector2 mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        x = mousePosition.x;
        y = mousePosition.y;

        PInverse();
    }

    private void PInverse()
    {
        double radToDeg = 180 / Mathf.PI;
        double theta1, theta2;
        double theta2Rad;

        double dist = Mathf.Sqrt((x * x) + (y * y));
        if (dist > l1 + l2)
        {
            dist = l1 + l2;
        }

        float temp = (float) ((l1 * l1) + (l2 * l2) - (dist * dist));
        Debug.Log(temp);
        float cosAlpha = temp / (2 * l1 * l2);

        if (!useValue2)
        {
            theta2Rad = Mathf.PI - Mathf.Acos(cosAlpha);
        }
        else
        {
            // this one MAY NEED TO CHANGE to (360 - calculated thing)
            theta2Rad = (2*Mathf.PI) - Mathf.Acos(-cosAlpha);
        }
        theta2 = theta2Rad * radToDeg;

        temp = l2 * Mathf.Sin((float)theta2Rad);
        float temp2 = l2 * Mathf.Cos((float)theta2Rad);

        float beta = Mathf.Atan2(temp, (l1 + temp2));

        temp = Mathf.Atan2(y, x);

        theta1 = (temp - beta) * radToDeg;

        positionArms(theta1, theta2);

        angles.text = "(" + System.Math.Round(theta1, 2) + ", " + System.Math.Round(theta2, 2) + ")";
    }

    private void positionArms(double theta1, double theta2)
    {
        float degToRad = Mathf.PI / 180;

        arm1.transform.eulerAngles = new Vector3(
            arm1.transform.eulerAngles.x,
            arm1.transform.eulerAngles.y,
            270 + (float) theta1);

        arm2.transform.position = new Vector3(l1 * Mathf.Cos((float) theta1 * degToRad), l2 * Mathf.Sin((float) theta1 * degToRad), 0);

        arm2.transform.eulerAngles = new Vector3(
            arm2.transform.eulerAngles.x,
            arm2.transform.eulerAngles.y,
            270 + (float) theta1 + (float) theta2);
    }
}
