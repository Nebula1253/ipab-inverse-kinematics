using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CyclicCoordinateDescent : MonoBehaviour
{
    public int nrJoints;
    public float threshold;
    private float jointSize;
    public GameObject joint;
    private GameObject endEffector;
    private int maxIterations = 10000;
    private GameObject[] joints;

    // Start is called before the first frame update
    void Start()
    {
        joints = new GameObject[nrJoints];
        joints[0] = Instantiate(joint, gameObject.transform);

        jointSize = joints[0].GetComponent<MeshRenderer>().bounds.extents.x * 2;

        for (int i = 1; i < nrJoints; i++)
        {
            joints[i] = Instantiate(joint, joints[i - 1].transform);
            joints[i].transform.localPosition = new Vector2(0, jointSize);
            joints[i].transform.eulerAngles = new Vector3(0, 0, -90);
        }

        endEffector = new GameObject("endEffector");
        endEffector.transform.position = new Vector3(jointSize * nrJoints, 0);
        endEffector.transform.SetParent(joints[nrJoints - 1].transform);
    }

    // Update is called once per frame
    void Update()
    {
        int nrIterations = 0;

        Vector3 targetPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        targetPos.z = 0;

        if (targetPos.magnitude > (nrJoints * jointSize))
        {
            targetPos = nrJoints * jointSize * targetPos.normalized;
            Debug.Log(nrJoints * jointSize);
        }

        while ((endEffector.transform.position - targetPos).magnitude > threshold
            && nrIterations < maxIterations)
        {
            for (int j = nrJoints - 1; j >= 0; j--)
            {
                Vector3 targetVector = targetPos - joints[j].transform.position;
                Vector3 endEffectorVector = endEffector.transform.position - joints[j].transform.position;

                if (targetVector.magnitude != 0 && endEffectorVector.magnitude != 0)
                {
                    float dotProduct = Vector3.Dot(targetVector, endEffectorVector);
                    double cosine = dotProduct / (targetVector.magnitude * endEffectorVector.magnitude);
                    // correcting for floating-point errors
                    if (cosine > 1) { cosine = 1; }
                    double angle = Mathf.Acos((float) cosine) * (180 / Mathf.PI);

                    Vector3 crossProduct = Vector3.Cross(targetVector, endEffectorVector);
                    float sine = crossProduct.z / (targetVector.magnitude * endEffectorVector.magnitude);

                    if ((Mathf.Asin(sine) * (180 / Mathf.PI)) > 0)
                    {
                        angle *= -1;
                    }
                    //Debug.Log((j, cosine, angle, Mathf.Acos((float) cosine) * (180 / Mathf.PI)));
                    joints[j].transform.localEulerAngles += new Vector3(0, 0, (float) angle);
                }
            }
            nrIterations++;
        }

        if (nrIterations == maxIterations)
        {
            Debug.Log((endEffector.transform.position, targetPos));
            Debug.Log((endEffector.transform.position - targetPos).magnitude);
        }
    }
}
