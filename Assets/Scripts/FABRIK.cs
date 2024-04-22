using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FABRIK : MonoBehaviour
{
    public int nrJoints;
    public float tolerance;
    public GameObject joint;
    public GameObject target;
    private GameObject endEffector;
    private GameObject[] joints;
    private int maxIterations = 10000;
    private float jointSize;
    private Vector3[] points;

    // Start is called before the first frame update
    void Start()
    {
        joints = new GameObject[nrJoints];
        points = new Vector3[nrJoints + 1];

        joints[0] = Instantiate(joint, gameObject.transform);
        jointSize = joints[0].GetComponent<MeshRenderer>().bounds.extents.x * 2;

        points[0] = new Vector3(0, 0);

        for (int i = 1; i < nrJoints; i++)
        {
            joints[i] = Instantiate(joint, joints[i - 1].transform);
            //joints[i].transform.localPosition = new Vector3(4,0);
            joints[i].transform.localPosition = new Vector3(jointSize,0);
            points[i] = new Vector3(jointSize * i, 0);
            //joints[i].transform.eulerAngles = new Vector3(0, 0, -90);
        }

        endEffector = new GameObject("endEffector");
        //endEffector = Instantiate(joint, new Vector3(4 * nrJoints, 0), Quaternion.identity);
        endEffector.transform.position = new Vector3(jointSize * nrJoints, 0);
        points[nrJoints] = endEffector.transform.position;
        endEffector.transform.SetParent(joints[nrJoints - 1].transform);
    }

    // Update is called once per frame
    void Update()
    {
        int nrIterations = 0;
        Vector3 targetPos = /*Camera.main.ScreenToWorldPoint(Input.mousePosition);*/ target.transform.position;
        //targetPos.z = 0;

        //if (targetPos.magnitude > (nrJoints * jointSize))
        //{
        //    targetPos = jointSize * nrJoints * targetPos.normalized;
        //}
        
        while ((points[nrJoints] - targetPos).magnitude > tolerance
            && nrIterations < maxIterations
            )
        {
            // backward iteration
            points[nrJoints] = targetPos;
            for (int i = nrJoints - 1; i >=0; i--)
            {
                float newDist = (points[i + 1] - points[i]).magnitude;
                float distRatio = jointSize / newDist;
                points[i] = ((1 - distRatio) * points[i + 1]) + (distRatio * points[i]);
            }

            // forward iteration
            points[0] = new Vector3(0, 0, 0);
            for (int i = 0; i <= nrJoints - 1; i++)
            {
                float newDist = (points[i + 1] - points[i]).magnitude;
                float distRatio = jointSize / newDist;
                points[i+1] = ((1 - distRatio) * points[i]) + (distRatio * points[i+1]);
            }
            nrIterations++;
        }
        // and now to move the things around

        Vector3[] jointPos = new Vector3[nrJoints + 1];
        jointPos[0] = Vector3.zero;
        for (int i = 1; i <= nrJoints; i++)
        {
            jointPos[i] = points[i] - points[i - 1];
        }

        for (int i = 0; i < nrJoints; i++)
        {
            joints[i].transform.position = points[i];
            joints[i].transform.right = points[i + 1] - points[i];
        }

        //Debug.Log(nrIterations);

        if (nrIterations == maxIterations)
        {
            Debug.Log((targetPos, points[nrJoints]));
        }
    }
}
