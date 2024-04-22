using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class MultibodyFABRIK : MonoBehaviour
{
    public GameObject target;

    public float errorScale, errorThreshold, maxIterations;
    private List<Transform> armPoints = new List<Transform>();
    private List<Transform> targets = new List<Transform>();
    private List<int> effectorIndices = new List<int>();
    private Dictionary<Transform, int> subBases = new Dictionary<Transform, int>();
    private GameObject endEffector;
    private Vector<float> error;

    // Start is called before the first frame update
    void Start()
    {
        endEffector = new GameObject("endEffector");
        var temp = GetComponentsInChildren<Transform>();
        for (int i = 1; i < temp.Length; i++)
        {
            armPoints.Add(temp[i]);
            if (temp[i].childCount == 0)
            {
                Vector3 endEffectorPosition = temp[i].position + (Quaternion.Euler(temp[i].eulerAngles.x, temp[i].eulerAngles.y, temp[i].eulerAngles.z) * new Vector3(temp[i].GetComponent<MeshFilter>().mesh.bounds.size.x, 0));
                armPoints.Add(Instantiate(endEffector, temp[i]).transform);
                armPoints[armPoints.Count - 1].position = endEffectorPosition;
                targets.Add(Instantiate(target, endEffectorPosition, Quaternion.identity).transform);
                effectorIndices.Add(armPoints.Count - 1);
            }
            if (temp[i].childCount > 1)
            {
                subBases.Add(temp[i], temp[i].childCount);
            }
        }
        error = Vector<float>.Build.Dense(effectorIndices.Count);
    }

    // Update is called once per frame
    void Update()
    {
        Transform curr;
        List<Vector3> subBasePositions = new List<Vector3>();
        int nrIterations = 0;

        int targetIndex = 0;
        foreach (int i in effectorIndices)
        {
            Vector3 errorTemp = armPoints[i].position - targets[targetIndex].position;
            error[0 + (3 * targetIndex)] = errorTemp.x;
            error[1 + (3 * targetIndex)] = errorTemp.y;
            error[2 + (3 * targetIndex)] = errorTemp.z;
            targetIndex++;
        }

        while (error.L2Norm() >= errorThreshold && nrIterations < maxIterations)
        {
            // forward reach
            for (int i = 0; i < effectorIndices.Count; i++)
            {
                armPoints[effectorIndices[i]].position = targets[i].position;
                curr = armPoints[effectorIndices[i]].parent;
                while (!subBases.ContainsKey(curr))
                {
                    float newDist = (curr.GetChild(0).position - curr.position).magnitude;
                    float distRatio = curr.GetComponent<MeshFilter>().mesh.bounds.size.x / newDist;
                    curr.position = ((1 - distRatio) * curr.position) + (distRatio * curr.GetChild(0).position);
                }
                // if it IS a sub-base, calculate a position but store it somewhere and reduce the child count, jump to next end effector?
                // once child count is 0, calculate sub-base position and iterate from there until the root
            }

            // backward reach
            
            nrIterations++;
        }

        // positioning + rotation
    }
}
