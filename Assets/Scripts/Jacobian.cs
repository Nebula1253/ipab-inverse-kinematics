using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class Jacobian : MonoBehaviour
{
    // Start is called before the first frame update

    public enum Method
    {
        Transpose,
        PseudoInverse
    }

    public int nrJoints;
    public float errorScale;
    public GameObject arm;
    public GameObject target;
    public bool altJacobian;
    public Method method;

    private GameObject endEffector;
    private GameObject[] armSegments;
    //private Matrix<float> jointAngles;
    private Matrix<float> jacobian;
    private Matrix<float> deltaTheta;
    private float armSize;
    private Vector3[] jointLocs;
    private Vector3 targetPos;

    void Start()
    {
        //jointAngles = Matrix<float>.Build.Dense(nrJoints, 1);
        armSegments = new GameObject[nrJoints];
        jointLocs = new Vector3[nrJoints + 1];

        armSegments[0] = Instantiate(arm, gameObject.transform);
        armSize = armSegments[0].GetComponent<MeshRenderer>().bounds.extents.x * 2;

        jointLocs[0] = Vector3.zero;

        for (int i = 1; i < nrJoints; i++)
        {
            armSegments[i] = Instantiate(arm, armSegments[i - 1].transform);
            armSegments[i].transform.localPosition = new Vector3(armSize, 0);
            jointLocs[i] = new Vector3(armSize * 1, 0);
        }

        endEffector = new GameObject("endEffector");
        endEffector.transform.position = new Vector3(armSize * nrJoints, 0);
        jointLocs[nrJoints] = endEffector.transform.position;
        endEffector.transform.SetParent(armSegments[nrJoints - 1].transform);
    }

    // Update is called once per frame
    void Update()
    {
        targetPos = target.transform.position;

        if (targetPos.magnitude > (nrJoints * armSize))
        {
            targetPos = armSize * nrJoints * targetPos.normalized;
        }

        var M = Matrix<float>.Build;
        deltaTheta = M.Dense(nrJoints, 1);

        Vector3 eVector = errorScale * (endEffector.transform.position - targetPos);
        Matrix<float> e = M.Dense(3, 1);
        e[0, 0] = eVector.x;
        e[1, 0] = eVector.y;
        e[2, 0] = eVector.z;

        CalculateJacobian();
        switch(method)
        {
            case Method.Transpose:
                TransposeMethod(eVector, e);
                break;
            case Method.PseudoInverse:
                PseudoInverse(e);
                break;
        }

        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].transform.localEulerAngles += new Vector3(0, 0, deltaTheta[i, 0]);
        }
    }

    void CalculateJacobian()
    {
        var M = Matrix<float>.Build;
        Vector3[] jacobianTemp = new Vector3[nrJoints];

        for (int i = 0; i < nrJoints; i++)
        {
            if (!altJacobian) { jacobianTemp[i] = Vector3.Cross(armSegments[i].transform.forward, (endEffector.transform.position - armSegments[i].transform.position)); }
            else { jacobianTemp[i] = Vector3.Cross(armSegments[i].transform.forward, (targetPos - armSegments[i].transform.position)); }
        }

        // for now the third row is basically gonna be all 0s but maybe this won't be as stupid if i adapt this for 3d movements?????
        float[,] temp = new float[3, nrJoints];
        for (int i = 0; i < nrJoints; i++)
        {
            temp[0, i] = jacobianTemp[i].x;
            temp[1, i] = jacobianTemp[i].y;
            temp[2, i] = jacobianTemp[i].z;
        }

        jacobian = M.DenseOfArray(temp);
    }

    void TransposeMethod(Vector3 eVector, Matrix<float> e)
    {
        var temp = jacobian * jacobian.Transpose() * e;
        Vector3 jjte = new Vector3(temp[0, 0], temp[1, 0], temp[2, 0]);

        if (eVector.magnitude > 0)
        {
            float alpha = Vector3.Dot(eVector, jjte) / (jjte.magnitude * jjte.magnitude);
            deltaTheta = alpha * jacobian.Transpose() * e;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
        }
    }

    void PseudoInverse(Matrix<float> e)
    {
        var svd = jacobian.Svd();
        // have to modify W (transpose and take reciprocal of non-zero diagonals
        var wPlus = svd.W.Transpose();
        for (int i = 0; i < Mathf.Min(wPlus.RowCount, wPlus.ColumnCount); i++)
        {
            if (wPlus[i,i] != 0) { wPlus[i, i] = 1 / wPlus[i, i]; }
        }

        var jacobianPseudoinverse = svd.VT.ConjugateTransposeThisAndMultiply(wPlus.Multiply(svd.U.ConjugateTranspose()));

        deltaTheta = jacobianPseudoinverse * e;
        deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
    }
}
