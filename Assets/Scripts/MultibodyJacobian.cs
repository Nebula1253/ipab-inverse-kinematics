using System;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class MultibodyJacobian : MonoBehaviour
{
    public enum Method
    {
        Transpose,
        Pseudoinverse,
        DampedLeastSquares1,
        DampedLeastSquares2,
        DampedLeastSquares3,
        MultipleObjectivesIK
    }
    
    public GameObject target, com;
    public float errorScale, errorThreshold, dampingMax, manipThreshold, stepSize;
    public bool altJacobian, clampAngles;
    public int maxIterations;
    public Method method;

    private GameObject endEffector;
    private List<Transform> armSegments = new List<Transform>();
    private float dampingConstant = 0;
    private List<Transform> targets = new List<Transform>();
    private List<Transform> endEffectorArms = new List<Transform>();
    private List<Transform> endEffectorPositions = new List<Transform>();
    private int nrEndEffectors = 0, nrJoints = 0;
    private Matrix<float> jacobian, deltaTheta, e;

    // Start is called before the first frame update
    void Start()
    {
        endEffector = new GameObject("endEffector");

        // get transforms of all joints
        foreach (Transform i in GetComponentsInChildren<Transform>())
        {
            if (i.gameObject.name.Contains("Cone"))
            {
                armSegments.Add(i);
            }
        }
        nrJoints = armSegments.Count;
        
        // find which ones are end effectors and get their position
        foreach (Transform arm in armSegments)
        {
            if (arm.childCount == 0 && arm.gameObject.name.Contains("Cone"))
            {
                Vector3 endEffectorPosition = arm.position + (Quaternion.Euler(arm.eulerAngles.x, arm.eulerAngles.y, arm.eulerAngles.z) * new Vector3(arm.GetComponent<MeshFilter>().mesh.bounds.size.x, 0));
                endEffectorPositions.Add(Instantiate(endEffector, arm).transform);
                endEffectorPositions[nrEndEffectors].transform.position = endEffectorPosition;

                targets.Add(Instantiate(target, endEffectorPosition, Quaternion.identity).transform);
                endEffectorArms.Add(arm);
                nrEndEffectors++;
            }
        }

        var M = Matrix<float>.Build;
        deltaTheta = M.Dense(3 * nrJoints, 1);
        e = M.Dense(3 * nrEndEffectors, 1);
    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < nrEndEffectors; i++)
        {
            Vector3 errorVector = errorScale * (endEffectorPositions[i].position - targets[i].position);
            e[0 + (2 * i), 0] = errorVector.x;
            e[1 + (2 * i), 0] = errorVector.y;
            e[2 + (3 * i), 0] = errorVector.z;
        }

        CalculateJacobian();
        switch(method)
        {
            case Method.Transpose:
                Transpose();
                break;
            case Method.Pseudoinverse:
                PseudoInverse();
                break;
            case Method.DampedLeastSquares1:
                DampedLeastSquares1();
                break;
            case Method.DampedLeastSquares2:
                DampedLeastSquares2();
                break;
            case Method.DampedLeastSquares3:
                DampedLeastSquares3();
                break;
            case Method.MultipleObjectivesIK:
                MultipleObjectivesIK();
                break;
        }

        string x = "DeltaTheta:";
        for (int i = 0; i < deltaTheta.RowCount; i++)
        {
            x += deltaTheta[i, 0] + ", ";
        }
        Debug.Log(x);

        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].localEulerAngles += new Vector3(0, 0, deltaTheta[i, 0]);
        }

        com.transform.position = centreOfMass();
    }

    void CalculateJacobian()
    {
        var M = Matrix<float>.Build;
        jacobian = M.Dense(3 * nrEndEffectors, 3 * nrJoints);
        Vector3 XRotation, YRotation, ZRotation;
        Vector3 XAxis, YAxis, ZAxis;
        Vector3 crossAxis;

        for (int endEffector = 0; endEffector < nrEndEffectors; endEffector++)
        {
            for (int joint = 0; joint < nrJoints; joint++)
            {
                // check if current joint affects this effector at all
                if (Array.Exists(armSegments[joint].GetComponentsInChildren<Transform>(), x => x == endEffectorArms[endEffector]))
                {
                    // decide whether to use world coordinate or parent coordinate system (frame of reference for rotation)
                    if (armSegments[joint].parent != null)
                    {
                        XAxis = armSegments[joint].parent.right;
                        YAxis = armSegments[joint].parent.up;
                        ZAxis = armSegments[joint].parent.forward;
                    }
                    else
                    {
                        XAxis = Vector3.right;
                        YAxis = Vector3.up;
                        ZAxis = Vector3.forward;
                    }

                    // alt jacobian: set targets closer to end effectors as opposed to other way around
                    if (!altJacobian) { crossAxis = endEffectorPositions[endEffector].position - armSegments[joint].position; }
                    else { crossAxis = targets[endEffector].position - armSegments[joint].position; }

                    XRotation = Vector3.Cross(XAxis, crossAxis);
                    YRotation = Vector3.Cross(YAxis, crossAxis);
                    ZRotation = Vector3.Cross(ZAxis, crossAxis);
                }
                // if it doesn't, the jacobian values are obviously 0
                else {
                    XRotation = Vector3.zero;
                    YRotation = Vector3.zero;
                    ZRotation = Vector3.zero; 
                }

                // effect of joint's x-axis rotation on end-effector's location
                jacobian[0 + (3 * endEffector), (3 * joint)] = XRotation.x;
                jacobian[1 + (3 * endEffector), (3 * joint)] = XRotation.y;
                jacobian[2 + (3 * endEffector), (3 * joint)] = XRotation.z;

                // effect of joint's y-axis rotation on end-effector's location
                jacobian[0 + (3 * endEffector), (3 * joint) + 1] = YRotation.x;
                jacobian[1 + (3 * endEffector), (3 * joint) + 1] = YRotation.y;
                jacobian[2 + (3 * endEffector), (3 * joint) + 1] = YRotation.z;

                // effect of joint's z-axis rotation on end-effector's location
                jacobian[0 + (3 * endEffector), (3 * joint) + 2] = ZRotation.x;
                jacobian[1 + (3 * endEffector), (3 * joint) + 2] = ZRotation.y;
                jacobian[2 + (3 * endEffector), (3 * joint) + 2] = ZRotation.z;
            }
        }

        // print entirety of jacobian contents
        string x = "Jacobian: \n";
        for (int r = 0; r < jacobian.RowCount; r++)
        {
            for (int c = 0; c < jacobian.ColumnCount; c++)
            {
                x += Math.Round(jacobian[r, c], 2) + ", ";
            }
            x += "\n";
        }
        Debug.Log(x);
    }

    void Transpose()
    {
        var jjte = (jacobian * jacobian.Transpose() * e).Column(0);
        
        if (e.Column(0).L2Norm() >= errorThreshold)
        {
            float alpha = (float)(jjte.DotProduct(e.Column(0)) / (jjte.L2Norm() * jjte.L2Norm()));

            deltaTheta = alpha * jacobian.Transpose() * e;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
        }

        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].localEulerAngles += new Vector3(deltaTheta[3 * i, 0], deltaTheta[(3 * i) + 1, 0], deltaTheta[(3 * i) + 2, 0]);
        }
    }

    void PseudoInverse()
    {
        var svd = jacobian.Svd();
        // have to modify W (transpose and take reciprocal of non-zero diagonals)
        var wPlus = svd.W.Transpose();
        for (int i = 0; i < Mathf.Min(wPlus.RowCount, wPlus.ColumnCount); i++)
        {
            if (wPlus[i, i] != 0) { wPlus[i, i] = 1 / wPlus[i, i]; }
        }

        var jacobianPseudoinverse = svd.VT.ConjugateTransposeThisAndMultiply(wPlus.Multiply(svd.U.ConjugateTranspose()));

        //var jacobianPseudoinverse = jacobian.Transpose() * (jacobian * jacobian.Transpose()).Inverse();

        if (e.Column(0).L2Norm() >= errorThreshold)
        {
            deltaTheta = jacobianPseudoinverse * e;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
        }

        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].localEulerAngles += new Vector3(deltaTheta[3 * i, 0], deltaTheta[(3 * i) + 1, 0], deltaTheta[(3 * i) + 2, 0]);
        }
    }

    void DampedLeastSquares1()
    {
        dampingConstant = 0;

        // calculate damping constant
        float manipulability = Mathf.Sqrt((jacobian * jacobian.Transpose()).Determinant());

        if (manipulability != 0) { Debug.Log(manipulability); }

        if (manipulability < manipThreshold)
        {
            dampingConstant = dampingMax * (1 - (manipulability / manipThreshold)) * (1 - (manipulability / manipThreshold));
        }

        var M = Matrix<float>.Build;

        if (e.Column(0).L2Norm() >= errorThreshold)
        {
            deltaTheta = jacobian.Transpose() * (jacobian * jacobian.Transpose() + (dampingConstant * dampingConstant) * M.DenseIdentity(jacobian.RowCount)).Inverse() * e;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
        }
        else { deltaTheta = M.Dense(3*nrJoints, 1); }

        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].localEulerAngles += new Vector3(deltaTheta[3 * i, 0], deltaTheta[(3 * i) + 1, 0], deltaTheta[(3 * i) + 2, 0]);
        }
    }

    void DampedLeastSquares2()
    {
        // get manip measure of previous iteration
        float manipKminus1 = Mathf.Sqrt((jacobian * jacobian.Transpose()).Determinant());

        var M = Matrix<float>.Build;

        // move arm
        if (e.Column(0).L2Norm() >= errorThreshold)
        {
            deltaTheta = jacobian.Transpose() * (jacobian * jacobian.Transpose() + (dampingConstant * dampingConstant) * M.DenseIdentity(jacobian.RowCount)).Inverse() * e;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
        }

        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].localEulerAngles += new Vector3(0, 0, deltaTheta[i, 0]);
        }

        CalculateJacobian();

        // get manip measure of current iteration
        float manipK = Mathf.Sqrt((jacobian * jacobian.Transpose()).Determinant());

        // calculate damping constant
        if (manipKminus1 != 0 && ((manipK / manipKminus1) < manipThreshold))
        {
            dampingConstant = dampingMax * (1 - (manipK / manipKminus1));
        }
        else { dampingConstant = 0; }
    }

    void DampedLeastSquares3()
    {
        var nrIterations = 0;
        while (e.Column(0).L2Norm() >= errorThreshold && nrIterations <= maxIterations)
        {
            dampingConstant = dampingMax * (float)(e.Column(0).L2Norm() * e.Column(0).L2Norm());

            var M = Matrix<float>.Build;
            deltaTheta = jacobian.Transpose() * (jacobian * jacobian.Transpose() + (dampingConstant * dampingConstant) * M.DenseIdentity(jacobian.RowCount)).Inverse() * e;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
            deltaTheta *= stepSize;

            //string x = "Delta Theta: \n";
            //for (int i = 0; i < deltaTheta.RowCount; i++)
            //{
            //    x += deltaTheta[i, 0] + ", ";
            //    if ((i + 1) % 3 == 0) { x += "\n"; }
            //}
            //Debug.Log(x);

            for (int i = 0; i < nrJoints; i++)
            {
                armSegments[i].localEulerAngles += new Vector3(deltaTheta[3 * i, 0], deltaTheta[(3 * i) + 1, 0], deltaTheta[(3 * i) + 2, 0]);
                //Debug.Log((i, armSegments[i].localEulerAngles));
            }


            for (int i = 0; i < nrEndEffectors; i++)
            {
                Vector3 errorVector = errorScale * (endEffectorPositions[i].position - targets[i].position);
                e[0 + (2 * i), 0] = errorVector.x;
                e[1 + (2 * i), 0] = errorVector.y;
                e[2 + (3 * i), 0] = errorVector.z;
            }
            nrIterations++;
        }
    }

    void MultipleObjectivesIK()
    {
        var Jplus = jacobian.PseudoInverse();

        var M = Matrix<float>.Build;
        var nullspaceProjector = M.DenseIdentity(Jplus.RowCount) - (Jplus * jacobian);
        bool[] jointFree = new bool[nrJoints * 3];

        // initialise joint state array
        for (int i = 0; i < nrJoints * 3; i++) { jointFree[i] = true; }

        dampingConstant = dampingMax * (float)(e.Column(0).L2Norm() * e.Column(0).L2Norm());

        // here delta alpha is used to keep angles close to 0 (assuming all gain values are 1)
        //var deltaAlpha = deltaTheta;

        // second jacobian is used for task "keep 4th joint as close to x = 20 as possible"
        var jacobian2 = M.Dense(1, 3*nrJoints);
        Vector3 XRotation, YRotation, ZRotation;
        Vector3 XAxis, YAxis, ZAxis;
        Vector3 crossAxis;

        for (int joint = 0; joint < nrJoints; joint++)
        {
            crossAxis = new Vector3(20, 0) - armSegments[joint].position;
            if (Array.Exists(armSegments[joint].GetComponentsInChildren<Transform>(), x => x == armSegments[3]))
            {
                if (armSegments[joint].parent != null)
                {
                    XAxis = armSegments[joint].parent.right;
                    YAxis = armSegments[joint].parent.up;
                    ZAxis = armSegments[joint].parent.forward;
                }
                else
                {
                    XAxis = Vector3.right;
                    YAxis = Vector3.up;
                    ZAxis = Vector3.forward;
                }

                XRotation = Vector3.Cross(XAxis, crossAxis);
                YRotation = Vector3.Cross(YAxis, crossAxis);
                ZRotation = Vector3.Cross(ZAxis, crossAxis);
            }
            else {
                XRotation = Vector3.zero;
                YRotation = Vector3.zero;
                ZRotation = Vector3.zero;
            }

            // get the change in x values that a rotation around each axis would cause (or at least an approximation of that)
            jacobian2[0, (3 * joint)] = XRotation.x;
            jacobian2[0, (3 * joint) + 1] = YRotation.x;
            jacobian2[0, (3 * joint) + 2] = ZRotation.x;
        }
        
        var e2 = M.Dense(1, 1);
        
        Vector3 errorVector = errorScale * (armSegments[3].position - new Vector3(20,0));
        e2[0, 0] = errorVector.x; 

        var jacobian1DampedPseudoinverse = jacobian.Transpose() * (jacobian * jacobian.Transpose() + (dampingConstant * dampingConstant) * M.DenseIdentity(jacobian.RowCount)).PseudoInverse();

        var dampingConstant2 = dampingMax * (float)(e2.Column(0).L2Norm() * e2.Column(0).L2Norm());
        var temp2 = jacobian2 * nullspaceProjector;
        var jacobian2DampedPseudoinverse = temp2.Transpose() * (temp2 * temp2.Transpose() + (dampingConstant2 * dampingConstant2) * M.DenseIdentity(temp2.RowCount)).PseudoInverse();

        if (e.Column(0).L2Norm() >= errorThreshold || e2.Column(0).L2Norm() >= errorThreshold)
        {
            deltaTheta = (jacobian1DampedPseudoinverse * e) /*+ (jacobian2DampedPseudoinverse * (e2 - (jacobian2 * (jacobian1DampedPseudoinverse * e))))*/;
            deltaTheta.MapInplace(x => x * (180 / Mathf.PI) * -1);
            //deltaTheta += nullspaceProjector * deltaAlpha;
            deltaTheta *= stepSize;
        }
        else { deltaTheta = M.Dense(nrJoints, 1); }

        // angle assignment
        for (int i = 0; i < nrJoints; i++)
        {
            armSegments[i].localEulerAngles += new Vector3(deltaTheta[3 * i, 0], deltaTheta[(3 * i) + 1, 0], deltaTheta[(3 * i) + 2, 0]);
        }

        if (clampAngles)
        {
            // clamping loop
            for (int i = 0; i < nrJoints * 3; i++)
            {
                if (jointFree[i])
                {
                    var theta = armSegments[(int) Math.Floor(i / 3f)].localEulerAngles;

                    for (int k = 0; k < 3; k++)
                    {
                        if (theta[k] > 180) { theta[k] -= 360; }

                        if (Mathf.Abs(theta[k]) > 90)
                        {
                            e -= jacobian.Column(i + k).ToColumnMatrix() * (Mathf.Abs(theta[k]));

                            Vector3 newAngles = new Vector3();
                            for (int a = 0; a < 3; a++)
                            {
                                if (a==k) { newAngles[a] = 90 * (theta[k] / Mathf.Abs(theta[k])); }
                                else { newAngles[a] = theta[a]; }
                            }
                            armSegments[(int)Math.Floor(i / 3f)].localEulerAngles = newAngles;

                            for (int b = 0; b < jacobian.RowCount; b++)
                            {
                                jacobian[b, i + k] = 0;
                            }

                            nullspaceProjector[i + k, i + k] = 0;
                            jointFree[i] = false;
                        }
                    }
                }
            }
        }
    }

    // conveniently works for 3d as well yay!!!!
    Vector3 centreOfMass()
    {
        var temp = Vector3.zero;
        for (int i = 0; i < nrJoints; i++)
        {
            var arm = armSegments[i];
            temp += arm.position + (Quaternion.Euler(arm.eulerAngles.x, arm.eulerAngles.y, arm.eulerAngles.z) * new Vector3(arm.GetComponent<MeshFilter>().mesh.bounds.size.x / 2, 0));
        }
        temp /= nrJoints;
        return temp;
    }

    public List<Transform> getTargets() { return targets; }
}
