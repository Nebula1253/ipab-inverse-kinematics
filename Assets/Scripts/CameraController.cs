using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    float rotationX = 0f, rotationY = 0f;
    public float rotationSensitivity, zoomSensitivity, panSensitivity;
    private Vector3 lastPosition = new Vector3(0, 0);
    private bool draggingObj;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        // rotation
        if (Input.GetMouseButton(1))
        {
            rotationY += Input.GetAxis("Mouse X") * rotationSensitivity;
            rotationX += Input.GetAxis("Mouse Y") * rotationSensitivity * -1;
            transform.localEulerAngles = new Vector3(rotationX, rotationY, 0);
        }

        // zooming
        if (Input.mouseScrollDelta.y > 0) { transform.position += transform.forward * zoomSensitivity; }
        else if (Input.mouseScrollDelta.y < 0) { transform.position += transform.forward * -1 * zoomSensitivity; }

        // panning
        if (!draggingObj)
        {
            if (Input.GetMouseButtonDown(0)) { lastPosition = Input.mousePosition; }
            if (Input.GetMouseButton(0))
            {
                var delta = (Input.mousePosition - lastPosition) * -1 * panSensitivity;
                transform.Translate(new Vector3(delta.x, delta.y, 0));
                lastPosition = Input.mousePosition;
            }
        }

    }

    public void setDrag(bool value) { draggingObj = value; }
}
