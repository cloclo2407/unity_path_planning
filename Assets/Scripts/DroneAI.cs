using Scripts.Vehicle;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    private DroneController m_Drone;

    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        // See carAI for how to get info from the world.
    }


    private void FixedUpdate()
    {
        // this is how you control the drone. Example with an arbitrary sine function.
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);
    }
}