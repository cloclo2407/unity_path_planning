using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Vehicle;
using Scripts.Map;
using UnityEngine.UIElements;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    private DroneController m_Drone; // the car controller we want to use
    private MapManager mapManager;
    private ObstacleMap obstacleMap;
    private BoxCollider carCollider;

    private PathFindingDrone pathFindingDrone;
    private ImprovePath improvePath;

    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public Vector3 desired_velocity;

    public float k_p = 5f;
    public float k_d = 5f;

    public Rigidbody my_rigidbody;


    private int currentPathIndex = 1;
    private List<Vector3> path;


    private void Start()
    {
        m_Drone = GetComponent<DroneController>();
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;

        Vector3 grid_size = new Vector3(1, 1, 1) * 2.4f; //can multiply *3 for instance
        obstacleMap = ObstacleMap.Initialize(mapManager, new List<GameObject>(), grid_size);
        my_rigidbody = GetComponent<Rigidbody>();

        //Get starting position
        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        // Get goal position
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();

        pathFindingDrone = new PathFindingDrone();
        improvePath = new ImprovePath();
        List<Vector3> first_path = pathFindingDrone.a_star(start_pos, goal_pos, obstacleMap, gameObject.transform);

        if (first_path.Count > 0)
        {
            float epsilon = 0.2f;
            //this.path = improvePath.smoothPath(first_path);
            this.path = improvePath.simplifyPath(first_path, epsilon);
        }
        else
        {
            this.path = new List<Vector3>();
        }
        // Plan your path here
        Debug.Log("Path length: " + path.Count);

        float thickness = 0.3f; // Adjust thickness
        Vector3 offset = Vector3.up * thickness;
        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i + 1], Color.cyan, 10000f);
            Debug.DrawLine(path[i] + offset, path[i + 1] + offset, Color.cyan, 10000f);
            Debug.DrawLine(path[i] - offset, path[i + 1] - offset, Color.cyan, 10000f);
        }
        for (int i = 0; i < first_path.Count - 1; i++)
        {
            Debug.DrawLine(first_path[i], first_path[i + 1], Color.red, 10000f);
        }
    }


    private void FixedUpdate()
    {
        var globalPosition = transform.position;

        if (path.Count != 0 && currentPathIndex < path.Count)
        {
            Vector3 target_position = path[currentPathIndex];
            target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            old_target_pos = target_position;

            float distance = Vector3.Distance(target_position, transform.position);

            // Scale k_p and k_d based on distance  between 1 and 10
            float scaleFactor = Mathf.Clamp(distance / 5f, 2f, 8f);  // Adjust 5f to control sensitivity
            float k_p_dynamic = Mathf.Lerp(2f, 10f, scaleFactor / 10f);
            float k_d_dynamic = Mathf.Lerp(2f, 8f, scaleFactor / 8f);

            float k_v = Mathf.Lerp(1f, 2f, scaleFactor / 8f);  // New gain factor for velocity feedback
            Vector3 velocity_damping = -k_v * my_rigidbody.linearVelocity;

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
            Vector3 desired_acceleration = k_p_dynamic * position_error + k_d_dynamic * velocity_error + velocity_damping;

            m_Drone.Move(desired_acceleration.x, desired_acceleration.z); 

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.linearVelocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.yellow);

            if (Vector3.Distance(target_position, transform.position) < 3f)
            {
                currentPathIndex++;
            }
        }
    }


}