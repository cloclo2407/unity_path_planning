using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Vehicle;
using Scripts.Map;
using UnityEngine.UIElements;
using Imported.StandardAssets.Vehicles;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone;
    private MapManager mapManager;
    private ObstacleMap obstacleMap;
    private BoxCollider droneCollider;

    private PathFindingDrone pathFinding;
    private ImprovePath improvePath;

    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public Vector3 desired_velocity;

    public float K_p = 10f;
    public float K_d = 5f;

    public Rigidbody my_rigidbody;

    private int currentPathIndex = 1;
    private List<Vector3> path;

    private void Start()
    {
        this.enabled = true;
        //droneCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;

        Vector3 grid_size = new Vector3(1, 1, 1) * 2.5f; //can multiply *3 for instance
        obstacleMap = ObstacleMap.Initialize(mapManager, new List<GameObject>(), grid_size);
        my_rigidbody = GetComponent<Rigidbody>();

        //Get starting position
        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        
        // Get goal position
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();

        pathFinding = new PathFindingDrone();
        improvePath = new ImprovePath();
        List<Vector3> first_path = pathFinding.a_star_hybrid(start_pos, goal_pos, obstacleMap, gameObject.transform);

        if (first_path.Count > 0)
        {
            this.path = improvePath.smoothPath(first_path);
        }
        else
        {
            this.path = new List<Vector3>();
        }
        // Plan your path here
        Debug.Log("Path length: " + path.Count);

        float thickness = 0.2f; // Adjust thickness
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
        // this is how you control the drone. Example with an arbitrary sine function.
        //m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);

        var globalPosition = transform.position;

        if (path.Count != 0 && currentPathIndex < path.Count)
        {
            Vector3 target_position = path[currentPathIndex];
            target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            old_target_pos = target_position;

            float myK_p = 10f;
            float myK_d = 8f;

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
            Vector3 desired_acceleration = myK_p * position_error + myK_d * velocity_error;

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