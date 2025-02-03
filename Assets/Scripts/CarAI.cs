using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Vehicle;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;
using UnityEngine.UIElements;


[RequireComponent(typeof(CarController))]
public class CarAI : MonoBehaviour
{
    // An example class, containing code snippets demonstrating how to do different things in the environment.

    private CarController m_Car; // the car controller we want to use
    private MapManager mapManager;
    private ObstacleMap obstacleMap;
    private BoxCollider carCollider;

    private PathFinding pathFinding;
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
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        // get the car controller
        m_Car = GetComponent<CarController>();
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;

        Vector3 grid_size = new Vector3(1,1,1)*1.6f; //can multiply *3 for instance
        obstacleMap = ObstacleMap.Initialize(mapManager, new List<GameObject>(), grid_size);
        my_rigidbody = GetComponent<Rigidbody>();

        //k_p = 2f;
        //k_d = 1.5f;

        //Get starting position
        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        // Get goal position
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();

        pathFinding = new PathFinding();
        improvePath = new ImprovePath();
        List<Vector3> first_path = pathFinding.a_star_hybrid(start_pos, goal_pos, obstacleMap, gameObject.transform);

        if (first_path.Count > 0)
        {
            //this.path = improvePath.smoothPath(first_path); // doesn't work well
            this.path = improvePath.smoothPath(first_path);
        }
        else
        {
            this.path = new List<Vector3>();
        }
        // Plan your path here
        Debug.Log("Path length: " + path.Count);



        // Plot your path to see if it makes sense
        // Note that path can only be seen in "Scene" window, not "Game" window
        

        float thickness = 0.1f; // Adjust thickness
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
            //Debug.DrawLine(first_path[i] + offset, first_path[i + 1] + offset, Color.red, 10000f);
            //Debug.DrawLine(first_path[i] - offset, first_path[i + 1] - offset, Color.red, 10000f);
        }

        /*
        RaycastHit hit;
        float maxRange = 500f;
        if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
        {
            Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
            Debug.Log("Did Hit");
        }

        // Raycast works as long as the objects are loaded in before hitting Play.
        // I.e if the map is loaded in from inspector before starting.
        var transformPosition = transform.position + transform.forward * 20;
        var transformDirection = transform.TransformDirection(Vector3.left);
        if (Physics.Raycast(transformPosition, transformDirection, out hit, maxRange))
        {
            Vector3 closestObstacleInFront = transformDirection * hit.distance;
            Debug.DrawRay(transformPosition, closestObstacleInFront, Color.red);
            Debug.Log("Did Hit");
        }*/
    }


    private void FixedUpdate()
    {
        var globalPosition = transform.position;

        //var localPointTraveribility = obstacleMap?.GetLocalPointTraversibility(transform.localPosition);
        //var globalPointTravesibility = obstacleMap?.GetGlobalPointTravesibility(transform.position);

        // Execute your path here
        
        if (path.Count !=0 && currentPathIndex < path.Count)
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

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.linearVelocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.yellow);

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            

            if (Vector3.Distance(target_position,transform.position) <6f)
            {
                currentPathIndex++;
            }
        }
    }

    
}
