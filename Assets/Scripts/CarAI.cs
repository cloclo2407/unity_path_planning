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

    public Vector3 target_velocity;
    public Vector3 old_target_pos;
    public Vector3 desired_velocity;

    public float k_p = 1f;
    public float k_d = 1f;

    public Rigidbody my_rigidbody;


    private int currentPathIndex = 1;
    private List<Vector3> path;


    private void Start()
    {
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        // get the car controller
        m_Car = GetComponent<CarController>();
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;
        obstacleMap = ObstacleMap.Initialize(mapManager, new List<GameObject>(), Vector3.one * 4);
        my_rigidbody = GetComponent<Rigidbody>();

        //Get starting position
        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        // Get goal position
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();

        pathFinding = new PathFinding();
        this.path = pathFinding.a_star(start_pos, goal_pos, obstacleMap, gameObject.transform);


        // Plan your path here
        //Vector3 someLocalPosition = mapManager.transform.InverseTransformPoint(transform.position); // Position of car w.r.p map coordinate origin (not world global)
        Debug.Log("Path length: " + path.Count);



        // Plot your path to see if it makes sense
        // Note that path can only be seen in "Scene" window, not "Game" window
        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i + 1], Color.black, 1000f); // Green line for path
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


        //Debug.DrawLine(globalPosition, mapManager.GetGlobalStartPosition(), Color.cyan); // Draw in global space
        //Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);


        // Execute your path here
        
        if (currentPathIndex < path.Count)
        {
            Vector3 target_position = path[currentPathIndex];
            target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            old_target_pos = target_position;

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.linearVelocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.linearVelocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.yellow);

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            Debug.Log("accelarion: " + acceleration);
            Debug.Log("steering: " + steering);

            if (Vector3.Distance(target_position,transform.position) <6f)
            {
                currentPathIndex++;
            }
        }
        
        // this is how you control the car
        //m_Car.Move(1f, 1f, 1f, 0f);
    }

    
}