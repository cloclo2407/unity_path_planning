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


    private void Start()
    {
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        // get the car controller
        m_Car = GetComponent<CarController>();
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;
        obstacleMap = ObstacleMap.Initialize(mapManager, new List<GameObject>(), Vector3.one * 4);

        //Get starting position and transform it into a grid cell
        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        Vector3Int startCell = obstacleMap.WorldToCell(start_pos);

        // Get goal position and transform it into a grid cell
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();
        Vector3Int goalCell = obstacleMap.WorldToCell(goal_pos);

        pathFinding = new PathFinding();
        List<Vector3> path = pathFinding.a_star(start_pos, goal_pos, obstacleMap);


        // Plan your path here
        Vector3 someLocalPosition = mapManager.transform.InverseTransformPoint(transform.position); // Position of car w.r.p map coordinate origin (not world global)


        // Plot your path to see if it makes sense
        // Note that path can only be seen in "Scene" window, not "Game" window
        Vector3 old_wp = start_pos;
        foreach (var wp in path)
        {
            //Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
            old_wp = wp;
        }

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
        }
    }


    private void FixedUpdate()
    {
        var globalPosition = transform.position;

        var localPointTraveribility = obstacleMap?.GetLocalPointTraversibility(transform.localPosition);
        var globalPointTravesibility = obstacleMap?.GetGlobalPointTravesibility(transform.position);


        Debug.DrawLine(globalPosition, mapManager.GetGlobalStartPosition(), Color.cyan); // Draw in global space
        Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);


        // Execute your path here
        target_position = my_target.transform.position;
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
        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

        // this is how you control the car
        Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
        m_Car.Move(steering, acceleration, acceleration, 0f);



        // this is how you control the car
        m_Car.Move(1f, 1f, 1f, 0f);
    }

    
}