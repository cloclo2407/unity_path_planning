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

        // Get traversibility grid (Dictionary<Vector2Int, Traversability>)
        var traversabilityGrid = obstacleMap.traversabilityPerCell;

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


        // Replace the code below that makes a random path
        // ...

        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();

        List<Vector3> my_path = new List<Vector3>();

        my_path.Add(start_pos);

        for (int i = 0; i < 3; i++)
        {
            Vector3 waypoint = new Vector3(
                UnityEngine.Random.Range(obstacleMap.localBounds.min.x, obstacleMap.localBounds.max.x), 0,
                UnityEngine.Random.Range(obstacleMap.localBounds.min.z, obstacleMap.localBounds.max.z));
            my_path.Add(waypoint);
        }

        my_path.Add(goal_pos);


        // Plot your path to see if it makes sense
        // Note that path can only be seen in "Scene" window, not "Game" window
        Vector3 old_wp = start_pos;
        foreach (var wp in my_path)
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

        // How to calculate if something intersects the location of a box
        var overlapped = Physics.CheckBox(
            center: new Vector3(3f, 0f, 3f), // Global position to check
            halfExtents: new Vector3(1f, 0.1f, 1f) // Size of box (+- Vec in each direction)
        );

        // 'out's give shortest direction and distance to "uncollide" two objects.
        if (overlapped)
        {
            // Do your thing
        }
        // For more details https:docs.unity3d.com/ScriptReference/Physics.CheckBox.html
        // The Physics class has a bunch of static classes for all kinds of checks.
        ///////////////////////////

        // // This is how you access information about the terrain from a simulated laser range finder
        // // It might be wise to use this for error recovery, but do most of the planning before the race clock starts
        RaycastHit hit;
        float maxRange = 50f;
        if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
        {
            Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);
            //   Debug.Log("Did Hit");
        }

        Debug.DrawLine(globalPosition, mapManager.GetGlobalStartPosition(), Color.cyan); // Draw in global space
        Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);


        // Execute your path here
        // ...

        
        // this is how you control the car
        m_Car.Move(1f, 1f, 1f, 0f);
    }

    
}