using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Scripts.Vehicle;
using Scripts.Map;
using Imported.StandardAssets.Vehicles.Car.Scripts;


[RequireComponent(typeof(CarController))]
public class CarAI : MonoBehaviour
{
    private CarController m_Car; // the car controller we want to use
    private MapManager mapManager;
    private ObstacleMap obstacleMap;
    private BoxCollider carCollider;

    private void Start()
    {
        carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
        // get the car controller
        m_Car = GetComponent<CarController>();
        mapManager = FindFirstObjectByType<GameManagerA1>().mapManager;
        obstacleMap = FindFirstObjectByType<ObstacleMapVisualizer>().ObstacleMap;


        // Plan your path here
        Vector3 someLocalPosition = mapManager.transform.InverseTransformPoint(transform.position); // Position of car w.r.p map coordinate origin (not world global)
        // transform.localRotation;  Rotation w.r.p map coordinate origin (not world global)

     

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


        // // Check and print traversability of currect position
        Vector3 myLocalPosition = Vector3.zero; //mapManager.grid.WorldToLocal(transform.position); // Position of car w.r.p map coordinate origin (not world global)
        Debug.Log(obstacleMap.IsLocalPointTraversable(myLocalPosition));
    }


    private void FixedUpdate()
    {
        // How to calculate if a physics collider overlaps another.
        var exampleObstacle = obstacleMap.obstacleObjects[0];

        var globalPosition = transform.position;

        bool overlapped = Physics.ComputePenetration(
            carCollider,
            globalPosition,
            transform.rotation, // Use global position 
            exampleObstacle.GetComponent<MeshCollider>(), // Can take any collider and "project" it using position and rotation vectors.
            exampleObstacle.transform.position,
            exampleObstacle.transform.rotation,
            out var direction,
            out var distance
        );
        // 'out's give shortest direction and distance to "uncollide" two objects.
        if (overlapped || distance > 0)
        {
            // Means collider "inside" another, i.e the collider would crash in this position.
        }
        // For more details https:docs.unity3d.com/ScriptReference/Physics.ComputePenetration.html
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