using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;


public class PathFinding
{
    public class Node
    {
        public Vector3 position;
        private float gCost;  // Cost from start
        public float GCost { set; get; }
        private float hCost;  // Heuristic cost to goal
        public float HCost { get; } // Doesn't need to be changed after initialization
        public float FCost => gCost + hCost;  // Total cost
        public Node parent { get; set; }

        public Node(Vector3 position, float gCost, float hCost) // Constructor
        {
            this.position = position;
            this.GCost = gCost;
            this.hCost = hCost;
        }
    }

    public List<Vector3> a_star(Vector3 start_pos, Vector3 goal_pos, ObstacleMap obstacleMap)
    {
        //Convert start and goal into cell vectors
        Vector3 startCell = obstacleMap.WorldToCell(start_pos);
        Vector3 goalCell = obstacleMap.WorldToCell(goal_pos);
        Debug.Log("start cell:" + startCell);
        //Convert start and goal into nodes
        Node startNode = new Node(startCell, 0, getHeuristic(startCell, goalCell));
        Node goalNode = new Node(goalCell, float.MaxValue, 0);

        //Create open and close sets
        List<Node> openList = new List<Node>();
        openList.Add(startNode);
        List<Node> closedList = new List<Node>();

        while (openList.Count > 0)
        {
            Node currentNode = openList.OrderBy(n => n.FCost).First();
            if (currentNode.position == goalNode.position)
            {
                return getPath(currentNode, obstacleMap);
            }

            openList.Remove(currentNode);
            closedList.Add(currentNode);
            Debug.Log("do stuff");

            foreach (var neighborVec in getNeighbors(currentNode.position, obstacleMap, closedList))
            {
                float possible_g = currentNode.GCost + getDistance(currentNode.position, neighborVec);
                Node inTheListNode = openList.Find(node => node.position == neighborVec);
                if (inTheListNode == null)
                {
                    Node neighborNode = new Node(neighborVec, possible_g, getHeuristic(neighborVec, goalNode.position));
                    openList.Add(neighborNode);
                    neighborNode.parent = currentNode;
                }
                else if (possible_g < inTheListNode.GCost)
                {
                    inTheListNode.GCost = possible_g;
                    inTheListNode.parent = currentNode;
                }
            }
        }
        return new List<Vector3>(); // No path has been found
        
    }

    private float getHeuristic(Vector3 position, Vector3 goal) // Flying distance
    {
        float heuristic = Mathf.Abs(position.x - goal.x) + Mathf.Abs(position.z - goal.z);
        return heuristic;
    }

    private List<Vector3> getNeighbors(Vector3 position, ObstacleMap obstacleMap, List<Node> closedList)
    {
        List<Vector3> neighbors = new List<Vector3>();
        List<Vector3> possible_neighbors = new List<Vector3>
        {
            new Vector3(position.x, 0, position.Z+1), //up
            new Vector3(position.x+1, 0, position.z+1), //up-right
            new Vector3(position.x+1, 0, position.z), //right
            new Vector3(position.x+1, 0, position.z-1), //right-down
            new Vector3(position.x, 0, position.z-1), //down
            new Vector3(position.x-1, 0, position.z-1), //down-left
            new Vector3(position.x-1, 0, position.z), //left
            new Vector3(position.x-1, 0, position.z+1) //left-up
        };
        var traversabilityGrid = obstacleMap.traversabilityPerCell;

        Vector3 test = new Vector3(position.x, 0, position.z + 1);
        Debug.Log("neighbor up : " + test);
        Debug.Log("free: " + obstacleMap.GetLocalPointTraversibility(test) == "Free");

        foreach (Vector3 vec in possible_neighbors)
        {
            if (obstacleMap.GetLocalPointTraversibility(vec) == ObstacleMap.Traversability.Free && !closedList.Any(node => node.position == vec))
            {
                neighbors.Add(vec);
            }
        }
        Debug.Log("neighbors: " + neighbors.Count);
        return neighbors;

    }

    private float getDistance(Vector3 vec1, Vector3 vec2) 
    {
        float distance = Mathf.Sqrt(Mathf.Pow(vec1.x - vec2.x, 2) + Mathf.Pow(vec1.z - vec2.z, 2));
        return distance;
    }

    private List<Vector3> getPath(Node goalNode, ObstacleMap obstacleMap)
    {
        List<Vector3> path = new List<Vector3>();
        Node currentNode = goalNode; //Start from the goal
        while (currentNode != null)
        {
            Vector3 worldPosition = new Vector3(currentNode.position.x * obstacleMap.trueScale.x, 0, currentNode.position.y * obstacleMap.trueScale.z); // Calculate real word vector
            path.Add(worldPosition);
            currentNode = currentNode.parent;
        }

        path.Reverse();  // Reverse path to go from start to goal
        return path;
    }

}

