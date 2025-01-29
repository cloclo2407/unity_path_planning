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
        public float orientation;
        private float gCost;  // Cost from start
        public float GCost { set; get; }
        private float hCost;  // Heuristic cost to goal
        public float HCost { get; } // Doesn't need to be changed after initialization
        public float FCost => gCost + hCost;  // Total cost
        public Node parent { get; set; }

        public Node(Vector3 position, float orientation, float gCost, float hCost, Node parent=null) // Constructor
        {
            this.position = position;
            this.orientation = orientation;
            this.GCost = gCost;
            this.hCost = hCost;
            this.parent = parent;
        }
    }

    public List<Vector3> a_star_hybrid(Vector3 start_pos, Vector3 goal_pos, ObstacleMap obstacleMap, Transform carTransform)
    {
        //Convert start and goal into cell vectors
        Vector3Int startCell = obstacleMap.WorldToCell(start_pos);
        Vector3Int goalCell = obstacleMap.WorldToCell(goal_pos);
        //Convert start and goal into nodes
        Node startNode = new Node(start_pos, carTransform.eulerAngles.y, 0, getHeuristic(startCell, goalCell));

        //Create open and close sets
        SortedSet<Node> openList = new SortedSet<Node>(Comparer<Node>.Create((a, b) =>
        {
            int compare = a.FCost.CompareTo(b.FCost);
            return compare == 0 ? a.GCost.CompareTo(b.GCost) : compare;
        }));
        openList.Add(startNode);
        HashSet<Vector3> closedSet = new HashSet<Vector3>();

        while (openList.Count > 0)
        {
            Node currentNode = openList.First();
            if (Vector3.Distance(currentNode.position, goal_pos) < 10f)
            {
                return getPath(currentNode);
            }

            openList.Remove(currentNode);
            closedSet.Add(currentNode.position);

            foreach (Node neighbor in getNeighbors(currentNode, goal_pos, obstacleMap))
            {
                if (closedSet.Contains(neighbor.position))
                    continue;

                if (openList.Contains(neighbor))
                {
                    // If this new path to the neighbor is better, update its costs
                    Node existingNode = openList.FirstOrDefault(n => n.position == neighbor.position);
                    if (existingNode != null && neighbor.GCost < existingNode.GCost)
                    {
                        existingNode.GCost = neighbor.GCost;
                        existingNode.parent = currentNode;
                    }
                }
                else
                {
                    openList.Add(neighbor);
                    Debug.Log($"Adding node: {neighbor.position}, FCost: {neighbor.FCost}, GCost: {neighbor.GCost}, HCost: {neighbor.HCost}");

                }

            }
        }
        return new List<Vector3>(); // No path has been found
        
    }

    private float getHeuristic(Vector3 position, Vector3 goal) // Flying distance
    {
        return Vector3.Distance(position, goal);
    }

    private List<Node> getNeighbors(Node currentNode, Vector3 goal, ObstacleMap obstacleMap)
    {
        List<Node> neighbors = new List<Node>();
        float stepSize = 1f; //size of a movement
        float[] angles = { -45, -25, 0, 25, 45 }; // possible directions

        foreach (float angle in angles)
        {
            Debug.Log("looking for neighbours");
            float newOrientation = currentNode.orientation + angle;
            Vector3 newPos = currentNode.position + stepSize * new Vector3(Mathf.Cos(newOrientation * Mathf.Deg2Rad), 0, Mathf.Sin(newOrientation * Mathf.Deg2Rad));

            if (!IsFarFromObstacles(obstacleMap.WorldToCell(newPos), obstacleMap))
            {
                float newCost = currentNode.GCost + stepSize;
                float heuristic = getHeuristic(newPos, goal);
                neighbors.Add(new Node(newPos, newOrientation, newCost, heuristic, currentNode));
            }
        }
        Debug.Log(neighbors.Count);
        return neighbors;

    }

    private List<Vector3> getPath(Node goalNode)
    {
        List<Vector3> path = new List<Vector3>();
        Node currentNode = goalNode; //Start from the goal
        while (currentNode != null)
        {
            path.Add(currentNode.position);
            currentNode = currentNode.parent;
        }
        path.Reverse();  // Reverse path to go from start to goal
        return path;
    }

    private bool IsFarFromObstacles(Vector3Int cell, ObstacleMap obstacleMap)
    {
        float minDistance = 2f;

        List<Vector3Int> surroundingCells = new List<Vector3Int>
    {
        new Vector3Int(cell.x + 1, 0, cell.z),// Right
        new Vector3Int(cell.x - 1, 0, cell.z),// Left
        new Vector3Int(cell.x, 0, cell.z + 1),// Forward
        new Vector3Int(cell.x, 0, cell.z - 1),// Back
        new Vector3Int(cell.x + 1, 0, cell.z + 1),// Forward-right
        new Vector3Int(cell.x - 1, 0, cell.z + 1),// Forward-left
        new Vector3Int(cell.x + 1, 0, cell.z - 1),// Back-right
        new Vector3Int(cell.x - 1, 0, cell.z - 1)// Back-left
    };

        foreach (Vector3Int neighbor in surroundingCells)
        {
            if (obstacleMap.traversabilityPerCell.TryGetValue(new Vector2Int(neighbor.x, neighbor.z), out var traversability) &&
                traversability != ObstacleMap.Traversability.Free) 
            {
                float distance = Vector2.Distance(new Vector2(cell.x, cell.z), new Vector2(neighbor.x, neighbor.z));

                if (distance < minDistance)
                {
                    return false; 
                }
            }
        }

        return true;
    }

}

