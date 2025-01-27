using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;


public class PathFinding
{
    public class Node
    {
        public Vector3Int position;
        private float gCost;  // Cost from start
        public float GCost { set; get; }
        private float hCost;  // Heuristic cost to goal
        public float HCost { get; } // Doesn't need to be changed after initialization
        public float FCost => gCost + hCost;  // Total cost
        public Node parent { get; set; }

        public Node(Vector3Int position, float gCost, float hCost) // Constructor
        {
            this.position = position;
            this.GCost = gCost;
            this.hCost = hCost;
        }
    }

    public List<Vector3> a_star(Vector3 start_pos, Vector3 goal_pos, ObstacleMap obstacleMap, Transform carTransform)
    {
        //Convert start and goal into cell vectors
        Vector3Int startCell = obstacleMap.WorldToCell(start_pos);
        Vector3Int goalCell = obstacleMap.WorldToCell(goal_pos);
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

            foreach (var neighborVec in getNeighbors(currentNode, obstacleMap, closedList, carTransform))
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

    private float getHeuristic(Vector3Int position, Vector3Int goal) // Flying distance
    {
        float heuristic = Mathf.Abs(position.x - goal.x) + Mathf.Abs(position.z - goal.z);
        return heuristic;
    }

    private List<Vector3Int> getNeighbors(Node currentNode, ObstacleMap obstacleMap, List<Node> closedList, Transform carTransform)
    {
        List<Vector3Int> neighbors = new List<Vector3Int>();

        Vector3Int gridForward;

        if (currentNode.parent == null)
        {
            gridForward = GetClosestGridDirection(carTransform.forward);
        }
        else
        {
            gridForward = (currentNode.position - currentNode.parent.position);
        }

        Vector3Int gridLeft = RotateGridDirection(gridForward, -1);
        Vector3Int gridRight = RotateGridDirection(gridForward, 1);

        List<Vector3Int> possible_neighbors = new List<Vector3Int>
        {
            currentNode.position + gridForward,  // Forward
            currentNode.position + gridLeft,     // Forward-Left
            currentNode.position + gridRight     // Forward-Right
        };

        foreach (Vector3Int vec in possible_neighbors)
        {
            if (obstacleMap.traversabilityPerCell.ContainsKey(new Vector2Int(vec.x, vec.z)))
            {
                var check = obstacleMap.traversabilityPerCell[new Vector2Int(vec.x, vec.z)];
                if (check == ObstacleMap.Traversability.Free && !closedList.Any(node => node.position == vec) && IsFarFromObstacles(vec, obstacleMap))
                {
                    neighbors.Add(vec);
                }
            }
        }
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
            
            Vector3 worldPosition = obstacleMap.CellToWorld(currentNode.position);
            path.Add(worldPosition);
            currentNode = currentNode.parent;
        }

        path.Reverse();  // Reverse path to go from start to goal
        return path;
    }

    private Vector3Int GetClosestGridDirection(Vector3 direction)
    {
        Vector3 directionNorm = direction.normalized;
        List<Vector3Int> gridDirections = new List<Vector3Int>
    {
        new Vector3Int(0, 0, 1),  // Up
        new Vector3Int(1, 0, 1),  // Up-Right
        new Vector3Int(1, 0, 0),  // Right
        new Vector3Int(1, 0, -1), // Down-Right
        new Vector3Int(0, 0, -1), // Down
        new Vector3Int(-1, 0, -1),// Down-Left
        new Vector3Int(-1, 0, 0), // Left
        new Vector3Int(-1, 0, 1)  // Up-Left
    };
        List<float> distances = new List<float>();
        foreach (Vector3Int vec in gridDirections)
        {
            distances.Add(Mathf.Sqrt(Mathf.Pow(vec.x, directionNorm.x) + Mathf.Pow(vec.z,  directionNorm.z)));
        }
        float maxValue = distances.Max();
        int indexOfMax = distances.IndexOf(maxValue);
        return gridDirections[indexOfMax];
    }

    private Vector3Int RotateGridDirection(Vector3Int direction, int rotation)
    {
        List<Vector3Int> gridDirections = new List<Vector3Int>
    {
        new Vector3Int(0, 0, 1),  // Up (0°)
        new Vector3Int(1, 0, 1),  // Up-Right (45°)
        new Vector3Int(1, 0, 0),  // Right (90°)
        new Vector3Int(1, 0, -1), // Down-Right (135°)
        new Vector3Int(0, 0, -1), // Down (180°)
        new Vector3Int(-1, 0, -1),// Down-Left (225°)
        new Vector3Int(-1, 0, 0), // Left (270°)
        new Vector3Int(-1, 0, 1)  // Up-Left (315°)
    };

        int currentIndex = gridDirections.IndexOf(direction);
        if (currentIndex == -1) return direction; // Safety check

        // Rotate left (-1) or right (+1) within bounds
        int newIndex = (currentIndex + rotation + gridDirections.Count) % gridDirections.Count;
        return gridDirections[newIndex];
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

