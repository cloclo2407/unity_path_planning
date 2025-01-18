using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;


public class PathFinding
{
    public class Node
    {
        public Vector2Int position;
        private float gCost;  // Cost from start
        public float GCost { set; get; }
        private float hCost;  // Heuristic cost to goal
        public float HCost { get; } // Doesn't need to be changed after initialization
        public float FCost => gCost + hCost;  // Total cost
        public Node parent { get; set; }

        public Node(Vector2Int position, float gCost, float hCost) // Constructor
        {
            this.position = position;
            this.GCost = gCost;
            this.hCost = hCost;
        }
    }

    public List<Vector2Int> a_star(Vector3 start_pos, Vector3 goal_pos, ObstacleMap obstacleMap)
    {
        //Convert start and goal into 2D vector
        Vector2Int startCell = new Vector2Int((int)(start_pos.x / obstacleMap.trueScale.x), (int)(start_pos.z / obstacleMap.trueScale.z));
        Vector2Int goalCell = new Vector2Int((int)(goal_pos.x / obstacleMap.trueScale.x), (int)(goal_pos.z / obstacleMap.trueScale.z));

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

            foreach (var neighborVec in getNeighbors(currentNode.position, obstacleMap, closedList))
            {
                float possible_g = currentNode.GCost + getDistance(currentNode, neighborVec);
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

    private float getHeuristic(Vector2Int position, Vector2Int goal) // Flying distance
    {
        float heuristic = Mathf.Abs(position.x - goal.x) + Mathf.Abs(position.y - goal.y);
        return heuristic;
    }

    private List<Vector2Int> getNeighbors(Vector2Int position, ObstacleMap obstacleMap, List<Node> closedList)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();
        List<Vector2Int> possible_neighbors = new List<Vector2Int>
        {
            new Vector2Int(position.x, position.y+1), //up
            new Vector2Int(position.x+1, position.y+1), //up-right
            new Vector2Int(position.x+1, position.y), //right
            new Vector2Int(position.x+1, position.y-1), //right-down
            new Vector2Int(position.x, position.y-1), //down
            new Vector2Int(position.x-1, position.y-1), //down-left
            new Vector2Int(position.x-1, position.y), //left
            new Vector2Int(position.x-1, position.y+1) //left-up
        };

        var traversabilityGrid = obstacleMap.traversabilityPerCell;
        foreach (Vector2Int vec in possible_neighbors)
        {
            if (traversabilityGrid.ContainsKey(vec) && traversabilityGrid[vec] == 0 && !closedList.Any(node => node.position == vec))
            {
                neighbors.Add(vec);
            }
        }
        return neighbors;

    }

    private float getDistance(Node node1, Node node2) 
    {
        float distance = Mathf.Sqrt(Mathf.Pow(node1.position.x - node2.position.x, 2) + Mathf.Pow(node1.position.y - node2.position.y, 2));
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

