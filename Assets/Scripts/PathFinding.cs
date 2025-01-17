using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Pathfinding
{

    public class Node
    {
        public Vector2Int position;
        public float gCost;  // Cost from start
        public float hCost;  // Heuristic cost to goal
        public float fCost => gCost + hCost;  // Total cost
        public Node parent;

        public Node(Vector2Int position) // Constructor
        {
            this.position = position;
        }
    }

    private void a_star(Vector3 start_pos, Vector3 goal_pos, Dictionary<Vector2Int, Traversability>& traversabilityGrid)
    {
        //Convert start and goal into 2D vector
        Vector2Int startCell = new Vector2Int((int)(start_pos.x / obstacleMap.trueScale.x), (int)(start_pos.z / obstacleMap.trueScale.z));
        Vector2Int goalCell = new Vector2Int((int)(goal_pos.x / obstacleMap.trueScale.x), (int)(goal_pos.z / obstacleMap.trueScale.z));

        //Convert start and goal into nodes
        Node startNode = new Node(startCell);
        Node goalNode = new Node(goalCell);

        //Create open and close sets
        List<Node> openList = new List<Node>();
        openList.Add(startNode);
        PriorityQueue<Node, int> closed_set = new PriorityQueue<Node, int>();

        while (openList.Count > 0)
        {

        }
    }

}

