using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Pathfinding
{
    public class Node
    {
        public Vector2Int position;
        private float gCost { get; set; };  // Cost from start
        private float hCost { get; set; };  // Heuristic cost to goal
        private float fCost { get; set; } => gCost + hCost;  // Total cost
        private Node parent { get; set; };

        public Node(Vector2Int position, int gCost, int hCost) // Constructor
        {
            this.position = position;
            this.gCost = gCost;
            this.hCost = hCost;
        }
    }

    private List<Vector2Int> a_star(Vector3 start_pos, Vector3 goal_pos, Dictionary<Vector2Int, Traversability>& traversabilityGrid)
    {
        //Convert start and goal into 2D vector
        Vector2Int startCell = new Vector2Int((int)(start_pos.x / obstacleMap.trueScale.x), (int)(start_pos.z / obstacleMap.trueScale.z));
        Vector2Int goalCell = new Vector2Int((int)(goal_pos.x / obstacleMap.trueScale.x), (int)(goal_pos.z / obstacleMap.trueScale.z));

        //Convert start and goal into nodes
        Node startNode = new Node(startCell, 0, );
        Node goalNode = new Node(goalCell, , 0);

        //Create open and close sets
        List<Node> openList = new List<Node>();
        openList.Add(startNode);
        List<Node> closedList = new List<Node>();

        while (openList.Count > 0)
        {
            Node currentNode = openList.OrderBy(Node => fCost).First();
            if (currentNode.positiond == goalNode.positions)
            {
                return ConstructPath(currentNode, traversabilityGrid);
            }

            openList.Remove(currentNode);
            closed_set.Enqueue(currentNode);
        }
    }

    private float getHeuristic(Vector2Int position, Vector2Int goal) // Flying distance
    {
        float heuristic = Mathf.abs(position.x - goal.x) + Mathf.Abs(position.y - goal.y);
        return heuristic;
    }

    private List<Vector2Int> getNeighbors(Vector2Int position, Dictionary<Vector2Int, Traversability>& traversabilityGrid)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>;
        List<Vector2Int> possible_neigbord = new List<Vector2Int>
        {
            new Vector2Int(position.x, position.y+1), //up
            new Vector2Int(position.x+1, position.y+1), //up-right
            new Vector2Int(position.x+1, position.y), //right
            new Vector2Int(position.x+1, position.y-1), //right-down
            new Vector2Int(position.x, position.y-1), //down
            new Vector2Int(position.x-1, position.y-1), //down-left
            new Vector2Int(position.x-1, position.y), //left
            new Vector2Int(position.x-1, position.y+1) //left-up
        }

        for (Vector2Int vec in possible_neigbord)
        {
            if (traversibilityGrid[vec] == 0)
            {
                neighbors.Add(vec);
            }
        }
    return neighbors;

    }

}

