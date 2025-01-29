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
        public float GCost { set; get; }
        public float HCost { get; } // Doesn't need to be changed after initialization
        public float FCost => GCost + HCost;  // Total cost
        public Node parent { get; set; }

        public Node(Vector3 position, float orientation, float gCost, float hCost, Node parent=null) // Constructor
        {
            this.position = position;
            this.orientation = orientation;
            this.GCost = gCost;
            this.HCost = hCost;
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

        PriorityQueue<Node> openQueue = new PriorityQueue<Node>(n => n.FCost);
        openQueue.Enqueue(startNode);

        HashSet<Vector3> closedSet = new HashSet<Vector3>();

        Dictionary<Vector3Int, Node> openDict = new Dictionary<Vector3Int, Node>();
        openDict[Vector3Int.RoundToInt(startNode.position)] = startNode;

        while (openQueue.Count > 0)
        {
            Node currentNode = openQueue.Dequeue();  // Get the lowest FCost node
            openDict.Remove(Vector3Int.RoundToInt(currentNode.position));  // Remove from dictionary

            if (Vector3Int.RoundToInt(currentNode.position) == Vector3Int.RoundToInt(goal_pos))
            {
                return getPath(currentNode);
            }


            closedSet.Add(currentNode.position);

            foreach (Node neighbor in getNeighbors(currentNode, goal_pos, obstacleMap))
            {
                if (closedSet.Contains(neighbor.position))
                    continue;

                Vector3Int roundedPos = Vector3Int.RoundToInt(neighbor.position);
                if (openDict.TryGetValue(roundedPos, out Node existingNode))
                {
                    if (neighbor.GCost < existingNode.GCost)  // Better path found
                    {
                        existingNode.GCost = neighbor.GCost;
                        existingNode.parent = currentNode;
                        openQueue.Enqueue(existingNode);  // Reinsert with new cost
                    }
                }
                else
                {
                    openQueue.Enqueue(neighbor);
                    openDict[roundedPos] = neighbor;
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
        float stepSize = 5f; //size of a movement
        float[] angles = { -30, 0, 30 }; // possible directions

        foreach (float angle in angles)
        {
            float newOrientation = (currentNode.orientation + angle) % 360;
            if (newOrientation < 0) newOrientation += 360;

            Vector3 newPos = currentNode.position + stepSize * new Vector3(Mathf.Cos(newOrientation * Mathf.Deg2Rad), 0, Mathf.Sin(newOrientation * Mathf.Deg2Rad));

            if (IsFarFromObstacles(obstacleMap.WorldToCell(newPos), obstacleMap))
            {
                float newCost = currentNode.GCost + stepSize;
                float heuristic = getHeuristic(newPos, goal);
                neighbors.Add(new Node(newPos, newOrientation, newCost, heuristic, currentNode));
            }
        }
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

    public class PriorityQueue<T>
    {
        private List<T> _elements = new List<T>();
        private System.Func<T, float> _getPriority;

        public PriorityQueue(System.Func<T, float> getPriority)
        {
            _getPriority = getPriority;
        }

        public void Enqueue(T item)
        {
            _elements.Add(item);
            BubbleUp(_elements.Count - 1); // Bubble up the new element to its correct position
        }

        public T Dequeue()
        {
            if (_elements.Count == 0)
                throw new System.InvalidOperationException("Queue is empty");

            T root = _elements[0];
            int lastIndex = _elements.Count - 1;
            _elements[0] = _elements[lastIndex];
            _elements.RemoveAt(lastIndex);
            BubbleDown(0); // Restore the heap property

            return root;
        }

        public int Count => _elements.Count;

        private void BubbleUp(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (_getPriority(_elements[index]) < _getPriority(_elements[parentIndex]))
                {
                    Swap(index, parentIndex);
                    index = parentIndex;
                }
                else
                {
                    break;
                }
            }
        }

        private void BubbleDown(int index)
        {
            int leftChildIndex = 2 * index + 1;
            int rightChildIndex = 2 * index + 2;
            int smallestIndex = index;

            if (leftChildIndex < _elements.Count && _getPriority(_elements[leftChildIndex]) < _getPriority(_elements[smallestIndex]))
                smallestIndex = leftChildIndex;

            if (rightChildIndex < _elements.Count && _getPriority(_elements[rightChildIndex]) < _getPriority(_elements[smallestIndex]))
                smallestIndex = rightChildIndex;

            if (smallestIndex != index)
            {
                Swap(index, smallestIndex);
                BubbleDown(smallestIndex);
            }
        }

        private void Swap(int i, int j)
        {
            T temp = _elements[i];
            _elements[i] = _elements[j];
            _elements[j] = temp;
        }
    }

}

