using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;


public class ImprovePath
{
    public List<Vector3> smoothPath(List<Vector3> path) // Create longer path smoother
    {
        //path = simplifyPath(path, 0.02f);
        List<Vector3> smoothedPath = new List<Vector3>();
        int n=path.Count;
        for (int i = 0; i < n- 1; i++)
        {
            Vector3 p0;
            if (i==0)
            {
                p0 = path[i];
            }
            else
            {
                p0 = path[i - 1];
            }
            Vector3 p1 = path[i];
            Vector3 p2 = path[i + 1];
            Vector3 p3;
            if (i==n-2)
            {
                p3 = path[i+1];
            }
            else
            {
                p3 = path[i+2];
            }

            for (float t = 0; t < 1f; t += 0.1f) // Decreasing step will increase number of points
            {
                Vector3 newPoint = catmull_Rom(p0, p1, p2, p3, t);
                smoothedPath.Add(newPoint);
            }
        }
        smoothedPath.Add(path[n - 1]); // Add last point
        smoothedPath = simplifyPath(smoothedPath, 0.2f);

        return smoothedPath;
    }

    private Vector3 catmull_Rom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) // interpolate new point between p0 and p1
    {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
    }

    public List<Vector3> simplifyPath(List<Vector3> path, float epsilon) //use Ramer-Douglas-Peucker
    {
        if (path == null || path.Count < 3)
            return path;

        List<Vector3> simplifiedPath = new List<Vector3>();
        simplifiedPath.Add(path[0]);
        simplifyRecursive(path, 0, path.Count - 1, epsilon, simplifiedPath);
        simplifiedPath.Add(path[path.Count - 1]); // add last point

        return simplifiedPath;
    }

    private void simplifyRecursive(List<Vector3> path, int startIndex, int endIndex, float epsilon, List<Vector3> new_path)
    {
        if (endIndex <= startIndex + 1) // can't simplify two points
            return; 

        float maxDistance = 0;
        int indexFurthest = 0;

        Vector3 startPoint = path[startIndex];
        Vector3 endPoint = path[endIndex];

        // Find the point farthest from the line segment
        for (int i = startIndex + 1; i < endIndex; i++)
        {
            float distance = perpendicularDistance(path[i], startPoint, endPoint);
            if (distance > maxDistance)
            {
                maxDistance = distance;
                indexFurthest = i;
            }
        }

        if (maxDistance > epsilon) // if points too far from line, keep it
        {
            simplifyRecursive(path, startIndex, indexFurthest, epsilon, new_path); // simplify first part of the segment
            new_path.Add(path[indexFurthest]); // add the point
            simplifyRecursive(path, indexFurthest, endIndex, epsilon, new_path); // simplify second part of the segment
        }
    }

    private float perpendicularDistance(Vector3 point, Vector3 lineStart, Vector3 lineEnd) // get distance from point to line
    {
        Vector3 line = lineEnd - lineStart;
        Vector3 projection = Vector3.Project(point - lineStart, line);
        Vector3 closestPoint = lineStart + projection;
        return Vector3.Distance(point, closestPoint);
    }
}