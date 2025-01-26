using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Scripts.Map;


public class ImprovePath
{
    Vector3 Catmull_Rom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) // interpolate new point between p0 and p1
    {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
    }
}