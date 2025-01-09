using System;
using System.Collections.Generic;
using System.Linq;
using Scripts.Game;
using UnityEngine;

public class GameManagerA1 : AbstractGameManager
{
    public float goalTolerance = 10.0f;
    public Dictionary<GameObject, Goal> vehicleToGoalMapping = new();

    public override List<Goal> CreateGoals(List<GameObject> vehicles)
    {
        var list = new List<Goal>();
        list.Add(new WaypointGoal(mapManager.GetGlobalGoalPosition(), goalTolerance));
        vehicleToGoalMapping[vehicles[0]] = list[0];
        return list;
    }

    private void FixedUpdate()
    {
        if (vehicleToGoalMapping.ToList().TrueForAll(pair => pair.Value.CheckAchieved(pair.Key)))
        {
            completionTime = goals.Max(goal => goal.CurrentTime());
        }
    }
}