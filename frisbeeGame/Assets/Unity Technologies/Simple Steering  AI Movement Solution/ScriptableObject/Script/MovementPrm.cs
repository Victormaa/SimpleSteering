using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering {
[CreateAssetMenu(fileName = "MovementPrm", menuName = "Movement_Parameter", order = 2)]
public class MovementPrm : ScriptableObject
{
    public SteeringPrm steeringPrm;

    [Header("MovementParameter", order = 0)]
    public float MaxSteeringForce;
    public float MaxSpeed;
    public float VehicleMass;

    public float AgentRadius;
    //public float VehicleScale;
    public float MaxTurnRatePerSecond;

    public float BrakeFactorIfUnderPlayerControl;
    [Header("Smooth and Tweaker Factor", order = 2)]
    //how many samples the smoother will use to average a value
    public int NumSamplesForSmoothing;

    //used to tweak the combined steering force (simply altering the MaxSteeringForce
    //will NOT work!This tweaker affects all the steering force multipliers
    //too).
    //public float SteeringForceTweaker;
}
}