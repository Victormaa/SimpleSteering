using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class MovingEntity : BaseGameEntity
{
    protected Vector2 m_velocity;
    protected Vector2 m_facing;
    protected Vector2 m_siding;
    protected float m_mass;
    protected float m_maxSpeed;
    protected float m_maxForce;
    protected float m_maxTurnRate;
    protected float m_breakDownFactor;
    protected float m_turnAngle;

    protected GameObject m_Agent;

    public MovingEntity(int entitytype,
                Vector2 position,
               float radius,
               Vector2 velocity,
               float max_speed,
               Vector2 heading,
               float mass,
               float scale,
               float turn_rate,
               float max_force,
               float break_factor) : base(entitytype, position,radius, scale)
    {
        m_facing = heading;
        m_velocity = velocity;
        m_siding = Vector2.Perpendicular(m_facing);
        m_maxForce = max_force;
        m_maxTurnRate = turn_rate;
        m_mass = mass;
        m_maxSpeed = max_speed;
        m_breakDownFactor = break_factor;
    }
    protected MovingEntity() : base() {}
    public Vector2 Velocity() { return m_velocity; }
    public void SetVelocity(Vector2 newVel) => m_velocity = newVel; 
    public float Mass() { return m_mass; }
    public Vector2 Side() { return m_siding; }
    public float MaxSpeed() { return m_maxSpeed; }
    public void SetMaxSpeed(float new_maxspeed) => m_maxSpeed = new_maxspeed;
    public float MaxForce() { return m_maxForce; }
    public void SetMaxForce(float new_maxforce) => m_maxForce = new_maxforce;
    public bool isSpeedMaxedOut() { return m_maxSpeed * m_maxSpeed < m_velocity.sqrMagnitude; }
    public float Speed() { return m_velocity.magnitude; }
    public float SpeedSqr() { return m_velocity.sqrMagnitude; }
    public Vector2 Facing() { return m_facing; }
    public void SetFacing(Vector2 newfacing) 
    {
        m_facing = newfacing;
        m_siding = Vector2.Perpendicular(m_facing); ;
    }
    public void SetFacing(Vector3 newfacing) 
    {
        m_facing = new Vector2(newfacing.x, newfacing.z);
        m_siding = Vector2.Perpendicular(m_facing); ;
    }
    public virtual bool SteeringUpdate(float time_elapsed, out Vector2 pos, out Quaternion rotate)
    {
        pos = Vector2.zero;
        rotate = Quaternion.identity;
        return false;
    }
    public virtual bool SteeringUpdate(float time_elapsed)
    {
        return false;
    }
    public virtual void RefreshSteeringPrm(MovementPrm prm)
    {
        m_maxForce = prm.MaxSteeringForce;
        m_maxTurnRate = prm.MaxTurnRatePerSecond;
        m_scale = 1;
        //m_scale = prm.VehicleScale;
        m_mass = prm.VehicleMass;
        m_maxSpeed = prm.MaxSpeed;
        m_dBoundingRadius = prm.AgentRadius;
        m_breakDownFactor = prm.BrakeFactorIfUnderPlayerControl;
    }
    public virtual bool FaceDirUpdate(Vector2 target, bool isFaceVelDir) 
    {
        m_turnAngle = 0;
        return false;
    }
    public float AngleDir(Vector3 fwd, Vector3 targetDir, Vector3 up)
    {
        //returns -1 when to the left, 1 to the right, and 0 for forward/backward
        Vector3 perp = Vector3.Cross(fwd, targetDir);
        float dir = Vector3.Dot(perp, up);

        if (dir > 0.0f)
        {
            return 1.0f;
        }
        else if (dir < 0.0f)
        {
            return -1.0f;
        }
        else
        {
            return 0.0f;
        }
    }
    public GameObject Agent() { return m_Agent; }
}
}