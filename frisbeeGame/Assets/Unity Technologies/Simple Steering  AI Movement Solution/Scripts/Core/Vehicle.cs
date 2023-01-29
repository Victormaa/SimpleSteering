using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// The Vehicle here is an example of an instance that can move.
/// 
/// -----------------------  slice line  ------------------------------
/// 
/// For you, if you wanna create a new object or say a different one that has
/// its own movement trait. You can either create a class inherited from MovingEntity
/// or just modify the parameter file used in the class.
/// </summary>

namespace SimpleSteering
{
    public class Vehicle : MovingEntity
    {
        private AgentManager m_agentManager;
        private SteeringBehavior m_Steering;

        private bool m_bIsFaceVelocityDir = true;
        private BaseGameEntity FaceTarget;
        //when true, smoothing is active
        private bool m_bSmoothingOn = true;
        private SmootherForVector2 m_Smoother2D;
        private Vector2 m_CurrentForce;

        public Vehicle(int entitytype,
                        AgentManager world,
                        MovementPrm movementprm,
                        Vector2 AgentPos,
                        GameObject agent,
                        int spawnID,
                        SteeringBehavior.Deceleration decel = SteeringBehavior.Deceleration.normal
                        )
            : base(entitytype, AgentPos, movementprm.AgentRadius, Vector2.zero,
                movementprm.MaxSpeed, Vector2.up, movementprm.VehicleMass, 1, movementprm.MaxTurnRatePerSecond, movementprm.MaxSteeringForce, movementprm.BrakeFactorIfUnderPlayerControl)
        {
            m_agentManager = world;
            m_Agent = agent;
            m_Steering = new SteeringBehavior(this, movementprm.steeringPrm, decel);
            m_Smoother2D = new SmootherForVector2(movementprm.NumSamplesForSmoothing, Vector2.zero);
            FaceTarget = new GameObjectEntity();
        }

        public Vehicle() : base() { }

        public void InitializedVehicleSystem(int entityType, GameWorldPrm worldprm, MovementAgent<MovementComponent> agent, AgentManager world, int spawnID)
        {
            Vector2 pos;
            if (spawnID == -1)
                pos = new Vector2(agent.transform.position.x, agent.transform.position.z);
            else
            {
                pos = new Vector2(Random.Range(worldprm.m_MxRange_min[spawnID], worldprm.m_MxRange_max[spawnID])
                    , Random.Range(worldprm.m_MyRange_min[spawnID], worldprm.m_MyRange_max[spawnID]));
            }

            agent.InitializedMovementRoleEntity(entityType, world, pos, spawnID);
        }
        public void InitializedVehicleSystem_GWP(int entityType, GameWorldPrm worldprm, MovementAgent<MovementComponent> agent, AgentManager world, int spawnID)
        {
            Vector2 pos = new Vector2(Random.Range(worldprm.m_MxRange_min[spawnID], worldprm.m_MxRange_max[spawnID])
                    , Random.Range(worldprm.m_MyRange_min[spawnID], worldprm.m_MyRange_max[spawnID]));

            agent.transform.position = new Vector3(pos.x, 0, pos.y);
            agent.transform.rotation = Quaternion.identity;

            agent.InitializedMovementRoleEntity(entityType, world, pos, spawnID);
        }
        public AgentManager AgentManager() { return m_agentManager; }
        public override bool SteeringUpdate(float time_elapsed)
        {
            var tempPos = m_pos;
            Vector2 newVel = m_velocity;
            

            m_CurrentForce = m_Steering.Calculate();

            if (m_CurrentForce.magnitude == 0)
            {
                m_CurrentForce = -m_velocity.normalized * MaxForce() / 3;

                var decel = m_CurrentForce / Mass() * m_breakDownFactor;
                newVel += decel * time_elapsed;
                if (Vector2.Dot(newVel, m_velocity) < 0)
                {
                    newVel = Vector2.zero;
                }
            }
            else
            {
                Vector2 acceleration = m_CurrentForce / m_mass;
                newVel += acceleration * time_elapsed;
                //maxspeed truncate;
                newVel = Vector2.ClampMagnitude(newVel, m_maxSpeed);
            }
            
            tempPos += newVel * time_elapsed;

            SetPos(tempPos);
            SetVelocity(newVel);

            Common.EnforceNonPenetrationConstraint(this, AgentManager().Vehicles());
            Common.EnforceNonPenetrationConstraint(this, AgentManager().Obstacles());
            Common.EnforceNonPenetrationOnWall(this, AgentManager().Walls());
            return false;
        }
        public void InControlSteeringUpdate(float time_elapsed, Vector2 MovingDir)
        {
            var tempPos = m_pos;
            Vector2 newVel = m_velocity;
            if (MovingDir.magnitude == 0)
            {
                m_CurrentForce = -m_velocity.normalized * MaxForce() / 3;

                var decel = m_CurrentForce / Mass() * m_breakDownFactor;
                newVel += decel * time_elapsed;
                if (Vector2.Dot(newVel, m_velocity) < 0)
                {
                    newVel = Vector2.zero;
                }
            }
            else
            {
                m_CurrentForce = m_Steering.Calculate();

                Common.AccumulateForce(ref m_CurrentForce, MovingDir * MaxForce(), MaxForce());

                Vector2 acceleration = m_CurrentForce / m_mass;

                newVel += acceleration * time_elapsed;
                newVel = Vector2.ClampMagnitude(newVel, MaxSpeed());
            }

            tempPos += newVel * time_elapsed;


            SetPos(tempPos);
            SetVelocity(newVel);

            Common.EnforceNonPenetrationConstraint(this, AgentManager().Vehicles());
            Common.EnforceNonPenetrationConstraint(this, AgentManager().Obstacles());
            Common.EnforceNonPenetrationOnWall(this, AgentManager().Walls());
        }
        public override bool FaceDirUpdate(Vector2 target, bool isFaceVelDir)
        {
            Vector2 toTarget = Vector2.zero;

            if (isFaceVelDir)
                toTarget = m_velocity.normalized;
            else
                toTarget = (target - m_pos).normalized;

            m_turnAngle = Vector2.Angle(m_facing, toTarget);

            if (m_turnAngle < 0.1f)
            {
                m_turnAngle = 0;
                return true;
            }

            if (m_turnAngle > m_maxTurnRate) m_turnAngle = m_maxTurnRate;

            m_turnAngle *= AngleDir(new Vector3(m_facing.x, 0, m_facing.y),
                            new Vector3(toTarget.x, 0, toTarget.y), Vector3.up);

            var rotate = Quaternion.AngleAxis(m_turnAngle, Vector3.up);

            var rotateMatrix = Matrix4x4.Rotate(rotate);

            var newfacing = rotateMatrix.MultiplyVector(new Vector3(m_facing.x, 0, m_facing.y));

            if (isSmoothingOn())
            {
                var smoothFacing = m_Smoother2D.SmoothFacingUpdate(new Vector2(newfacing.x, newfacing.z)).normalized;

                m_turnAngle = Vector2.Angle(m_facing, smoothFacing);

                m_turnAngle *= AngleDir(new Vector3(m_facing.x, 0, m_facing.y),
                                new Vector3(smoothFacing.x, 0, smoothFacing.y), Vector3.up);

                newfacing.x = smoothFacing.x;
                newfacing.z = smoothFacing.y;
            }

            SetFacing(newfacing);

            return false;
        }
        public void UpdateSteeringAndRotate()
        {
            SteeringUpdate(Time.fixedDeltaTime);
            FaceDirUpdate(FaceTarget.Pos(), GetIsFaceVelDir());
        }
        public void UpdateSteeringAndRotateInPlayerControl(Vector2 moveDir)
        {
            InControlSteeringUpdate(Time.fixedDeltaTime, moveDir);
            FaceDirUpdate(FaceTarget.Pos(), GetIsFaceVelDir());
        }
        public void UpdateTransform(float groundHight)
        {
            m_Agent.transform.position = new Vector3(Pos().x, groundHight, Pos().y);
            m_Agent.transform.eulerAngles += new Vector3(0, m_turnAngle, 0);
        }
        public void AgentUpdate()
        {
            m_Agent.GetComponent<MovementAgent<MovementComponent>>().AgentUpdate();
        }
        public void AgentStart()
        {
            m_Agent.GetComponent<MovementAgent<MovementComponent>>().AgentStart();
        }
        public SteeringBehavior Steering() { return m_Steering; }
        public bool GetIsFaceVelDir() { return m_bIsFaceVelocityDir; }
        public Vector2 GetCurrentForce() { return m_CurrentForce; }
        public void SetIsFaceVelDir(bool b) { m_bIsFaceVelocityDir = b; }
        public bool isSmoothingOn() { return m_bSmoothingOn; }
        public void SmoothingOn() { m_bSmoothingOn = true; }
        public void SmoothingOff() { m_bSmoothingOn = false; }
        public void ToggleSmoothing() { m_bSmoothingOn = !m_bSmoothingOn; }
        public bool IsPlayerControl() { return m_Agent.GetComponent<MovementAgent<MovementComponent>>().IsPlayerControl; }
        public override void RefreshSteeringPrm(MovementPrm prm)
        {
            base.RefreshSteeringPrm(prm);
            m_Steering.RefreshSteeringPrm(prm.steeringPrm);
        }
    }
}
