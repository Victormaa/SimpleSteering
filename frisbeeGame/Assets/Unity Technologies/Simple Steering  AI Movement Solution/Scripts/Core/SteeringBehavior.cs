using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering
{
    public class SteeringBehavior
    {
        //--------------------------- Constants ----------------------------------
        //the radius of the constraining circle for the wander behavior
        const float WanderRad = 10.0f;
        //distance the wander circle is projected in front of the agent
        const float WanderDist = 4.0f;
        //the maximum amount of displacement along the circle each frame
        const float WanderJitterPerSec = 160.0f;

        //used in path following
        const float WaypointSeekDist = 1.5f;

        const float ArriveDistance = 1.4f;
        const float ArriveStopDistance = 0.85f;
        //------------------------------------------------------------------------
        public enum summing_method { weighted_average, prioritized, dithered };

        [System.Flags]
        public enum behavior_type
        {
            none = 0x00000,
            seek = 0x00002,
            flee = 0x00004,
            arrive = 0x00008,
            wander = 0x00010,
            cohesion = 0x00020,
            separation = 0x00040,
            alignment = 0x00080,
            obstacle_avoidance = 0x00100,
            wall_avoidance = 0x00200,
            follow_path = 0x00400,
            pursuit = 0x00800,
            evade = 0x01000,
            interpose = 0x02000,
            hide = 0x04000,
            flock = 0x08000,
            offset_pursuit = 0x10000,
        };

        [System.Flags]
        public enum behavior_type_for_switch
        {
            none = 0x00000,
            wander = 0x00010,
            obstacle_avoidance = 0x00100,
            wall_avoidance = 0x00200,
            flock = 0x08000,
        };

        private Dictionary<int, bool> m_dicBehaviors = new Dictionary<int, bool>();
        //a pointer to the owner of this instance
        private MovingEntity m_MovementAgent;

        //the steering force created by the combined effect of all
        //the selected behaviors
        private Vector2 m_vSteeringForce;

        //these can be used to keep track of friends, pursuers, or prey
        private MovingEntity m_pTargetAgent1;
        private MovingEntity m_pTargetAgent2;

        //the current target
        private Vector2 m_vTarget;

        //length of the 'detection box' utilized in obstacle avoidance
        private float m_dDBoxLength;
        private readonly float miniDetectionBoxFactor;

        //the current position on the wander circle the agent is
        //attempting to steer towards
        private Vector2 m_vWanderTarget;

        //explained above
        private float m_dWanderJitter;
        private float m_dWanderRadius;
        private float m_dWanderDistance;

        //multipliers. These can be adjusted to effect strength of the  
        //appropriate behavior. Useful to get flocking the way you require
        //for example.
        private float m_dWeightSeparation;
        private float m_dWeightCohesion;
        private float m_dWeightAlignment;
        private float m_dWeightWander;
        private float m_dWeightObstacleAvoidance;
        private float m_dWeightWallAvoidance;
        private float m_dWeightSeek;
        private float m_dWeightFlee;
        private float m_dWeightArrive;
        private float m_dWeightPursuit;
        private float m_dWeightOffsetPursuit;
        private float m_dWeightInterpose;
        private float m_dWeightHide;
        private float m_dWeightEvade;
        private float m_dWeightFollowPath;

        //how far the agent can 'see'
        private float m_fViewDistance;

        //the distance (squared) a vehicle has to be from a path waypoint before
        //it starts seeking to the next waypoint
        private float m_dWaypointSeekDistSq;

        //any offset used for formations or offset pursuit
        private Vector2 m_vOffset;

        //binary flags to indicate whether or not a behavior should be active
        private int m_iFlags;

        private Vector2[] m_Feelers;

        //the length of the 'feeler/s' used in wall detection
        private float m_fWallDetectionFeelerLength;

        //Arrive makes use of these to determine how quickly a vehicle
        //should decelerate to its target
        public enum Deceleration { slow = 3, normal = 2, fast = 1 };

        //default
        private Deceleration m_Deceleration;

        //is cell space partitioning to be used or not?
        private bool m_bCellSpaceOn;

        //what type of method is used to sum any active behavior
        private summing_method m_SummingMethod;

        //this function tests if a specific bit of m_iFlags is set
        private bool On(behavior_type bt) { return (m_iFlags & ((int)bt)) == ((int)bt); }
        private bool AccumulateForce(ref Vector2 steeringforce, Vector2 ForceToAdd)
        {
            // 启发函数 帮助节省计算以及 包含一个优先级的考量
            float forceMagnitudeSofar = steeringforce.magnitude;

            float forceMagnitudeRemaining = m_MovementAgent.MaxForce() - forceMagnitudeSofar;

            if (forceMagnitudeRemaining <= 0) return false;

            float forceMagnitutdeToAdd = ForceToAdd.magnitude;

            if (forceMagnitutdeToAdd < forceMagnitudeRemaining) steeringforce += ForceToAdd;
            else steeringforce += ForceToAdd.normalized * forceMagnitudeRemaining;

            return true;
        }

        /* .......................................................
                       BEGIN BEHAVIOR DECLARATIONS
         .......................................................*/

        //this behavior moves the agent towards a target position
        private Vector2 Seek(Vector2 TargetPos)
        {
            Vector2 DesiredVelocity = (TargetPos - m_MovementAgent.Pos()).normalized * m_MovementAgent.MaxSpeed();

            return (DesiredVelocity - m_MovementAgent.Velocity());
        }

        //this behavior returns a vector that moves the agent away
        //from a target position
        private Vector2 Flee(Vector2 TargetPos)
        {
            //only flee if the target is within 'panic distance'. Work in distance
            //squared space.
            const float PanicDistance = 10.0f;
            if (Vector2.Distance(m_MovementAgent.Pos(), TargetPos) > PanicDistance)
            {
                if (m_MovementAgent.Speed() > 0.4f)
                {
                    m_MovementAgent.SetVelocity(m_MovementAgent.Velocity() * 0.85f);
                    return -m_MovementAgent.Velocity() * 0.85f;
                }
                else
                {
                    m_MovementAgent.SetVelocity(Vector2.zero);
                    return Vector2.zero;
                }
            }
            else
            {
                Vector2 DesiredVelocity = -(TargetPos - m_MovementAgent.Pos()).normalized * m_MovementAgent.MaxSpeed();

                return (DesiredVelocity - m_MovementAgent.Velocity());
            }
        }

        //this behavior is similar to seek but it attempts to arrive 
        //at the target position with a zero velocity
        private Vector2 Arrive(Vector2 TargetPos, Deceleration deceleration)
        {
            Vector2 ToTarget = TargetPos - m_MovementAgent.Pos();

            //calculate the distance to the target
            float dist = Vector2.Distance(TargetPos, m_MovementAgent.Pos());

            if (dist < ArriveStopDistance)
            {
                m_MovementAgent.SetVelocity(Vector2.zero);
                return Vector2.zero;
            }
            else if (dist > ArriveDistance)
            {
                //because Deceleration is enumerated as an int, this value is required
                //to provide fine tweaking of the deceleration..
                const float DecelerationTweaker = 0.5f;

                //calculate the speed required to reach the target given the desired
                //deceleration
                float speed = dist / ((float)deceleration * DecelerationTweaker);

                //make sure the velocity does not exceed the max
                speed = Mathf.Min(speed, m_MovementAgent.MaxSpeed());

                //from here proceed just like Seek except we don't need to normalize 
                //the ToTarget vector because we have already gone to the trouble
                //of calculating its length: dist. 
                Vector2 DesiredVelocity = ToTarget * speed / dist;

                return (DesiredVelocity - m_MovementAgent.Velocity());
            }
            else
            {
                return -m_MovementAgent.Velocity();
            }
        }

        //this behavior predicts where an agent will be in time T and seeks
        //towards that point to intercept it.
        private Vector2 Pursuit(MovingEntity evader)
        {
            //if the evader is ahead and facing the agent then we can just seek
            //for the evader's current position.
            Vector2 ToEvader = evader.Pos() - m_MovementAgent.Pos();

            float RelativeHeading = Vector2.Dot(m_MovementAgent.Facing(), evader.Facing());

            if ((Vector2.Dot(ToEvader, m_MovementAgent.Facing()) > 0)
                && (RelativeHeading < -0.95))  //acos(0.95)=18 degs
            {
                return Seek(evader.Pos());
            }

            //Not considered ahead so we predict where the evader will be.

            //the lookahead time is propotional to the distance between the evader
            //and the pursuer; and is inversely proportional to the sum of the
            //agent's velocities
            float LookAheadTime = ToEvader.magnitude /
                                  (m_MovementAgent.MaxSpeed() + evader.Speed());

            //now seek to the predicted future position of the evader
            return Seek(evader.Pos() + evader.Velocity() * LookAheadTime);
        }

        //this behavior maintains a position, in the direction of offset
        //from the target vehicle
        //------------------------- Offset Pursuit -------------------------------
        //
        //  Produces a steering force that keeps a vehicle at a specified offset
        //  from a leader vehicle
        //------------------------------------------------------------------------
        private Vector2 OffsetPursuit(MovingEntity leader, Vector2 offset)
        {
            //calculate the offset's position in world space
            Vector3 offset3D = new Vector3(offset.x, 0, offset.y);

            Vector3 worldOffsetPos3D = leader.Agent().transform.TransformPoint(offset3D);

            Vector2 worldOffsetPos2D = new Vector2(worldOffsetPos3D.x, worldOffsetPos3D.z);

            Vector2 toOffset = worldOffsetPos2D - m_MovementAgent.Pos();

            float LookAheadTime = toOffset.magnitude / (m_MovementAgent.MaxSpeed() + leader.Speed());

            return Arrive((worldOffsetPos2D + leader.Velocity() * LookAheadTime), Deceleration.slow);
        }

        //this behavior attempts to evade a pursuer
        private Vector2 Evade(MovingEntity pursuer)
        {
            /* Not necessary to include the check for facing direction this time */
            Vector2 ToPursuer = pursuer.Pos() - m_MovementAgent.Pos();

            //uncomment the following two lines to have Evade only consider pursuers 
            //within a 'threat range'
            const double ThreatRange = 10.0;
            if (ToPursuer.sqrMagnitude > ThreatRange * ThreatRange)
            {
                if (m_MovementAgent.Speed() > 0.4f)
                {
                    m_MovementAgent.SetVelocity(m_MovementAgent.Velocity() * 0.85f);
                    return -m_MovementAgent.Velocity() * 0.85f;
                }
                else
                {
                    m_MovementAgent.SetVelocity(Vector2.zero);
                    return Vector2.zero;
                }
            }
            else
            {
                //the lookahead time is propotional to the distance between the pursuer
                //and the pursuer; and is inversely proportional to the sum of the
                //agents' velocities
                float LookAheadTime = ToPursuer.magnitude /
                                       (m_MovementAgent.MaxSpeed() + pursuer.Speed());

                //now flee away from predicted future position of the pursuer
                return Flee(pursuer.Pos() + pursuer.Velocity() * LookAheadTime);
            }
        }

        //this behavior makes the agent wander about randomly
        private Vector2 Wander()
        {
            if (isArrivedWanderNearbyPoint && waitedTime > WanderCoolDownTime || curWanderTarget == Vector2.zero)
            {
                //this behavior is dependent on the update rate, so this line must
                //be included when using time independent framerate.
                //float JitterThisTimeSlice = m_dWanderJitter * m_pVehicle.TimeElapsed();
                float JitterThisTimeSlice = m_dWanderJitter * Time.fixedDeltaTime;

                //first, add a small random vector to the target's position
                m_vWanderTarget += new Vector2((Random.value - 0.5f) * 2.0f * JitterThisTimeSlice,
                                            (Random.value - 0.5f) * 2.0f * JitterThisTimeSlice);

                //reproject this new vector back on to a unit circle
                m_vWanderTarget.Normalize();

                //increase the length of the vector to the same as the radius
                //of the wander circle
                m_vWanderTarget *= m_dWanderRadius;

                //move the target into a position WanderDist in front of the agent
                curWanderTarget = m_vWanderTarget;// + new Vector2(m_dWanderDistance, 0);

                //project the target into world space
                //Vector2 Target = m_pVehicle.Pos() + target;
                waitedTime = 0;
                isArrivedWanderNearbyPoint = false;
                //and steer towards it
                return curWanderTarget;
            }
            else
            {
                if (Vector2.Distance(m_MovementAgent.Pos(), curWanderTarget) < 1.35f || m_MovementAgent.Speed() < 0.1f)
                {
                    isArrivedWanderNearbyPoint = true;
                    waitedTime++;
                }
                return Arrive(curWanderTarget, m_Deceleration);
            }
        }

        private bool isArrivedWanderNearbyPoint = false;
        private int waitedTime = 0; //
        private const int WanderCoolDownTime = 30; // 1 second
        Vector2 curWanderTarget = Vector2.zero;
        public Vector2 GetCurWanderTarget() { return curWanderTarget; }
        private Vector2 ReturnRandomWanderNearbyTarget()
        {
            Vector2 target = Vector2.zero;
            bool isPosInsideObstcle = true;

            while (isPosInsideObstcle)
            {
                target = new Vector2((Random.value - 0.5f) * 2.0f * m_dWanderRadius,
                                            (Random.value - 0.5f) * 2.0f * m_dWanderRadius);

                isPosInsideObstcle = Common.IsPosInsideObstacles(target);
            }

            return target;
        }
        private Vector2 WanderNearby()
        {
            if (isArrivedWanderNearbyPoint && waitedTime > WanderCoolDownTime || curWanderTarget == Vector2.zero)
            {
                //first, add a small random vector to the target's position
                m_vWanderTarget = ReturnRandomWanderNearbyTarget();

                //move the target into a position WanderDist in front of the agent
                curWanderTarget = m_vWanderTarget;// + new Vector2(m_dWanderDistance, 0);

                //and steer towards it
                waitedTime = 0;
                isArrivedWanderNearbyPoint = false;
                return Arrive(curWanderTarget, m_Deceleration);
            }
            else
            {
                if (Vector2.Distance(m_MovementAgent.Pos(), curWanderTarget) < 1.35f || m_MovementAgent.Speed() < 0.1f)
                {
                    isArrivedWanderNearbyPoint = true;
                    waitedTime++;
                }
                return Arrive(curWanderTarget, m_Deceleration);
            }
        }

        //this returns a steering force which will attempt to keep the agent 
        //away from any obstacles it may encounter
        private Vector2 ObstacleAvoidance()
        {
            //the detection box length is proportional to the agent's velocity
            m_dDBoxLength = miniDetectionBoxFactor +
                            (m_MovementAgent.Speed() / m_MovementAgent.MaxSpeed()) *
                            miniDetectionBoxFactor;

            Vector3 force = AvoidObstacles(AgentManager.Instance.Obstacles());

            return new Vector2(force.x, force.z);
        }
        private Vector2 AgentAvoidance()
        {
            m_dDBoxLength = miniDetectionBoxFactor +
                            (m_MovementAgent.Speed() / m_MovementAgent.MaxSpeed()) *
                            miniDetectionBoxFactor;

            Vector3 force = AvoidAgents(AgentManager.Instance.Vehicles());

            return new Vector2(force.x, force.z);
        }
        private Vector3 AvoidAgents(List<Vehicle> agents)
        {
            //tag all obstacles within range of the box for processing
            //m_MovementAgent.AgentManager().TagObstaclesWithinViewRange(m_MovementAgent, m_dDBoxLength);
            Common.TagVehiclesWithinViewRange(m_MovementAgent, m_dDBoxLength);

            //this will keep track of the closest intersecting obstacle (CIB)
            BaseGameEntity ClosestIntersectingObstacle = null;

            //this will be used to track the distance to the CIB
            float DistToClosestIP = float.MaxValue;

            //this will record the transformed local coordinates of the CIB
            Vector2 LocalPosOfClosestObstacle = Vector2.zero;

            Vehicle curOb;
            int count = agents.Count;
            for (int i = 0; i < count; ++i)
            {
                curOb = agents[i];

                /// to debug how many stuff is tag

                //if the obstacle has been tagged within range proceed
                if (curOb.IsTag())
                {

                    //calculate this obstacle's position in local space
                    //        Vector2D LocalPos = PointToLocalSpace((*curOb)->Pos(),
                    //                                               m_pVehicle->Heading(),
                    //                                               m_pVehicle->Side(),
                    //                                               m_pVehicle->Pos());
                    Vector3 LocalPos = m_MovementAgent.Agent().transform.InverseTransformPoint(new Vector3(curOb.Pos().x, AgentManager.Instance.GetGroundHeight(), curOb.Pos().y));

                    //if the local position has a negative x value then it must lay
                    //behind the agent. (in which case it can be ignored)
                    if (LocalPos.z >= 0)
                    {
                        //if the distance from the x axis to the object's position is less
                        //than its radius + half the width of the detection box then there
                        //is a potential intersection.
                        float ExpandedRadius = (curOb).BRadius() + m_MovementAgent.BRadius();

                        if (Mathf.Abs(LocalPos.x) < ExpandedRadius)
                        {
                            //now to do a line/circle intersection test. The center of the 
                            //circle is represented by (cX, cY). The intersection points are 
                            //given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
                            //We only need to look at the smallest positive value of x because
                            //that will be the closest point of intersection.
                            float cX = LocalPos.x;
                            float cY = LocalPos.z;

                            //we only need to calculate the sqrt part of the above equation once
                            float SqrtPart = Mathf.Sqrt(ExpandedRadius * ExpandedRadius - cX * cX);

                            float ip = cY - SqrtPart;

                            if (ip <= 0.0)
                            {
                                ip = cY + SqrtPart;
                            }

                            //test to see if this is the closest so far. If it is keep a
                            //record of the obstacle and its local coordinates
                            if (ip < DistToClosestIP)
                            {
                                DistToClosestIP = ip;

                                ClosestIntersectingObstacle = curOb;

                                LocalPosOfClosestObstacle = new Vector2(LocalPos.x, LocalPos.z);
                            }
                        }
                    }
                }
            }

            //if we have found an intersecting obstacle, calculate a steering 
            //force away from it
            Vector2 SteeringForce = Vector2.zero;

            if (ClosestIntersectingObstacle != null)
            {
                ClosestIntersectingObstacle.ClosestHighLightDebug(true);
                const float BrakingWeight = 0.25f;

                float factor = 0.8f;

                //the closer the agent is to an object, the stronger the 
                //steering force should be
                float multiplier = (m_MovementAgent.MaxForce() * factor - Mathf.Abs(LocalPosOfClosestObstacle.y)) * BrakingWeight;

                //calculate the lateral force
                SteeringForce.x = -(LocalPosOfClosestObstacle.x) * multiplier;

                //apply a braking force proportional to the obstacles distance from
                //the vehicle. 
                SteeringForce.y = -(m_MovementAgent.MaxForce() * factor -
                                   LocalPosOfClosestObstacle.y) *
                                   BrakingWeight;
            }
            //finally, convert the steering vector from local to world space
            var force = m_MovementAgent.Agent().transform.TransformVector(new Vector3(SteeringForce.x, AgentManager.Instance.GetGroundHeight(), SteeringForce.y));
            return force;
        }
        private Vector3 AvoidObstacles(List<Obstacle> obstacles)
        {
            //tag all obstacles within range of the box for processing
            //m_MovementAgent.AgentManager().TagObstaclesWithinViewRange(m_MovementAgent, m_dDBoxLength);
            Common.TagObstaclesWithinViewRange(m_MovementAgent, m_dDBoxLength);

            //this will keep track of the closest intersecting obstacle (CIB)
            BaseGameEntity ClosestIntersectingObstacle = null;

            //this will be used to track the distance to the CIB
            float DistToClosestIP = float.MaxValue;

            //this will record the transformed local coordinates of the CIB
            Vector2 LocalPosOfClosestObstacle = Vector2.zero;

            BaseGameEntity curOb;
            int count = obstacles.Count;
            for (int i = 0; i < count; ++i)
            {
                curOb = obstacles[i];

                /// to debug how many stuff is tag
                curOb.HighLightDebug(false);
                curOb.ClosestHighLightDebug(false);

                //if the obstacle has been tagged within range proceed
                if (curOb.IsTag())
                {
                    curOb.HighLightDebug(true);

                    //calculate this obstacle's position in local space
                    //        Vector2D LocalPos = PointToLocalSpace((*curOb)->Pos(),
                    //                                               m_pVehicle->Heading(),
                    //                                               m_pVehicle->Side(),
                    //                                               m_pVehicle->Pos());
                    Vector3 LocalPos = m_MovementAgent.Agent().transform.InverseTransformPoint(new Vector3(curOb.Pos().x, AgentManager.Instance.GetGroundHeight(), curOb.Pos().y));

                    //if the local position has a negative x value then it must lay
                    //behind the agent. (in which case it can be ignored)
                    if (LocalPos.z >= 0)
                    {
                        //if the distance from the x axis to the object's position is less
                        //than its radius + half the width of the detection box then there
                        //is a potential intersection.
                        float ExpandedRadius = (curOb).BRadius() + m_MovementAgent.BRadius();

                        if (Mathf.Abs(LocalPos.x) < ExpandedRadius)
                        {
                            //now to do a line/circle intersection test. The center of the 
                            //circle is represented by (cX, cY). The intersection points are 
                            //given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
                            //We only need to look at the smallest positive value of x because
                            //that will be the closest point of intersection.
                            float cX = LocalPos.x;
                            float cY = LocalPos.z;

                            //we only need to calculate the sqrt part of the above equation once
                            float SqrtPart = Mathf.Sqrt(ExpandedRadius * ExpandedRadius - cX * cX);

                            float ip = cY - SqrtPart;

                            if (ip <= 0.0)
                            {
                                ip = cY + SqrtPart;
                            }

                            //test to see if this is the closest so far. If it is keep a
                            //record of the obstacle and its local coordinates
                            if (ip < DistToClosestIP)
                            {
                                DistToClosestIP = ip;

                                ClosestIntersectingObstacle = curOb;

                                LocalPosOfClosestObstacle = new Vector2(LocalPos.x, LocalPos.z);
                            }
                        }
                    }
                }
            }

            //if we have found an intersecting obstacle, calculate a steering 
            //force away from it
            Vector2 SteeringForce = Vector2.zero;

            if (ClosestIntersectingObstacle != null)
            {
                ClosestIntersectingObstacle.ClosestHighLightDebug(true);
                const float BrakingWeight = 0.25f;

                float factor = 0.8f;

                //the closer the agent is to an object, the stronger the 
                //steering force should be
                float multiplier = (m_MovementAgent.MaxForce() * factor - Mathf.Abs(LocalPosOfClosestObstacle.y)) * BrakingWeight;

                //calculate the lateral force
                SteeringForce.x = -(LocalPosOfClosestObstacle.x) * multiplier;

                //apply a braking force proportional to the obstacles distance from
                //the vehicle. 
                SteeringForce.y = -(m_MovementAgent.MaxForce() * factor -
                                   LocalPosOfClosestObstacle.y) *
                                   BrakingWeight;
            }
            //finally, convert the steering vector from local to world space
            var force = m_MovementAgent.Agent().transform.TransformVector(new Vector3(SteeringForce.x, AgentManager.Instance.GetGroundHeight(), SteeringForce.y));
            return force;
        }

        //this returns a steering force which will keep the agent away from any
        //walls it may encounter
        private Vector2 WallAvoidance(List<Wall2D> walls)
        {
            CreateFeelers();

            float DistToThisIP = 0.0f;
            float DistToClosestIP = float.MaxValue;

            //this will hold an index into the vector of walls
            int ClosestWall = -1;

            Vector2 SteeringForce = Vector2.zero,
                    point = Vector2.zero,         //used for storing temporary info
                    ClosestPoint = Vector2.zero;  //holds the closest intersection point

            //examine each feeler in turn
            for (int flr = 0; flr < m_Feelers.Length; ++flr)
            {
                //run through each wall checking for any intersection points
                for (int w = 0; w < AgentManager.Instance.Walls().Count; ++w)
                {
                    if (!walls[w].GetUseWallForce())
                        return Vector2.zero;

                    if (Common.LineIntersection2D(m_MovementAgent.Pos(),
                                           m_Feelers[flr],
                                           walls[w].From(),
                                           walls[w].To(),
                                           DistToThisIP,
                                           ref point))
                    {
                        //is this the closest found so far? If so keep a record
                        if (DistToThisIP < DistToClosestIP)
                        {
                            DistToClosestIP = DistToThisIP;

                            ClosestWall = w;

                            ClosestPoint = point;
                        }
                    }
                }//next wall

                //if an intersection point has been detected, calculate a force  
                //that will direct the agent away
                if (ClosestWall >= 0)
                {
                    //calculate by what distance the projected position of the agent
                    //will overshoot the wall
                    Vector2 OverShoot = m_Feelers[flr] - ClosestPoint;

                    //create a force in the direction of the wall normal, with a 
                    //magnitude of the overshoot
                    SteeringForce = walls[ClosestWall].Normal() * OverShoot.magnitude;
                }
            }//next feeler

            return SteeringForce;
        }


        //given a series of Vector2Ds, this method produces a force that will
        //move the agent along the waypoints in order

        //------------------------------- FollowPath -----------------------------
        //
        //  Given a series of Vector2Ds, this method produces a force that will
        //  move the agent along the waypoints in order. The agent uses the
        // 'Seek' behavior to move to the next waypoint - unless it is the last
        //  waypoint, in which case it 'Arrives'
        //------------------------------------------------------------------------

        private Vector2 FollowPath()
        {
            var path = AgentManager.Instance.GetPath();
            var dis = Vector2.Distance(path.CurrentWayPoint(), m_MovementAgent.Pos());
            if (dis * dis < m_dWaypointSeekDistSq)
            {
                path.SetNextWayPoint();
            }

            if (!path.Finished())
            {
                return Seek(path.CurrentWayPoint());
            }
            else
            {
                return Arrive(path.CurrentWayPoint(), Deceleration.normal);
            }
        }

        //this results in a steering force that attempts to steer the vehicle
        //to the center of the vector connecting two moving agents.
        private Vector2 Interpose(MovingEntity AgentA, MovingEntity AgentB)
        {
            //first we need to figure out where the two agents are going to be at 
            //time T in the future. This is approximated by determining the time
            //taken to reach the mid way point at the current time at at max speed.
            Vector2 MidPoint = (AgentA.Pos() + AgentB.Pos()) / 2.0f;

            float TimeToReachMidPoint = Vector2.Distance(m_MovementAgent.Pos(), MidPoint) / m_MovementAgent.MaxSpeed();

            //now we have T, we assume that agent A and agent B will continue on a
            //straight trajectory and extrapolate to get their future positions
            Vector2 APos = AgentA.Pos() + AgentA.Velocity() * TimeToReachMidPoint;
            Vector2 BPos = AgentB.Pos() + AgentB.Velocity() * TimeToReachMidPoint;

            //calculate the mid point of these predicted positions
            MidPoint = (APos + BPos) / 2.0f;

            //then steer to Arrive at it
            return Arrive(MidPoint, Deceleration.fast);
        }

        //given another agent position to hide from and a list of BaseGameEntitys this
        //method attempts to put an obstacle between itself and its opponent
        private Vector2 Hide(MovingEntity hunter, List<Obstacle> obstacles)
        {
            float DistToClosest = float.MaxValue;
            Vector2 BestHidingSpot = m_MovementAgent.Pos();

            Obstacle curOb;
            Obstacle closest = null;
            if (obstacles.Count == 0)
            {
                Debug.LogError("there is no obstacles, cannot run Hide Steering");
            }
            int count = obstacles.Count;
            for (int i = 0; i < count; ++i)
            {
                curOb = obstacles[i];

                //calculate the position of the hiding spot for this obstacle
                Vector2 HidingSpot = GetHidingPosition(curOb.Pos(),
                                                         curOb.BRadius(),
                                                          hunter.Pos());
                //work in distance-squared space to find the closest hiding
                //spot to the agent
                float dist = Mathf.Pow(Vector2.Distance(HidingSpot, m_MovementAgent.Pos()), 2);

                if (dist < DistToClosest)
                {
                    DistToClosest = dist;

                    BestHidingSpot = HidingSpot;

                    closest = curOb;
                }
            }

            //if no suitable obstacles found then Evade the hunter
            if (DistToClosest == float.MaxValue)
            {
                return Evade(hunter);
            }

            //else use Arrive on the hiding spot
            return Arrive(BestHidingSpot, Deceleration.fast);
        }

        // -- Group Behaviors -- //
        //-------------------------------- Cohesion ------------------------------
        //
        //  returns a steering force that attempts to move the agent towards the
        //  center of mass of the agents in its immediate area
        //------------------------------------------------------------------------
        private Vector2 Cohesion(List<Vehicle> neighbors)
        {
            //first find the center of mass of all the agents
            Vector2 CenterOfMass = Vector2.zero, SteeringForce = Vector2.zero;

            int NeighborCount = 0;

            //iterate through the neighbors and sum up all the position vectors
            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure *this* agent isn't included in the calculations and that
                //the agent being examined is close enough ***also make sure it doesn't
                //include the evade target ***
                if ((neighbors[a] != m_MovementAgent) && neighbors[a].IsTag() &&
                  (neighbors[a] != m_pTargetAgent1))
                {
                    CenterOfMass += neighbors[a].Pos();

                    ++NeighborCount;
                }
            }

            if (NeighborCount > 0)
            {
                //the center of mass is the average of the sum of positions
                CenterOfMass /= (float)NeighborCount;

                //now seek towards that position
                SteeringForce = Seek(CenterOfMass);
            }

            //the magnitude of cohesion is usually much larger than separation or
            //allignment so it usually helps to normalize it.
            return SteeringForce.normalized;
        }

        private Vector2 Separation(List<Vehicle> neighbors)
        {
            Vector2 SteeringForce = Vector2.zero;

            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure this agent isn't included in the calculations and that
                //the agent being examined is close enough. ***also make sure it doesn't
                //include the evade target ***
                if ((neighbors[a] != m_MovementAgent) && neighbors[a].IsTag() &&
                  (neighbors[a] != m_pTargetAgent1))
                {
                    Vector2 ToAgent = m_MovementAgent.Pos() - neighbors[a].Pos();

                    //scale the force inversely proportional to the agents distance  
                    //from its neighbor.
                    SteeringForce += ToAgent.normalized / ToAgent.magnitude;
                }
            }

            return SteeringForce;
        }

        //---------------------------- Alignment ---------------------------------
        //
        //  returns a force that attempts to align this agents heading with that
        //  of its neighbors
        //------------------------------------------------------------------------
        private Vector2 Alignment(List<Vehicle> neighbors)
        {
            //used to record the average heading of the neighbors
            Vector2 AverageHeading = Vector2.zero;

            //used to count the number of vehicles in the neighborhood
            int NeighborCount = 0;

            //iterate through all the tagged vehicles and sum their heading vectors  
            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure *this* agent isn't included in the calculations and that
                //the agent being examined  is close enough ***also make sure it doesn't
                //include any evade target ***
                if ((neighbors[a] != m_MovementAgent) && neighbors[a].IsTag() &&
                  (neighbors[a] != m_pTargetAgent1))
                {
                    AverageHeading += neighbors[a].Facing();

                    ++NeighborCount;
                }
            }

            //if the neighborhood contained one or more vehicles, average their
            //heading vectors.
            if (NeighborCount > 0)
            {
                AverageHeading /= (float)NeighborCount;

                AverageHeading.Normalize();

                AverageHeading -= m_MovementAgent.Facing();
            }
            return AverageHeading;
        }

        //the following three are the same as above but they use cell-space
        //partitioning to find the neighbors
        private Vector2 CohesionPlus(List<Vehicle> neighbors)
        {
            //first find the center of mass of all the agents
            Vector2 CenterOfMass = Vector2.zero, SteeringForce = Vector2.zero;

            int NeighborCount = 0;

            //iterate through the neighbors and sum up all the position vectors
            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure *this* agent isn't included in the calculations and that
                //the agent being examined is close enough ***also make sure it doesn't
                //include the evade target ***
                if ((neighbors[a] != m_MovementAgent) && neighbors[a].IsTag() &&
                  (neighbors[a] != m_pTargetAgent1))
                {
                    CenterOfMass += neighbors[a].Pos();

                    ++NeighborCount;
                }
            }

            if (NeighborCount > 0)
            {
                //the center of mass is the average of the sum of positions
                CenterOfMass /= (float)NeighborCount;

                //now seek towards that position
                SteeringForce = Seek(CenterOfMass);
            }

            //the magnitude of cohesion is usually much larger than separation or
            //allignment so it usually helps to normalize it.
            return SteeringForce.normalized;
        }

        private Vector2 SeparationPlus(List<Vehicle> agents) { return new Vector2(); }
        private Vector2 AlignmentPlus(List<Vehicle> agents) { return new Vector2(); }

        /* .......................................................
                          END BEHAVIOR DECLARATIONS
         .......................................................*/

        //calculates and sums the steering forces from any active behaviors

        //---------------------- CalculateWeightedSum ----------------------------
        //
        //  this simply sums up all the active behaviors X their weights and 
        //  truncates the result to the max available steering force before 
        //  returning
        //------------------------------------------------------------------------
        private Vector2 CalculateWeightedSum()
        {
            if (On(behavior_type.wall_avoidance))
            {
                m_vSteeringForce += WallAvoidance(AgentManager.Instance.Walls()) *
                             m_dWeightWallAvoidance;
            }

            if (On(behavior_type.obstacle_avoidance))
            {
                m_vSteeringForce += ObstacleAvoidance() *
                        m_dWeightObstacleAvoidance;
            }

            if (On(behavior_type.evade))
            {
                if (m_pTargetAgent1 == null)
                {
                    Debug.LogError("Evade target not assigned");
                }
                else
                {
                    m_vSteeringForce += Evade(m_pTargetAgent1) * m_dWeightEvade;
                }
            }

            //these next three can be combined for flocking behavior (wander is
            //also a good behavior to add into this mix)
            if (!isSpacePartitioningOn())
            {
                if (On(behavior_type.separation))
                {
                    m_vSteeringForce += Separation(AgentManager.Instance.Vehicles()) * m_dWeightSeparation;
                }

                if (On(behavior_type.alignment))
                {
                    m_vSteeringForce += Alignment(AgentManager.Instance.Vehicles()) * m_dWeightAlignment;
                }

                if (On(behavior_type.cohesion))
                {
                    m_vSteeringForce += Cohesion(AgentManager.Instance.Vehicles()) * m_dWeightCohesion;
                }
            }
            else
            {
                if (On(behavior_type.separation))
                {
                    m_vSteeringForce += SeparationPlus(AgentManager.Instance.Vehicles()) * m_dWeightSeparation;
                }

                if (On(behavior_type.alignment))
                {
                    m_vSteeringForce += AlignmentPlus(AgentManager.Instance.Vehicles()) * m_dWeightAlignment;
                }

                if (On(behavior_type.cohesion))
                {
                    m_vSteeringForce += CohesionPlus(AgentManager.Instance.Vehicles()) * m_dWeightCohesion;
                }
            }

            if (On(behavior_type.wander))
            {
                m_vSteeringForce += Wander() * m_dWeightWander;
            }

            if (On(behavior_type.seek))
            {
                m_vSteeringForce += Seek(m_pTargetAgent1.Pos()) * m_dWeightSeek;
            }

            if (On(behavior_type.flee))
            {
                m_vSteeringForce += Flee(m_pTargetAgent1.Pos()) * m_dWeightFlee;
            }

            if (On(behavior_type.arrive))
            {
                m_vSteeringForce += Arrive(m_pTargetAgent1.Pos(), m_Deceleration) * m_dWeightArrive;
            }

            if (On(behavior_type.pursuit))
            {
                if (m_pTargetAgent1 == null)
                {
                    Debug.LogError("pursuit target not assigned");
                }
                else
                {
                    m_vSteeringForce += Pursuit(m_pTargetAgent1) * m_dWeightPursuit;
                }
            }

            if (On(behavior_type.offset_pursuit))
            {
                if (m_pTargetAgent1 == null)
                {
                    Debug.LogError("pursuit target not assigned");
                }
                else if (m_vOffset == Vector2.zero)
                {
                    Debug.LogError("No offset assigned");
                }
                else
                {
                    m_vSteeringForce += OffsetPursuit(m_pTargetAgent1, m_vOffset) * m_dWeightOffsetPursuit;
                }


            }

            if (On(behavior_type.interpose))
            {
                if (m_pTargetAgent1 == null && m_pTargetAgent2 == null)
                {
                    Debug.LogError("Interpose agents not assigned");
                }
                else
                {
                    m_vSteeringForce += Interpose(m_pTargetAgent1, m_pTargetAgent2) * m_dWeightInterpose;
                }
            }

            if (On(behavior_type.hide))
            {
                if (m_pTargetAgent1 == null)
                {
                    Debug.LogError("Hide target not assigned");
                }
                else
                {
                    m_vSteeringForce += Hide(m_pTargetAgent1, AgentManager.Instance.Obstacles()) * m_dWeightHide;
                }
            }

            if (On(behavior_type.follow_path))
            {
                m_vSteeringForce += FollowPath() * m_dWeightFollowPath;
            }

            //m_vSteeringForce.Truncate(m_pVehicle.MaxForce());
            m_vSteeringForce = Vector2.ClampMagnitude(m_vSteeringForce, m_MovementAgent.MaxForce());

            return m_vSteeringForce;
        }
        private Vector2 CalculatePrioritized()
        {
            Vector2 force;
            if (On(behavior_type.wall_avoidance))
            {
                force = WallAvoidance(AgentManager.Instance.Walls()) * m_dWeightWallAvoidance;
                if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
            }

            if (On(behavior_type.obstacle_avoidance))
            {
                force = ObstacleAvoidance() * m_dWeightObstacleAvoidance;

                if (!AccumulateForce(ref m_vSteeringForce, force))
                {
                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.evade))
            {
                if (m_pTargetAgent1 != null)
                {
                    force = Evade(m_pTargetAgent1);

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    Debug.LogError("Evade target not assigned");
                }
            }

            if (On(behavior_type.flee))
            {
                if (m_pTargetAgent1 != null)
                {
                    force = Flee(m_pTargetAgent1.Pos()) * m_dWeightFlee;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    Debug.LogError("flee target not assigned");
                }
            }

            //these next three can be combined for flocking behavior (wander is
            //also a good behavior to add into this mix)
            if (!isSpacePartitioningOn())
            {
                if (On(behavior_type.separation))
                {
                    force = Separation(AgentManager.Instance.Vehicles()) * m_dWeightSeparation;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }

                if (On(behavior_type.alignment))
                {
                    force = Alignment(AgentManager.Instance.Vehicles()) * m_dWeightAlignment;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }

                if (On(behavior_type.cohesion))
                {
                    force = Cohesion(AgentManager.Instance.Vehicles()) * m_dWeightCohesion;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
            }
            else
            {

                if (On(behavior_type.separation))
                {
                    force = SeparationPlus(AgentManager.Instance.Vehicles()) * m_dWeightSeparation;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }

                if (On(behavior_type.alignment))
                {
                    force = AlignmentPlus(AgentManager.Instance.Vehicles()) * m_dWeightAlignment;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }

                if (On(behavior_type.cohesion))
                {
                    force = CohesionPlus(AgentManager.Instance.Vehicles()) * m_dWeightCohesion;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
            }

            if (On(behavior_type.seek))
            {
                if (m_pTargetAgent1 != null)
                {
                    force = Seek(m_pTargetAgent1.Pos()) * m_dWeightSeek;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    /// if there is no target to arrive what should we do??
                    Debug.LogError("seek target not assigned");
                }
            }


            if (On(behavior_type.arrive))
            {
                if (m_pTargetAgent1 != null)
                {
                    force = Arrive(m_pTargetAgent1.Pos(), m_Deceleration) * m_dWeightArrive;

                    if (!AccumulateForce(ref m_vSteeringForce, force))
                    {
                        return m_vSteeringForce;
                    }
                }
                else
                {
                    /// if there is no target to arrive what should we do??
                    Debug.LogError("arrive target not assigned");
                }
            }

            if (On(behavior_type.wander))
            {
                //force = Wander() * m_dWeightWander;
                force = WanderNearby() * m_dWeightWander;

                if (!AccumulateForce(ref m_vSteeringForce, force))
                {
                    return m_vSteeringForce;
                }
            }

            if (On(behavior_type.pursuit))
            {
                if (m_pTargetAgent1 != null)
                {
                    force = Pursuit(m_pTargetAgent1) * m_dWeightPursuit;
                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    Debug.LogError("pursuit target not assigned");
                }
            }

            if (On(behavior_type.offset_pursuit))
            {
                if (m_pTargetAgent1 != null && m_vOffset != Vector2.zero)
                {
                    force = OffsetPursuit(m_pTargetAgent1, m_vOffset);

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    if (m_pTargetAgent1 == null) Debug.LogError("pursuit target not assigned");
                    if (m_vOffset == Vector2.zero) Debug.LogError("No offset assigned");
                }
            }

            if (On(behavior_type.interpose))
            {

                if (m_pTargetAgent1 != null && m_pTargetAgent2 != null)
                {
                    force = Interpose(m_pTargetAgent1, m_pTargetAgent2) * m_dWeightInterpose;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    Debug.LogError("pursuit target not assigned");
                }
            }

            if (On(behavior_type.hide))
            {
                if (m_pTargetAgent1 != null)
                {
                    force = Hide(m_pTargetAgent1, AgentManager.Instance.Obstacles()) * m_dWeightPursuit;
                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    Debug.LogError("hide target not assigned");
                }
            }

            if (On(behavior_type.follow_path))
            {
                if (AgentManager.Instance.GetPath() != null)
                {
                    force = FollowPath() * m_dWeightFollowPath;

                    if (!AccumulateForce(ref m_vSteeringForce, force)) return m_vSteeringForce;
                }
                else
                {
                    Debug.LogError("Path havent initilized");
                }
            }

            return m_vSteeringForce;
        }
        private Vector2 CalculateDithered() { return new Vector2(); }

        //helper method for Hide. Returns a position located on the other
        //side of an obstacle to the pursuer
        private Vector2 GetHidingPosition(Vector2 posOb,
                                          float radiusOb,
                                          Vector2 posHunter)
        {
            //calculate how far away the agent is to be from the chosen obstacle's
            //bounding radius
            const float DistanceFromBoundary = 3.0f;
            float DistAway = radiusOb + DistanceFromBoundary;

            //calculate the heading toward the object from the hunter
            Vector2 ToOb = (posOb - posHunter).normalized;

            //scale it to size and add to the obstacles position to get
            //the hiding spot.
            return (ToOb * DistAway) + posOb;
        }

        //------------------------------- CreateFeelers --------------------------
        //
        //  Creates the antenna utilized by WallAvoidance
        //------------------------------------------------------------------------
        private void CreateFeelers()
        {
            //feeler pointing straight in front
            m_Feelers[0] = m_MovementAgent.Pos() + m_fWallDetectionFeelerLength * m_MovementAgent.Facing();

            //feeler to left
            //Vector2 temp = m_pVehicle.Facing();

            float angle = 35;

            var temp = Common.Vec2DRotateAroundOrigin(m_MovementAgent.Facing(), angle);

            m_Feelers[1] = m_MovementAgent.Pos() + m_fWallDetectionFeelerLength / 1.5f * temp;

            //feeler to right
            angle = -35;
            temp = Common.Vec2DRotateAroundOrigin(m_MovementAgent.Facing(), angle);

            m_Feelers[2] = m_MovementAgent.Pos() + m_fWallDetectionFeelerLength / 1.5f * temp;
        }

        public SteeringBehavior(MovingEntity agent, SteeringPrm prm, Deceleration decel, MovingEntity target1 = null, MovingEntity target2 = null)
        {
            m_MovementAgent = agent;
            m_iFlags = 0;
            m_dDBoxLength = prm.MinDetectionBoxLength;
            miniDetectionBoxFactor = prm.MinDetectionBoxLength;
            m_dWeightCohesion = prm.CohesionWeight;
            m_dWeightAlignment = prm.AlignmentWeight;
            m_dWeightSeparation = prm.SeparationWeight;
            m_dWeightObstacleAvoidance = prm.ObstacleAvoidanceWeight;
            m_dWeightWander = prm.WanderWeight;
            m_dWeightWallAvoidance = prm.WallAvoidanceWeight;
            m_fViewDistance = prm.ViewDistance;
            m_fWallDetectionFeelerLength = prm.WallDetectionFeelerLength;
            m_Feelers = new Vector2[3];
            m_Deceleration = decel;
            m_pTargetAgent1 = target1;
            m_pTargetAgent2 = target2;
            m_dWanderDistance = WanderDist;
            m_dWanderJitter = WanderJitterPerSec;
            m_dWanderRadius = WanderRad;
            m_dWaypointSeekDistSq = WaypointSeekDist * WaypointSeekDist;

            m_dWeightSeek = prm.SeekWeight;
            m_dWeightFlee = prm.FleeWeight;
            m_dWeightArrive = prm.ArriveWeight;
            m_dWeightPursuit = prm.PursuitWeight;
            m_dWeightOffsetPursuit = prm.OffsetPursuitWeight;
            m_dWeightInterpose = prm.InterposeWeight;
            m_dWeightHide = prm.HideWeight;
            m_dWeightEvade = prm.EvadeWeight;
            m_dWeightFollowPath = prm.FollowPathWeight;
            m_bCellSpaceOn = false;
            m_SummingMethod = summing_method.prioritized;

            float theta = Random.value * Mathf.PI * 2.0f;

            m_vWanderTarget = new Vector2(m_dWanderRadius * Mathf.Cos(theta), m_dWanderRadius * Mathf.Sin(theta));

            InitializeBehaviorDictionary();
        }
        private void InitializeBehaviorDictionary()
        {
            ///seek                 1
            ///flee                 2
            ///arrive               3
            ///wander               4
            ///cohesion             5
            ///seperation           6
            ///allignment           7
            ///obstacle_avoidance   8
            ///wall_avoidance       9
            ///follow_path          10
            ///pursuit              11
            ///evade                12
            ///interpose            13
            ///hide                 14
            ///flock                15
            ///offset_pursuit       16

            m_dicBehaviors.Add((int)behavior_type.seek, false);
            m_dicBehaviors.Add((int)behavior_type.flee, false);
            m_dicBehaviors.Add((int)behavior_type.arrive, false);
            m_dicBehaviors.Add((int)behavior_type.wander, false);
            m_dicBehaviors.Add((int)behavior_type.cohesion, false);
            m_dicBehaviors.Add((int)behavior_type.separation, false);
            m_dicBehaviors.Add((int)behavior_type.alignment, false);
            m_dicBehaviors.Add((int)behavior_type.obstacle_avoidance, false);
            m_dicBehaviors.Add((int)behavior_type.wall_avoidance, false);
            m_dicBehaviors.Add((int)behavior_type.follow_path, false);
            m_dicBehaviors.Add((int)behavior_type.pursuit, false);
            m_dicBehaviors.Add((int)behavior_type.evade, false);
            m_dicBehaviors.Add((int)behavior_type.interpose, false);
            m_dicBehaviors.Add((int)behavior_type.hide, false);
            m_dicBehaviors.Add((int)behavior_type.flock, false);
            m_dicBehaviors.Add((int)behavior_type.offset_pursuit, false);

        }
        public void RefreshSteeringPrm(SteeringPrm prm)
        {
            m_fViewDistance = prm.ViewDistance;
            m_dDBoxLength = prm.MinDetectionBoxLength;
            m_fWallDetectionFeelerLength = prm.WallDetectionFeelerLength;

            m_dWeightSeparation = prm.SeparationWeight;
            m_dWeightAlignment = prm.AlignmentWeight;
            m_dWeightCohesion = prm.CohesionWeight;

            m_dWeightObstacleAvoidance = prm.ObstacleAvoidanceWeight;
            m_dWeightWallAvoidance = prm.WallAvoidanceWeight;

            m_dWeightWander = prm.WanderWeight;
            m_dWeightSeek = prm.SeekWeight;
            m_dWeightFlee = prm.FleeWeight;
            m_dWeightArrive = prm.ArriveWeight;
            m_dWeightPursuit = prm.PursuitWeight;
            m_dWeightOffsetPursuit = prm.OffsetPursuitWeight;
            m_dWeightInterpose = prm.InterposeWeight;
            m_dWeightHide = prm.HideWeight;
            m_dWeightEvade = prm.EvadeWeight;
            m_dWeightFollowPath = prm.FollowPathWeight;
        }
        //calculates and sums the steering forces from any active behaviors
        public Vector2 Calculate()
        {
            //reset the steering force
            m_vSteeringForce = Vector2.zero;

            //use space partitioning to calculate the neighbours of this vehicle
            //if switched on. If not, use the standard tagging system
            if (!isSpacePartitioningOn())
            {
                //tag neighbors if any of the following 3 group behaviors are switched on
                if (On(behavior_type.separation) || On(behavior_type.alignment) || On(behavior_type.cohesion))
                {
                    //AgentManager.Instance.TagVehiclesWithinViewRange(m_MovementAgent, m_fViewDistance);
                    Common.TagVehiclesWithinViewRangeForFLocking(m_MovementAgent, m_fViewDistance);
                }
            }
            else
            {
                //calculate neighbours in cell-space if any of the following 3 group
                //behaviors are switched on
                if (On(behavior_type.separation) || On(behavior_type.alignment) || On(behavior_type.cohesion))
                {
                    //m_pVehicle->World()->CellSpace()->CalculateNeighbors(m_pVehicle->Pos(), m_dViewDistance);
                }
            }

            switch (m_SummingMethod)
            {
                case summing_method.weighted_average:

                    m_vSteeringForce = CalculateWeightedSum(); break;

                case summing_method.prioritized:

                    m_vSteeringForce = CalculatePrioritized(); break;

                case summing_method.dithered:

                    m_vSteeringForce = CalculateDithered(); break;

                default: m_vSteeringForce = new Vector2(0, 0); break;

            }//end switch

            return m_vSteeringForce;
        }
        //calculates the component of the steering force that is parallel
        //with the vehicle heading
        public float ForwardComponent() { return m_vSteeringForce.y; }
        //calculates the component of the steering force that is perpendicuar
        //with the vehicle heading
        public float SideComponent() { return m_vSteeringForce.x; }
        //renders visual aids and info for seeing how each behavior is
        //calculated
        public void RenderAids() { }
        public void SetTarget(Vector2 t) { m_vTarget = t; }
        public void SetTargetAgent1(MovingEntity Agent) { m_pTargetAgent1 = Agent; }
        public MovingEntity GetTargetAgent1() { return m_pTargetAgent1; }
        public void SetTargetAgent2(MovingEntity Agent) { m_pTargetAgent2 = Agent; }
        public MovingEntity GetTargetAgent2() { return m_pTargetAgent2; }
        public void SetOffset(Vector2 offset) { m_vOffset = offset; }
        public Vector2 GetOffset() { return m_vOffset; }
        //void SetPath(std::list<Vector2> new_path) { m_Path->Set(new_path); }
        //void CreateRandomPath(int num_waypoints, int mx, int my, int cx, int cy) {m_pPath->CreateRandomPath(num_waypoints, mx, my, cx, cy);}
        public Vector2 Force() { return m_vSteeringForce; }
        public void ToggleSpacePartitioningOnOff() { m_bCellSpaceOn = !m_bCellSpaceOn; }
        public Dictionary<int, bool> GetDicOfBehaviors() { return m_dicBehaviors; }
        public bool isSpacePartitioningOn() { return m_bCellSpaceOn; }
        public void SetSummingMethod(summing_method sm) { m_SummingMethod = sm; }
        public void FleeOn(Vehicle v) { m_iFlags |= ((int)behavior_type.flee); m_pTargetAgent1 = v; m_dicBehaviors[(int)behavior_type.flee] = true; }
        public void SeekOn(Vehicle v) { m_iFlags |= ((int)behavior_type.seek); m_pTargetAgent1 = v; m_dicBehaviors[(int)behavior_type.seek] = true; }
        public void ArriveOn(Vehicle v) { m_iFlags |= ((int)behavior_type.arrive); m_pTargetAgent1 = v; m_dicBehaviors[(int)behavior_type.arrive] = true; }
        public void WanderOn() { m_iFlags |= ((int)behavior_type.wander); m_dicBehaviors[(int)behavior_type.wander] = true; }
        public void PursuitOn(Vehicle v) { m_iFlags |= ((int)behavior_type.pursuit); m_pTargetAgent1 = v; m_dicBehaviors[(int)behavior_type.pursuit] = true; }
        public void EvadeOn(Vehicle v) { m_iFlags |= ((int)behavior_type.evade); m_pTargetAgent1 = v; m_dicBehaviors[(int)behavior_type.evade] = true; }
        public void CohesionOn() { m_iFlags |= ((int)behavior_type.cohesion); m_dicBehaviors[(int)behavior_type.cohesion] = true; }
        public void SeparationOn() { m_iFlags |= ((int)behavior_type.separation); m_dicBehaviors[(int)behavior_type.separation] = true; }
        public void AlignmentOn() { m_iFlags |= ((int)behavior_type.alignment); m_dicBehaviors[(int)behavior_type.alignment] = true; }
        public void ObstacleAvoidanceOn() { m_iFlags |= ((int)behavior_type.obstacle_avoidance); m_dicBehaviors[(int)behavior_type.obstacle_avoidance] = true; }
        public void WallAvoidanceOn() { m_iFlags |= ((int)behavior_type.wall_avoidance); m_dicBehaviors[(int)behavior_type.wall_avoidance] = true; }
        public void FollowPathOn() { m_iFlags |= ((int)behavior_type.follow_path); m_dicBehaviors[(int)behavior_type.follow_path] = true; }
        public void InterposeOn(Vehicle v1, Vehicle v2) { m_iFlags |= ((int)behavior_type.interpose); m_pTargetAgent1 = v1; m_pTargetAgent2 = v2; m_dicBehaviors[(int)behavior_type.interpose] = true; }
        public void HideOn(Vehicle v) { m_iFlags |= ((int)behavior_type.hide); m_pTargetAgent1 = v; m_dicBehaviors[(int)behavior_type.hide] = true; }
        public void OffsetPursuitOn(Vehicle v1, Vector2 offset) { m_iFlags |= ((int)behavior_type.offset_pursuit); m_vOffset = offset; m_pTargetAgent1 = v1; m_dicBehaviors[(int)behavior_type.offset_pursuit] = true; }
        public void FlockingOn()
        {
            CohesionOn();
            AlignmentOn();
            SeparationOn();
            WanderOn();
        }

        public void FleeOff() { if (On(behavior_type.flee)) m_iFlags ^= ((int)behavior_type.flee); m_dicBehaviors[(int)behavior_type.flee] = false; }
        public void SeekOff() { if (On(behavior_type.seek)) m_iFlags ^= ((int)behavior_type.seek); m_dicBehaviors[(int)behavior_type.seek] = false; }
        public void ArriveOff() { if (On(behavior_type.arrive)) m_iFlags ^= ((int)behavior_type.arrive); m_dicBehaviors[(int)behavior_type.arrive] = false; }
        public void WanderOff() { if (On(behavior_type.wander)) m_iFlags ^= ((int)behavior_type.wander); m_dicBehaviors[(int)behavior_type.wander] = false; }
        public void PursuitOff() { if (On(behavior_type.pursuit)) m_iFlags ^= ((int)behavior_type.pursuit); m_dicBehaviors[(int)behavior_type.pursuit] = false; }
        public void EvadeOff() { if (On(behavior_type.evade)) m_iFlags ^= ((int)behavior_type.evade); m_dicBehaviors[(int)behavior_type.evade] = false; }
        public void CohesionOff() { if (On(behavior_type.cohesion)) m_iFlags ^= ((int)behavior_type.cohesion); m_dicBehaviors[(int)behavior_type.cohesion] = false; }
        public void SeparationOff() { if (On(behavior_type.separation)) m_iFlags ^= ((int)behavior_type.separation); m_dicBehaviors[(int)behavior_type.separation] = false; }
        public void AlignmentOff() { if (On(behavior_type.alignment)) m_iFlags ^= ((int)behavior_type.alignment); m_dicBehaviors[(int)behavior_type.alignment] = false; }
        public void ObstacleAvoidanceOff() { if (On(behavior_type.obstacle_avoidance)) m_iFlags ^= ((int)behavior_type.obstacle_avoidance); m_dicBehaviors[(int)behavior_type.obstacle_avoidance] = false; }
        public void WallAvoidanceOff() { if (On(behavior_type.wall_avoidance)) m_iFlags ^= ((int)behavior_type.wall_avoidance); m_dicBehaviors[(int)behavior_type.wall_avoidance] = false; }
        public void FollowPathOff() { if (On(behavior_type.follow_path)) m_iFlags ^= ((int)behavior_type.follow_path); m_dicBehaviors[(int)behavior_type.follow_path] = false; }
        public void InterposeOff() { if (On(behavior_type.interpose)) m_iFlags ^= ((int)behavior_type.interpose); m_dicBehaviors[(int)behavior_type.interpose] = false; }
        public void HideOff() { if (On(behavior_type.hide)) m_iFlags ^= ((int)behavior_type.hide); m_dicBehaviors[(int)behavior_type.hide] = false; }
        public void OffsetPursuitOff() { if (On(behavior_type.offset_pursuit)) m_iFlags ^= ((int)behavior_type.offset_pursuit); m_dicBehaviors[(int)behavior_type.offset_pursuit] = false; }
        public void FlockingOff() { CohesionOff(); AlignmentOff(); SeparationOff(); WanderOff(); }

        public bool isFleeOn() { return On(behavior_type.flee); }
        public bool isSeekOn() { return On(behavior_type.seek); }
        public bool isArriveOn() { return On(behavior_type.arrive); }
        public bool isWanderOn() { return On(behavior_type.wander); }
        public bool isPursuitOn() { return On(behavior_type.pursuit); }
        public bool isEvadeOn() { return On(behavior_type.evade); }
        public bool isCohesionOn() { return On(behavior_type.cohesion); }
        public bool isSeparationOn() { return On(behavior_type.separation); }
        public bool isAlignmentOn() { return On(behavior_type.alignment); }
        public bool isObstacleAvoidanceOn() { return On(behavior_type.obstacle_avoidance); }
        public bool isWallAvoidanceOn() { return On(behavior_type.wall_avoidance); }
        public bool isFollowPathOn() { return On(behavior_type.follow_path); }
        public bool isInterposeOn() { return On(behavior_type.interpose); }
        public bool isHideOn() { return On(behavior_type.hide); }
        public bool isOffsetPursuitOn() { return On(behavior_type.offset_pursuit); }
        public float DBoxLength() { return m_dDBoxLength; }
        public Vector2[] GetFeelers() { return m_Feelers; }
        public float WanderJitter() { return m_dWanderJitter; }
        public float WanderDistance() { return m_dWanderDistance; }
        public float WanderRadius() { return m_dWanderRadius; }
        public float SeparationWeight() { return m_dWeightSeparation; }
        public float AlignmentWeight() { return m_dWeightAlignment; }
        public float CohesionWeight() { return m_dWeightCohesion; }
        public float GetViewDist() { return m_fViewDistance; }

    }
}