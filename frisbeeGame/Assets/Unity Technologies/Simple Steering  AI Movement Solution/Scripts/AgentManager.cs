using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class AgentManager : MonoBehaviourSingleton<AgentManager>
{
    public bool IsUseWorldPrm;
    [HideInInspector] public GameWorldPrm m_worldPrm;
    [SerializeField] private float m_groundHeight;
    private List<Vehicle> m_Vehicles = new List<Vehicle>();
    private List<Obstacle> m_Obstacles = new List<Obstacle>();
    private List<Wall2D> m_Walls = new List<Wall2D>();
    private MovementAgent<MovementComponent> Role;
    
    private int RoleID = 0;
    public float GetGroundHeight() { return m_groundHeight; }
    public List<Obstacle> Obstacles() { return m_Obstacles; }
    public List<Wall2D> Walls() { return m_Walls; }
    public List<Vehicle> Vehicles() { return m_Vehicles; }
    public Path GetPath() { return m_Path; }
    public int GetRoleID() { return RoleID; }
    public MovementAgent<MovementComponent> GetRole() { return Role; }
    private Path m_Path = new Path();
    protected override void SingletonAwakened()
    {
        Application.targetFrameRate = 30;

        var agents = GameObject.FindObjectsOfType(typeof(MovementAgent<MovementComponent>));
        int count = agents.Length;

        if (count > 0)
        {
            foreach (var agent in (MovementAgent<MovementComponent>[])agents)
            {
                int type;
                SetTag(agent.TagFilter, out type);

                agent.GetVehicle().InitializedVehicleSystem(type, m_worldPrm, agent, this, -1);
                m_Vehicles.Add(agent.GetVehicle());

                if (agent.GetVehicle().GetEntityType() == ((int)EntityType.PlayerVehicle)) { RoleID = agent.GetVehicle().GetID(); Role = agent; }

                EntityManager.Instance.RegisterEntity(agent.GetVehicle());

                var turnOnBehaviors = agent.GetMovementPrm().steeringPrm.behavior_Type;

                TurnOnBehavior(turnOnBehaviors, agent.GetVehicle().Steering());
            }
        }
        #region Initialized with WorldPrm
        if (m_worldPrm != null)
            InitializedWorldPrm(m_worldPrm);
        #endregion
        var obstacles = FindObjectsOfType(typeof(ObstacleAgent<ObstacleComponent>));
        count = obstacles.Length;
        if (count > 0)
        {
            foreach (var obstacle in (ObstacleAgent<ObstacleComponent>[])obstacles)
            {
                int type = (int)EntityType.Obstacle;

                obstacle.GetObstacle().InitializedObstacle(type,m_worldPrm,obstacle,this,-1);

                m_Obstacles.Add(obstacle.GetObstacle());

                EntityManager.Instance.RegisterEntity(obstacle.GetObstacle());
            }
        }

        var walls = FindObjectsOfType(typeof(WallAgent<Wall2DComponent>));
        count = walls.Length;
        if (count > 0)
        {
            foreach (var wall in (WallAgent<Wall2DComponent>[])walls)
            {
                int type = (int)EntityType.Wall;

                wall.InitializedWall2D(type);

                EntityManager.Instance.RegisterEntity(wall.GetWall());
                m_Walls.Add(wall.GetWall());
            }
        }
    }
    protected override void SingletonStarted()
    {
        foreach (var ve in m_Vehicles)
        {
            ve.AgentStart();
        }
    }
    protected override void SingletonDestroyed()
    {
        
    }
    void Update()
    {
        /// Update steering
        
        foreach(var ve in m_Vehicles)
        {
            if (!ve.IsPlayerControl())
                ve.UpdateSteeringAndRotate();
            else
            {
                var xx = Input.GetAxis("Horizontal");
                var zz = Input.GetAxis("Vertical");
                Vector2 dir = new Vector2(xx, zz);
                ve.UpdateSteeringAndRotateInPlayerControl(dir);
            }
        }
        /// Update Agent's Transform
        foreach (var ve in m_Vehicles)
        {
            ve.UpdateTransform(m_groundHeight);
        }
        /// Render
        foreach (var wall in m_Walls)
        {
            wall.Render(true);
        }
        if(m_Path.isRender())
            m_Path.Render();
        
        foreach(var ve in m_Vehicles)
        {
            ve.AgentUpdate();
        }
    }

    private void InitializedWorldPrm(GameWorldPrm worldPrm)
    {
        int agentPrefebCount = worldPrm.m_Prefebs.Count;
        int obstacleCount = worldPrm.NumObstacles;

        if (agentPrefebCount > 0)
        {
            for (int i = 0; i < agentPrefebCount; ++i)
            {
                for (int n = 0; n < worldPrm.m_NumsofEachPrefeb[i]; ++n)
                {
                    var newAgent = worldPrm.m_Prefebs[i];

                    var obj = GameObject.Instantiate(newAgent);
                    SetTag(obj.TagFilter, out int type);
                    obj.GetVehicle().InitializedVehicleSystem_GWP(type, m_worldPrm, obj, this, i);

                    m_Vehicles.Add(obj.GetVehicle());

                    EntityManager.Instance.RegisterEntity(obj.GetVehicle());

                    var turnOnBehaviors = obj.GetMovementPrm().steeringPrm.behavior_Type;

                    TurnOnBehavior(turnOnBehaviors, obj.GetVehicle().Steering());
                }
            }
        }

        if (obstacleCount > 0)
        {
            int type = (int)EntityType.Obstacle;
            for (int i = 0; i < obstacleCount; ++i)
            {
                var obj = GameObject.Instantiate(worldPrm.m_ObstaclePrefeb);

                obj.GetObstacle().InitializedObstacle_GWP(type, worldPrm, obj, this);

                m_Obstacles.Add(obj.GetObstacle());
            }
        }

    }
    public void RefreshSteeringPrm()
    {
        foreach(var vehicle in m_Vehicles)
        {
            vehicle.RefreshSteeringPrm(vehicle.Agent().GetComponent<MovementAgent<MovementComponent>>().GetMovementPrm());
        }
    }
    private void TurnOnBehavior(SteeringBehavior.behavior_type_for_switch type, SteeringBehavior steering, Vehicle target1 = null, Vehicle target2 = null)
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
        if ((type & SteeringBehavior.behavior_type_for_switch.wander) != 0)
            steering.WanderOn();
        if ((type & SteeringBehavior.behavior_type_for_switch.obstacle_avoidance) != 0)
            steering.ObstacleAvoidanceOn();
        if ((type & SteeringBehavior.behavior_type_for_switch.wall_avoidance) != 0)
            steering.WallAvoidanceOn();
        if ((type & SteeringBehavior.behavior_type_for_switch.flock) != 0)
            steering.FlockingOn();
    }
    private void TurnOnBehavior_Original2(SteeringBehavior.behavior_type type, SteeringBehavior steering, Vehicle target1 = null, Vehicle target2 = null)
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

        if ((type & SteeringBehavior.behavior_type.seek) != 0)
            steering.SeekOn(target1);
        if ((type & SteeringBehavior.behavior_type.flee) != 0)
            steering.FleeOn(target1);
        if ((type & SteeringBehavior.behavior_type.arrive) != 0)
            steering.ArriveOn(target1);
        if ((type & SteeringBehavior.behavior_type.wander) != 0)
            steering.WanderOn();
        if ((type & SteeringBehavior.behavior_type.cohesion) != 0)
            steering.CohesionOn();
        if ((type & SteeringBehavior.behavior_type.separation) != 0)
            steering.SeparationOn();
        if ((type & SteeringBehavior.behavior_type.alignment) != 0)
            steering.AlignmentOn();
        if ((type & SteeringBehavior.behavior_type.obstacle_avoidance) != 0)
            steering.ObstacleAvoidanceOn();
        if ((type & SteeringBehavior.behavior_type.wall_avoidance) != 0)
            steering.WallAvoidanceOn();
        if ((type & SteeringBehavior.behavior_type.follow_path) != 0)
            steering.FollowPathOn();
        if ((type & SteeringBehavior.behavior_type.pursuit) != 0)
            steering.PursuitOn(target1);
        if ((type & SteeringBehavior.behavior_type.evade) != 0)
            steering.EvadeOn(target1);
        if ((type & SteeringBehavior.behavior_type.interpose) != 0)
            steering.InterposeOn(target1, target2);
        if ((type & SteeringBehavior.behavior_type.hide) != 0)
            steering.HideOn(target1);
        if ((type & SteeringBehavior.behavior_type.flock) != 0)
            steering.FlockingOn();
        if ((type & SteeringBehavior.behavior_type.offset_pursuit) != 0)
            steering.OffsetPursuitOn(target1, target1.Facing());
    }
    private void TurnOnBehavior_OriginalVersion(SteeringBehavior.behavior_type type, SteeringBehavior steering, Vehicle target1 = null, Vehicle target2 = null)
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
        switch (type)
        {
            case SteeringBehavior.behavior_type.seek:
                steering.SeekOn(target1);
                break;
            case SteeringBehavior.behavior_type.flee:
                steering.FleeOn(target1);
                break;
            case SteeringBehavior.behavior_type.arrive:
                steering.ArriveOn(target1);
                break;
            case SteeringBehavior.behavior_type.wander:
                steering.WanderOn();
                break;
            case SteeringBehavior.behavior_type.cohesion:
                steering.CohesionOn();
                break;
            case SteeringBehavior.behavior_type.separation:
                steering.SeparationOn();
                break;
            case SteeringBehavior.behavior_type.alignment:
                steering.AlignmentOn();
                break;
            case SteeringBehavior.behavior_type.obstacle_avoidance:
                steering.ObstacleAvoidanceOn();
                break;
            case SteeringBehavior.behavior_type.wall_avoidance:
                steering.WallAvoidanceOn();
                break;
            case SteeringBehavior.behavior_type.follow_path:
                steering.FollowPathOn();
                break;
            case SteeringBehavior.behavior_type.pursuit:
                steering.PursuitOn(target1);
                break;
            case SteeringBehavior.behavior_type.evade:
                steering.EvadeOn(target1);
                break;
            case SteeringBehavior.behavior_type.interpose:
                steering.InterposeOn(target1, target2);
                break;
            case SteeringBehavior.behavior_type.hide:
                steering.HideOn(target1);
                break;
            case SteeringBehavior.behavior_type.flock:
                steering.FlockingOn();
                break;
            case SteeringBehavior.behavior_type.offset_pursuit:
                steering.OffsetPursuitOn(target1, -target1.Facing());
                break;
            default:
                break;
        }
    }
    private bool NeedATargetBehavior(SteeringBehavior.behavior_type type)
    {
        if (type == SteeringBehavior.behavior_type.pursuit
            || type == SteeringBehavior.behavior_type.evade
            || type == SteeringBehavior.behavior_type.hide
            || type == SteeringBehavior.behavior_type.offset_pursuit
            || type == SteeringBehavior.behavior_type.flee
            || type == SteeringBehavior.behavior_type.arrive
            || type == SteeringBehavior.behavior_type.seek)
            return true;
        return false;
    }
    //public void TagObstaclesWithinViewRange(BaseGameEntity m_pVehicle, float m_dDBoxLength)
    //{
    //    Common.TagNeighbors(m_pVehicle, m_Obstacles, m_dDBoxLength);
    //}
    public void TagVehiclesWithinViewRange(BaseGameEntity vehicle, float viewDistance)
    {
        List<BaseGameEntity> list = new List<BaseGameEntity>();
        foreach(var entity in Vehicles())
        {
            if ((entity.Steering().isCohesionOn() || entity.Steering().isSeparationOn() || entity.Steering().isAlignmentOn())
                && entity.GetEntityType() == vehicle.GetEntityType())
                list.Add(entity);
        }
        Common.TagNeighbors(vehicle, list, viewDistance);
    }
    private void SetTag(string tagInstring, out int type)
    {
        type = 0;
        if (tagInstring == EntityType.AIVehicle.ToString())
            type = 0;
        if (tagInstring == EntityType.PlayerVehicle.ToString())
            type = 1;
        if (tagInstring == EntityType.Obstacle.ToString())
            type = 2;
        if (tagInstring == EntityType.Wall.ToString())
            type = 3;
        if (tagInstring == EntityType.Red.ToString())
            type = 4;
        if (tagInstring == EntityType.Blue.ToString())
            type = 5;
        if (tagInstring == EntityType.Goblin.ToString())
            type = 6;
    }
    //public Vector2 Crosshair(){return m_vCrosshair;}
    //public void SetCrosshair(Vector2 v) { m_vCrosshair = v; }
    //public void SetCrosshair(Vector3 v) { m_vCrosshair = new Vector2(v.x, v.z); }
}
}