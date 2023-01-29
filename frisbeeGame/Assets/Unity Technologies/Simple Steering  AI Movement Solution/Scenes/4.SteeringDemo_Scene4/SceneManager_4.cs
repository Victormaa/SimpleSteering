using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace SimpleSteering
{
    public class SceneManager_4 : MonoBehaviour
    {
        public List<Vector2> path = new List<Vector2>();
        private List<int> Goblins = new List<int>();
        public SteeringBehavior.behavior_type m_behavior_Type;
        void Start()
        {
            AgentManager.Instance.GetPath().Set(path);
            AgentManager.Instance.GetPath().LoopOn();
            //AgentManager.Instance.GetPath().RenderOn();

            foreach (var vehicle in AgentManager.Instance.Vehicles())
            {
                vehicle.Steering().WallAvoidanceOn();
                vehicle.Steering().ObstacleAvoidanceOn();
                if (vehicle.GetEntityType() == (int)EntityType.Goblin)
                {
                    vehicle.Steering().WanderOn();
                    Goblins.Add(vehicle.GetID());
                }
            }
        }
        private void ExitCurSteering()
        {
            Dictionary<int, bool> Dic = new Dictionary<int, bool>();
            foreach (var gbl in Goblins)
            {
                var vehicle = (Vehicle)EntityManager.Instance.GetEntityFromID(gbl);

                Dic.Clear();
                foreach (var a in vehicle.Steering().GetDicOfBehaviors())
                {
                    Dic.Add(a.Key, a.Value);
                }

                foreach (var a in Dic)
                {
                    if (a.Value)
                    {
                        if ((SteeringBehavior.behavior_type)a.Key != SteeringBehavior.behavior_type.obstacle_avoidance
                        && (SteeringBehavior.behavior_type)a.Key != SteeringBehavior.behavior_type.wall_avoidance)
                            TurnOffBehaviors((SteeringBehavior.behavior_type)a.Key, a.Value, vehicle.Steering());
                    }
                }
            }
        }
        private void EnterASteering()
        {
            if ((m_behavior_Type & SteeringBehavior.behavior_type.interpose) == 0)
            {
                foreach (var gbl in Goblins)
                {
                    var vehicle = (Vehicle)EntityManager.Instance.GetEntityFromID(gbl);
                    TurnOnBehavior(m_behavior_Type, vehicle.Steering());
                }
            }
            else
            {
                Vehicle gbl1 = (Vehicle)EntityManager.Instance.GetEntityFromID(Goblins[0]);
                Vehicle gbl2 = (Vehicle)EntityManager.Instance.GetEntityFromID(Goblins[1]);

                TurnOnBehavior(m_behavior_Type, gbl1.Steering(), gbl2);
                TurnOnBehavior(SteeringBehavior.behavior_type.wander, gbl2.Steering());
            }
        }
        public void ChangeSteering()
        {
            ExitCurSteering();
            EnterASteering();
        }
        private void TurnOffBehaviors(SteeringBehavior.behavior_type behavior, bool isOn, SteeringBehavior steering)
        {
            if ((behavior == SteeringBehavior.behavior_type.seek) && isOn) //1
                steering.SeekOff();
            if ((behavior == SteeringBehavior.behavior_type.flee) && isOn) //2
                steering.FleeOff();
            if ((behavior == SteeringBehavior.behavior_type.arrive) && isOn) //3
                steering.ArriveOff();
            if ((behavior == SteeringBehavior.behavior_type.wander) && isOn) //4
                steering.WanderOff();
            if ((behavior == SteeringBehavior.behavior_type.cohesion) && isOn) //5
                steering.CohesionOff();
            if ((behavior == SteeringBehavior.behavior_type.separation) && isOn) //6
                steering.SeparationOff();
            if ((behavior == SteeringBehavior.behavior_type.alignment) && isOn) //7
                steering.AlignmentOff();
            if ((behavior == SteeringBehavior.behavior_type.obstacle_avoidance) && isOn) //8
                steering.ObstacleAvoidanceOff();
            if ((behavior == SteeringBehavior.behavior_type.wall_avoidance) && isOn) //9
                steering.WallAvoidanceOff();
            if ((behavior == SteeringBehavior.behavior_type.follow_path) && isOn) //10
                steering.FollowPathOff();
            if ((behavior == SteeringBehavior.behavior_type.pursuit) && isOn) //11
                steering.PursuitOff();
            if ((behavior == SteeringBehavior.behavior_type.evade) && isOn) //12
                steering.EvadeOff();
            if ((behavior == SteeringBehavior.behavior_type.interpose) && isOn) //13
                steering.InterposeOff();
            if ((behavior == SteeringBehavior.behavior_type.hide) && isOn) //14
                steering.HideOff();
            if ((behavior == SteeringBehavior.behavior_type.flock) && isOn) //15
                steering.FlockingOff();
            if ((behavior == SteeringBehavior.behavior_type.offset_pursuit) && isOn) //16
                steering.OffsetPursuitOff();
        }
        // Update is called once per frame
        private void TurnOnBehavior(SteeringBehavior.behavior_type type, SteeringBehavior steering, Vehicle aGbl = null)
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
                steering.SeekOn(AgentManager.Instance.GetRole().GetVehicle());
            if ((type & SteeringBehavior.behavior_type.flee) != 0)
                steering.FleeOn(AgentManager.Instance.GetRole().GetVehicle());
            if ((type & SteeringBehavior.behavior_type.arrive) != 0)
                steering.ArriveOn(AgentManager.Instance.GetRole().GetVehicle());
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
                steering.PursuitOn(AgentManager.Instance.GetRole().GetVehicle());
            if ((type & SteeringBehavior.behavior_type.evade) != 0)
                steering.EvadeOn(AgentManager.Instance.GetRole().GetVehicle());
            if ((type & SteeringBehavior.behavior_type.interpose) != 0)
                steering.InterposeOn(AgentManager.Instance.GetRole().GetVehicle(), aGbl);
            if ((type & SteeringBehavior.behavior_type.hide) != 0)
                steering.HideOn(AgentManager.Instance.GetRole().GetVehicle());
            if ((type & SteeringBehavior.behavior_type.flock) != 0)
                steering.FlockingOn();
            if ((type & SteeringBehavior.behavior_type.offset_pursuit) != 0)
                steering.OffsetPursuitOn(AgentManager.Instance.GetRole().GetVehicle(), -AgentManager.Instance.GetRole().GetVehicle().Facing());
        }
        public void DropDownFuction(int value)
        {
            if (value > 7)
                value += 2;

            value = (int)Mathf.Pow(2, value);
            m_behavior_Type = (SteeringBehavior.behavior_type)value;
        }
        void Update()
        {

        }
    }
}