using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

namespace SimpleSteering { 
[CreateAssetMenu(fileName = "WorldPrm", menuName = "World_Parameter", order = 1)]
public class GameWorldPrm : ScriptableObject 
{
    #region MovementAgent Part
    [HideInInspector] public List<bool> isFoldouts = new List<bool>();
    [HideInInspector] public List<MovementAgent<MovementComponent>> m_Prefebs = new List<MovementAgent<MovementComponent>>();
    [HideInInspector] public List<int> m_NumsofEachPrefeb = new List<int>();
    [HideInInspector] public List<float> m_MxRange_min = new List<float>();
    [HideInInspector] public List<float> m_MxRange_max = new List<float>();
    [HideInInspector] public List<float> m_MyRange_min = new List<float>();
    [HideInInspector] public List<float> m_MyRange_max = new List<float>();
    #endregion

    #region ObstacleAgent Part
    [HideInInspector] public int NumObstacles;
    [HideInInspector] public float MinObstacleRadius;
    [HideInInspector] public float MaxObstacleRadius;

    [HideInInspector] public float m_OxRange_min;
    [HideInInspector] public float m_OxRange_max;
    [HideInInspector] public float m_OyRange_min;
    [HideInInspector] public float m_OyRange_max;

    [HideInInspector] public ObstacleAgent<ObstacleComponent> m_ObstaclePrefeb;
    #endregion

    //number of horizontal cells used for spatial partitioning
    [HideInInspector] public int NumCellsX;
    //number of vertical cells used for spatial partitioning
    [HideInInspector] public int NumCellsY;
    private SteeringBehavior.behavior_type GetBehavior_Type(int id)
    {
        switch (id)
        {
            case 1:
                return SteeringBehavior.behavior_type.seek;
            case 2:
                return SteeringBehavior.behavior_type.flee;
            case 3:
                return SteeringBehavior.behavior_type.arrive;
            case 4:
                return SteeringBehavior.behavior_type.wander;
            case 5:
                return SteeringBehavior.behavior_type.cohesion;
            case 6:
                return SteeringBehavior.behavior_type.separation;
            case 7:
                return SteeringBehavior.behavior_type.alignment;
            case 8:
                return SteeringBehavior.behavior_type.obstacle_avoidance;
            case 9:
                return SteeringBehavior.behavior_type.wall_avoidance;
            case 10:
                return SteeringBehavior.behavior_type.follow_path;
            case 11:
                return SteeringBehavior.behavior_type.pursuit;
            case 12:
                return SteeringBehavior.behavior_type.evade;
            case 13:
                return SteeringBehavior.behavior_type.interpose;
            case 14:
                return SteeringBehavior.behavior_type.hide;
            case 15:
                return SteeringBehavior.behavior_type.flock;
            case 16:
                return SteeringBehavior.behavior_type.offset_pursuit;
            default:
                return SteeringBehavior.behavior_type.none;
        }
    }

    //public void ReadAgentPos()
    //{
    //    string posloca = Application.streamingAssetsPath + "/LoadResource/worldPrm.csv";
    //    using (var reader = new StreamReader(posloca))
    //    {
    //        var line = reader.ReadLine().Split(',');
    //        while (line[0] != "NumofAgent")
    //        {
    //            line = reader.ReadLine().Split(',');
    //        }
    //        NumAgents = int.Parse(line[1]);
    //        agentPositions = new Vector3[NumAgents];
    //        reader.ReadLine();
    //        for (int i = 0; i < NumAgents; ++i)
    //        {
    //            if (!reader.EndOfStream)
    //            {
    //                line = reader.ReadLine().Split(',');
    //                agentPositions[i] = new Vector3(float.Parse(line[0].ToString()), float.Parse(line[1].ToString()), float.Parse(line[2].ToString()));
    //            }
    //            else
    //            {
    //                Debug.LogError("the position file has something wrong");
    //            }
    //        }

    //        /// the code bellow try to read the switch on behavior for each chatacter
    //        /// 

    //        while(line[0] != "SteeringBehavior")
    //        {
    //            line = reader.ReadLine().Split(',');
    //        }

    //        reader.ReadLine();
    //        behaviorSwitches = new List<SteeringBehavior.behavior_type[]>();
    //        List<SteeringBehavior.behavior_type> temp = new List<SteeringBehavior.behavior_type>();
    //        for (int i = 0; i < NumAgents; ++i)
    //        {
    //            line = reader.ReadLine().Split(',');
    //            int count = line.Length;
    //            ///seek                 1
    //            ///flee                 2
    //            ///arrive               3
    //            ///wander               4
    //            ///cohesion             5
    //            ///seperation           6
    //            ///allignment           7
    //            ///obstacle_avoidance   8
    //            ///wall_avoidance       9
    //            ///follow_path          10
    //            ///pursuit              11
    //            ///evade                12
    //            ///interpose            13
    //            ///hide                 14
    //            ///flock                15
    //            ///offset_pursuit       16
    //            temp.Clear();
    //            for (int a = 1; a < count; ++a)
    //            {
    //                if (int.TryParse(line[a], out int result))
    //                {
    //                    temp.Add(GetBehavior_Type(a));
    //                }
    //            }

    //            int steeringOnCount = temp.Count;
    //            var behaviorSwitchOns = new SteeringBehavior.behavior_type[steeringOnCount];
    //            for (int o = 0; o < steeringOnCount; ++o)
    //            {
    //                behaviorSwitchOns[o] = temp[o];
    //            }
    //            behaviorSwitches.Add(behaviorSwitchOns);
    //        }
    //        /// the code bellow read the target
    //        /// 
    //        vehiclesTarget = new List<(int, (int, int))>();
    //        while (line[0] != "Target")
    //        {
    //            line = reader.ReadLine().Split(',');
    //        }
    //        (int, (int, int)) temp2;
    //        //while (!reader.EndOfStream)
    //        for (int i = 0; i < NumAgents; ++i)
    //        {
    //            line = reader.ReadLine().Split(',');

    //            if(!int.TryParse(line[1], out int result))
    //            {
    //                temp2.Item1 = int.Parse(line[0]);
    //                temp2.Item2.Item1 = temp2.Item2.Item2 = -1;
    //                vehiclesTarget.Add(temp2);
    //            }
    //            else
    //            {
    //                if (!int.TryParse(line[2], out int result2))
    //                {
    //                    temp2.Item1 = int.Parse(line[0]);
    //                    temp2.Item2.Item1 = int.Parse(line[1]);
    //                    temp2.Item2.Item2 = -1;
    //                    vehiclesTarget.Add(temp2);
    //                }
    //                else
    //                {
    //                    temp2.Item1 = int.Parse(line[0]);
    //                    temp2.Item2.Item1 = int.Parse(line[1]);
    //                    temp2.Item2.Item2 = int.Parse(line[2]);
    //                    vehiclesTarget.Add(temp2);
    //                }
    //            }
    //        }

    //        while(line[0] != "NumofObstacle")
    //        {
    //            line = reader.ReadLine().Split(',');
    //        }
    //        NumObstacles = int.Parse(line[1]);
    //        obstaclePositions = new Vector3[NumObstacles];
    //        reader.ReadLine(); // 读取标头
    //        for (int i = 0; i < NumObstacles; ++i)
    //        {
    //            if (!reader.EndOfStream)
    //            {
    //                line = reader.ReadLine().Split(',');
    //                obstaclePositions[i] = new Vector3(float.Parse(line[0]), float.Parse(line[1]), float.Parse(line[2]));
    //            }
    //            else
    //            {
    //                Debug.LogError("the position file has something wrong at obstacles position part");
    //            }
    //        }
    //        obstacleRadius = new float[NumObstacles];
    //        reader.ReadLine();
    //        // 读取标头
    //        for (int i = 0; i < NumObstacles; ++i)
    //        {
    //            if (!reader.EndOfStream)
    //            {
    //                line = reader.ReadLine().Split(',');
    //                obstacleRadius[i] = float.Parse(line[0]);
    //            }
    //            else
    //            {
    //                Debug.LogError("the position file has something wrong at obstacles scale part");
    //            }
    //        }

    //        line = null;
    //        reader.DiscardBufferedData();
    //        reader.Close();
    //    }
    //}
}
}
