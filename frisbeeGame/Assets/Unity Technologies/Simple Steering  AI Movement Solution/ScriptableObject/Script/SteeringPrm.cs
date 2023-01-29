using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SimpleSteering
{
    [CreateAssetMenu(fileName = "SteeringPrm", menuName = "Steering_Parameter", order = 0)]
    public class SteeringPrm : ScriptableObject
    {
        [Header("Detection Parameter", order = 1)]
        //how close a neighbour must be before an agent perceives it (considers it
        //to be within its neighborhood)
        public float ViewDistance;

        //used in obstacle avoidance
        public float MinDetectionBoxLength;

        //used in wall avoidance
        public float WallDetectionFeelerLength;

        [Header("SteeringBehaviorWeight", order = 3)]
        public float SeparationWeight;
        public float AlignmentWeight;
        public float CohesionWeight;
        public float ObstacleAvoidanceWeight;
        public float WallAvoidanceWeight;
        public float WanderWeight;
        public float SeekWeight;
        public float FleeWeight;
        public float ArriveWeight;
        public float PursuitWeight;
        public float OffsetPursuitWeight;
        public float InterposeWeight;
        public float HideWeight;
        public float EvadeWeight;
        public float FollowPathWeight;

        //[Header("Steering probabilities", order = 4)]
        //these are the probabilities that a steering behavior will be used
        //when the prioritized dither calculate method is used

        //public float prWallAvoidance;
        //public float prObstacleAvoidance;
        //public float prSeparation;
        //public float prAlignment;
        //public float prCohesion;
        //public float prWander;
        //public float prSeek;
        //public float prFlee;
        //public float prEvade;
        //public float prHide;
        //public float prArrive;

        [Header("Steering Type", order = 5), HideInInspector]
        //public SteeringBehavior.behavior_type[] behavior_Type;
        //public SteeringBehavior.behavior_type behavior_Type;
        public SteeringBehavior.behavior_type_for_switch behavior_Type;
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(SteeringPrm))]
    public class SteeringPrm_Editor : Editor
    {
        private SteeringPrm script;
        private SerializedObject obj;

        private void OnEnable()
        {
            obj = new SerializedObject(target);
            script = target as SteeringPrm;
        }

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            obj.Update();

            EditorGUI.BeginChangeCheck();

            //script.behavior_Type = (SteeringBehavior.behavior_type)EditorGUILayout.EnumFlagsField("Behavior Types", script.behavior_Type);
            script.behavior_Type = (SteeringBehavior.behavior_type_for_switch)EditorGUILayout.EnumFlagsField("Behavior Types", script.behavior_Type);

            EditorGUI.EndChangeCheck();

            if (GUI.changed)
            {
                obj.ApplyModifiedProperties();
            }
        }
    }
#endif
}

