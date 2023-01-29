using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SimpleSteering {
[CustomEditor(typeof(GameWorldPrm))]
public class GameWorldPrm_Editor : Editor
{
    private GameWorldPrm script;
    private SerializedObject obj;

    private void OnEnable()
    {
        obj = new SerializedObject(target);
        script = target as GameWorldPrm;
    }

    public override void OnInspectorGUI()
    {
        //DrawDefaultInspector();
        base.OnInspectorGUI();
        EditorGUI.BeginChangeCheck();
        DrawMovementAgentPrefeb();
        DrawObstacleAgent();
        if (EditorGUI.EndChangeCheck())
        {
            EditorUtility.SetDirty(script);
        }

    }

    private void DrawObstacleAgent()
    {
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Obstacles", EditorStyles.boldLabel);
        script.NumObstacles = Mathf.Max(0, EditorGUILayout.IntField("Obstacles_Size", script.NumObstacles, GUILayout.MaxWidth(340)));
        if (script.NumObstacles != 0)
        {
            EditorGUI.indentLevel++;
            script.m_ObstaclePrefeb = EditorGUILayout.ObjectField("ObstacleAgent_Prefab", script.m_ObstaclePrefeb, typeof(ObstacleAgent<ObstacleComponent>), false) as ObstacleAgent<ObstacleComponent>;
            EditorGUILayout.LabelField("Obstacle Radius");
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("MinRadius", GUILayout.MaxWidth(80));
            script.MinObstacleRadius = Mathf.Max(0, EditorGUILayout.FloatField(script.MinObstacleRadius, GUILayout.MaxWidth(80)));
            EditorGUILayout.LabelField("MaxRadius", GUILayout.MaxWidth(80));
            script.MaxObstacleRadius = EditorGUILayout.FloatField(script.MaxObstacleRadius, GUILayout.MaxWidth(80));
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Initialized Position Range");
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("x_min", GUILayout.MaxWidth(55));
            script.m_OxRange_min = EditorGUILayout.FloatField(script.m_OxRange_min, GUILayout.MaxWidth(80));
            EditorGUILayout.LabelField("x_max", GUILayout.MaxWidth(55));
            script.m_OxRange_max = EditorGUILayout.FloatField(script.m_OxRange_max, GUILayout.MaxWidth(80));
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("y_min", GUILayout.MaxWidth(55));
            script.m_OyRange_min = EditorGUILayout.FloatField(script.m_OyRange_min, GUILayout.MaxWidth(80));
            EditorGUILayout.LabelField("y_max", GUILayout.MaxWidth(55));
            script.m_OyRange_max = EditorGUILayout.FloatField(script.m_OyRange_max, GUILayout.MaxWidth(80));
            EditorGUILayout.EndHorizontal();
            EditorGUI.indentLevel--;
        }
    }
    private void DrawMovementAgentPrefeb()
    {
        EditorGUILayout.LabelField("MovementAgent", EditorStyles.boldLabel);

        int count = Mathf.Max(0, EditorGUILayout.IntField("Prefab_Size", script.isFoldouts.Count, GUILayout.MaxWidth(340)));

        List<bool> blist = script.isFoldouts;
        List<MovementAgent<MovementComponent>> mlist = script.m_Prefebs;
        List<int> ilist = script.m_NumsofEachPrefeb;
        List<float> xminlist = script.m_MxRange_min;
        List<float> xmaxlist = script.m_MxRange_max;
        List<float> yminlist = script.m_MyRange_min;
        List<float> ymaxlist = script.m_MyRange_max;

        while (count > blist.Count)
        {
            blist.Add(false);
            if(count > mlist.Count)
                mlist.Add(null);
            if (count > ilist.Count)
                ilist.Add(0);
            if (count > xminlist.Count)
                xminlist.Add(0);
            if (count > yminlist.Count)
                yminlist.Add(0);
            if (count > xmaxlist.Count)
                xmaxlist.Add(0);
            if (count > ymaxlist.Count)
                ymaxlist.Add(0);
        }
        while (count < blist.Count)
        {
            blist.RemoveAt(blist.Count - 1);
            if (count < mlist.Count)
                mlist.RemoveAt(mlist.Count - 1);
            if (count < ilist.Count)
                ilist.RemoveAt(ilist.Count - 1);
            if (count < xminlist.Count)
                xminlist.RemoveAt(xminlist.Count - 1);
            if (count < yminlist.Count)
                yminlist.RemoveAt(yminlist.Count - 1);
            if (count < xmaxlist.Count)
                xmaxlist.RemoveAt(xmaxlist.Count - 1);
            if (count < ymaxlist.Count)
                ymaxlist.RemoveAt(ymaxlist.Count - 1);
        } 

        for (int i = 0; i < blist.Count; ++i)
        {
            blist[i] = EditorGUILayout.Foldout(blist[i], "Prefab_" + (i + 1).ToString());

            if (blist[i])
            {
                EditorGUI.indentLevel++;
                DrawMovementAgentParamGUI(i);
                EditorGUI.indentLevel--;
            }
        }
    }
    private void DrawMovementAgentParamGUI(int i)
    {
        /// MovementPrefeb
        script.m_Prefebs[i] = EditorGUILayout.ObjectField("MovementAgent_Prefab", script.m_Prefebs[i], typeof(MovementAgent<MovementComponent>), true) as MovementAgent<MovementComponent>;

        /// Each Prefeb's Nums
        EditorGUILayout.Space(2);
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("Agent_Nums", GUILayout.MaxWidth(80));
        script.m_NumsofEachPrefeb[i] = EditorGUILayout.IntField(script.m_NumsofEachPrefeb[i],GUILayout.MaxWidth(50));
        EditorGUILayout.EndHorizontal();

        /// Generate Position Range
        EditorGUILayout.Space(2);
        EditorGUILayout.LabelField("Initialized Position Range");
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField("x_min", GUILayout.MaxWidth(55));
        script.m_MxRange_min[i] = EditorGUILayout.FloatField(script.m_MxRange_min[i], GUILayout.MaxWidth(80));
        EditorGUILayout.LabelField("x_max", GUILayout.MaxWidth(55));
        script.m_MxRange_max[i] = EditorGUILayout.FloatField(script.m_MxRange_max[i], GUILayout.MaxWidth(80));

        EditorGUILayout.EndHorizontal();

        EditorGUILayout.BeginHorizontal();

        EditorGUILayout.LabelField("y_min", GUILayout.MaxWidth(55));
        script.m_MyRange_min[i] = EditorGUILayout.FloatField(script.m_MyRange_min[i], GUILayout.MaxWidth(80));
        EditorGUILayout.LabelField("y_max", GUILayout.MaxWidth(55));
        script.m_MyRange_max[i] = EditorGUILayout.FloatField(script.m_MyRange_max[i], GUILayout.MaxWidth(80));

        EditorGUILayout.EndHorizontal();

        ///// Switch on Behaviors
        //EditorGUILayout.Space(2);
        //EditorGUILayout.LabelField("Switch on Behaviors");
        //script.behavior_Type = (SteeringBehavior.behavior_type)EditorGUILayout.EnumFlagsField("Behavior Types", script.behavior_Type);
    }
    }
}
