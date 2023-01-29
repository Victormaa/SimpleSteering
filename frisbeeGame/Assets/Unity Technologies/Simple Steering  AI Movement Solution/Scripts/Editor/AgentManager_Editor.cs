using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace SimpleSteering { 
[CustomEditor(typeof(AgentManager))]
public class AgentManager_Editor : Editor
{
    private AgentManager script;
    private SerializedObject obj;

    private void OnEnable()
    {
        obj = new SerializedObject(target);
        script = target as AgentManager;
    }
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        obj.Update();
        EditorGUI.BeginChangeCheck();
        if (script.IsUseWorldPrm)
        {
            script.m_worldPrm = EditorGUILayout.ObjectField("WorldPrm", script.m_worldPrm, typeof(GameWorldPrm), true) as GameWorldPrm;
        }
        else
        {
            script.m_worldPrm = null;
        }
        EditorGUI.EndChangeCheck();
    }
}
}