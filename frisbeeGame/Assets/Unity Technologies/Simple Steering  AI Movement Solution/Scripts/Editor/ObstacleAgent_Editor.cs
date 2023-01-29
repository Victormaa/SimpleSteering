using UnityEngine;
using UnityEditor;

namespace SimpleSteering {
[CustomEditor(typeof(ObstacleComponent))]
public class ObstacleAgent_Editor : Editor
{
    private ObstacleComponent script;
    private SerializedObject obj;

    private void OnEnable()
    {
        obj = new SerializedObject(target);
        script = target as ObstacleComponent;
    }

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        script.gameObject.transform.localScale = new Vector3(script.m_radius * script.radiusToScaleFactor, 1, script.m_radius * script.radiusToScaleFactor);

    }
    private void OnSceneGUI()
    {
        var tr = script.transform;
        var pos = tr.position;
        // display an orange disc where the object is
        var color = new Color(1, 0.8f, 0.4f, 1);
        Handles.color = color;
        Handles.DrawWireDisc(pos, tr.up, script.m_radius);
        // display object "value" in scene
        GUI.color = color;
        Handles.Label(pos, script.m_radius.ToString("F1"));
    }
    }
}