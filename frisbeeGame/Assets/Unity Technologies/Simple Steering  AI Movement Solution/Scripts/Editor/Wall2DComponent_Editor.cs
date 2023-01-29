using UnityEditor;
using UnityEngine;

namespace SimpleSteering {
[CustomEditor(typeof(Wall2DComponent))]
public class Wall2DComponent_Editor : Editor
{
    private Wall2DComponent script;
    private SerializedObject obj;

    private void OnEnable()
    {
        obj = new SerializedObject(target);
        script = target as Wall2DComponent;
    }

    private void OnSceneGUI()
    {
        var tr = script.transform;

        var point1 = tr.Find("point1").gameObject.transform.position;
        var point2 = tr.Find("point2").gameObject.transform.position;

        var pos = tr.position;
        // display an orange disc where the object is
        var color = new Color(1, 0.8f, 0.4f, 1);

        // display object "value" in scene
        GUI.color = color;
        var length = Vector3.Distance(point1, point2);
        Handles.Label(pos, length.ToString("F1"));
    }
}
}