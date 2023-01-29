using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering
{
    [RequireComponent(typeof(LineRenderer))]
    public class MovementComponent : MovementAgent<MovementComponent>
    {
        /// <summary>
        ///  Dont do Update() Start(),
        /// </summary>

        private LineRenderer LineDrawer;

        public override void AgentUpdate()
        { }
        public override void AgentStart()
        {
            LineDrawer = GetComponent<LineRenderer>();
            LineDrawer.startColor = Color.white;
            LineDrawer.endColor = Color.white;
            LineDrawer.startWidth = 0.05f;
            LineDrawer.endWidth = 0.05f;
            Material whiteDiffuseMat = new Material(Shader.Find("Unlit/Texture"));
            LineDrawer.material = whiteDiffuseMat;
        }
    }
}