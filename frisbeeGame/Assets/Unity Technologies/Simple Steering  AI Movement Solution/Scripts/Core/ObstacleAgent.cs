using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering
{
    [RequireComponent(typeof(LineRenderer))]
    public class ObstacleAgent<T> : MonoBehaviour where T : MonoBehaviour
    {
        private LineRenderer LineDrawer;
        public readonly float radiusToScaleFactor = 1.8f;
        private Obstacle m_Obstacle = new Obstacle();
        public float m_radius;
        private void Start()
        {
            InitializeLineRender();
        }

        private void OnEnable()
        {
            InitializeLineRender();
        }

        private void InitializeLineRender()
        {
            LineDrawer = GetComponent<LineRenderer>();
            LineDrawer.startColor = Color.green;
            LineDrawer.endColor = Color.green;
            LineDrawer.startWidth = 0.02f;
            LineDrawer.endWidth = 0.02f;
        }
        public Obstacle GetObstacle()
        {
            return m_Obstacle;
        }
        public void ResetSize(float radius)
        {
            m_Obstacle.SetBRadius(radius);
            m_radius = radius;
        }
        public void InitializedObstacle(int entityType, Vector2 pos, float radius)
        {
            if (m_Obstacle != null)
                m_Obstacle = null;

            m_Obstacle = new Obstacle(entityType, pos, radius, this.gameObject);
        }
    }
}