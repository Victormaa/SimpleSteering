using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering
{
    public class SmootherForVector2
    {
        private Vector2[] m_History;
        private int m_iNextUpdateSlot;
        private Vector2 m_ZeroValue;

        public SmootherForVector2(int sampleSize, Vector2 ZeroValue)
        {
            m_History = new Vector2[sampleSize];
            m_ZeroValue = ZeroValue;
            m_iNextUpdateSlot = 0;
        }

        public Vector2 SmoothFacingUpdate(Vector2 MostRecentValue)
        {
            Vector2 result = m_ZeroValue;

            m_History[m_iNextUpdateSlot++] = MostRecentValue;

            if (m_iNextUpdateSlot == m_History.Length) m_iNextUpdateSlot = 0;

            foreach (var t in m_History)
            {
                result += t;
            }

            return result / (float)m_History.Length;
        }

    }
}