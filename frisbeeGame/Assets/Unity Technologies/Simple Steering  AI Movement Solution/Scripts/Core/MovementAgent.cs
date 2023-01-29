using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class MovementAgent<T> : MonoBehaviour where T : MonoBehaviour
{
    public bool IsPlayerControl = false;
    [SerializeField] private MovementPrm m_movementParameter;
    private Vehicle m_oVehicleSystem = new Vehicle();
    [TagSelector] public string TagFilter = "";
    public void InitializedMovementRoleEntity(int entityType, AgentManager agemtManager, Vector2 pos, int spawnID)
    {
        if (m_oVehicleSystem != null)
            m_oVehicleSystem = null;

        m_oVehicleSystem = new Vehicle(entityType, agemtManager, m_movementParameter, pos, this.gameObject, spawnID);
    }
    public Vehicle GetVehicle(){ return m_oVehicleSystem; }
    public MovementPrm GetMovementPrm() { return m_movementParameter; }
    public virtual void AgentUpdate() { }
    public virtual void AgentStart() { }
}
}



