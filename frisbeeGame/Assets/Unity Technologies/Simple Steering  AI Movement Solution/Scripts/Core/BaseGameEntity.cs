using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public abstract class BaseGameEntity
{
    private int m_ID;
    private static int m_NextValidID;
    private bool m_bTag;
    private int m_EntityType;
    #region base parametre
    protected int type;
    protected Vector2 m_pos;
    //the length of this object's bounding radius
    protected float m_dBoundingRadius;

    protected float m_scale;

    private bool Highlight = false;
    private bool ClosestHL = false;
    #endregion

    private void SetID(int val)
    {
        if (val >= m_NextValidID)
        {
            m_ID = val;
            m_NextValidID += 1;
        }
        else
        {
            Debug.LogError("ID passed in is used.");
        }
    }
    protected BaseGameEntity()
    { 
        //m_ID = NextValidID();
        //m_dBoundingRadius = 0;
        //m_pos = new Vector2();
        //m_scale = 1;
        //m_EntityType = 0;
        //m_bTag = false;
    }
    //protected BaseGameEntity() { }
    protected BaseGameEntity(int entity_type) 
    {
        m_ID = NextValidID();
        m_dBoundingRadius = 0;
        m_pos = new Vector2();
        m_scale = 1;
        m_EntityType = entity_type;
        m_bTag = false;
    }
    protected BaseGameEntity(int entity_type, int ForcedID) 
    {
        m_ID = ForcedID;
        m_dBoundingRadius = 0;
        m_pos = new Vector2();
        m_scale = 1;
        m_EntityType = entity_type;
        m_bTag = false;
    }
    protected BaseGameEntity(int entity_type, Vector2 pos, float radius, float scale)
    {
        m_ID = NextValidID(); 
        m_dBoundingRadius = radius;
        m_pos = pos;
        //m_scale = scale;
        m_scale = 1;
        m_EntityType = entity_type;
        m_bTag = false;
    }
    private int NextValidID() { return m_NextValidID++; }
    public int ID() {return m_ID;}
    public virtual bool HandleMessage(Telegram msg) { return false; }
    public Vector2 Pos() { return m_pos; }
    public int GetID() { return m_ID; }
    public void UnTag() { m_bTag = false; }
    public void Tag() { m_bTag = true; }
    public bool IsTag() { return m_bTag; }
    public float BRadius() {return m_dBoundingRadius;}
    public void SetBRadius(float r) { m_dBoundingRadius = r; }
    public void HighLightDebug(bool on) { Highlight = on; }
    public void ClosestHighLightDebug(bool on) { ClosestHL = on; }
    public bool IsHightlight() { return Highlight; }
    public bool IsClosestHightlight() { return ClosestHL; }
    public void SetPos(Vector2 newPos) { m_pos = newPos; }
    public void SetPos(Vector3 newPos) { m_pos = new Vector2(newPos.x, newPos.z); }
    public int GetEntityType()
    {
        return m_EntityType;
    }
}
}