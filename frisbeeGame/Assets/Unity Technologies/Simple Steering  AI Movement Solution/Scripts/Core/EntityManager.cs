using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class EntityManager 
{
    private Dictionary<int, BaseGameEntity> m_EntityDictionary;
    private static EntityManager instance = null;
    private static System.Object alock = new Object();
    
    private EntityManager() 
    {
        m_EntityDictionary = new Dictionary<int, BaseGameEntity>(); 
    }

    public static EntityManager Instance
    {
        get 
        {
            if(instance != null)
                return instance;
            else
            {
                lock (alock)
                {
                    if(instance == null)
                    {
                        instance = new EntityManager();
                        return instance;
                    }
                    else
                    {
                        return instance;
                    }
                }
            }
        }
        set 
        {
            if(EntityManager.instance == null)
            {
                lock (alock)
                {
                    if (EntityManager.instance == null)
                    {
                        EntityManager.instance = new EntityManager();
                    }
                }
            }
        }
    }

    public void RegisterEntity(BaseGameEntity newEntity)
    {
        this.m_EntityDictionary.Add(newEntity.GetID(), newEntity);
    }

    public BaseGameEntity GetEntityFromID(int id)
    {
        BaseGameEntity gameEntity;

        gameEntity = m_EntityDictionary[id];

        return gameEntity;
    }

    public void RemoveEntity(BaseGameEntity entity)
    {
        this.m_EntityDictionary.Remove(entity.GetID());
    }
}
}