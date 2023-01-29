using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class State<T>
{
    public virtual void Enter(T t) { }
    public virtual void Execute(T t) { }
    public virtual void Exit(T t) { }
    public virtual bool OnMessage(T entity, Telegram telegram) { return false; }
}
}