using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class StateMachine<T>
{
    private T m_Owner;

    private State<T> m_CurrentState;
    private State<T> m_PreviousState;
    private State<T> m_GlobalState;

    public StateMachine(T owner)
    {
        m_Owner = owner;
        m_CurrentState = null;
        m_PreviousState = null;
        m_GlobalState = null;
    }

    public void SetCurrentState(State<T> s) => m_CurrentState = s;
    public void SetGlobalState(State<T> s) => m_GlobalState = s;
    public void SetPreviousState(State<T> s) => m_PreviousState = s;

    public void Update()
    {
        if (m_GlobalState != null) m_GlobalState.Execute(m_Owner);
        if (m_CurrentState != null) m_CurrentState.Execute(m_Owner);
    }

    public void ChangeState(State<T> newState)
    {
        if (newState == null)
        {
            Debug.LogError("StateMachine::ChangeState trying to change to a null state");
            return;
        }
        m_PreviousState = m_CurrentState;
        m_CurrentState.Exit(m_Owner);
        m_CurrentState = newState;
        m_CurrentState.Enter(m_Owner);
    }

    public void RevertToPreviousState() => ChangeState(m_PreviousState);

    public State<T> CurrentState() { return m_CurrentState; }
    public State<T> GlobalState() { return m_GlobalState; }
    public State<T> PreviousState() { return m_PreviousState; }
    public bool HandleMessage(Telegram telegram)
    {
        if(m_CurrentState != null && m_CurrentState.OnMessage(m_Owner, telegram))
            return true;

        if (m_GlobalState != null && m_GlobalState.OnMessage(m_Owner, telegram))
            return true;

        return false;
    }
    public bool isInState(State<T> state)
    {
        if (m_CurrentState == state)
            return true;

        return false;
    }
}
}