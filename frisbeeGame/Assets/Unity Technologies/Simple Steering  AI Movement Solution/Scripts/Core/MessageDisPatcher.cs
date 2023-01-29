using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class MessageDisPatcher 
{
    private SortedList<double, Telegram> teleQue = new SortedList<double, Telegram>();
    
    private void Discharge(BaseGameEntity reveiver, Telegram telegram) { }
    private MessageDisPatcher() { }
    private static MessageDisPatcher instance = null;
    private static System.Object alock = new Object();
    public static MessageDisPatcher Instance { 
        get { return instance; }
        set { 
            if (MessageDisPatcher.instance == null) // there we should set up a lock make sure no other code initilized it
            {
                lock (alock)
                {
                    if (MessageDisPatcher.instance == null)
                        MessageDisPatcher.instance = new MessageDisPatcher();
                }
            }
        }
    }
    public void DispatchMessage(double delay,
                                int sender,
                                int receiver,
                                int msg,
                                object ExtraInfo)
    {
        BaseGameEntity _receiver = EntityManager.Instance.GetEntityFromID(receiver);
        Telegram tele = new Telegram(sender, receiver, msg, delay, ExtraInfo);
        if (delay <= 0)
            Discharge(_receiver, tele);
        else
        {
            double currentTime = Time.frameCount;
            tele.DispatchTime = currentTime + delay;
            teleQue.Add(tele.DispatchTime, tele); // this needs to be sorted as the delay time ;
        }
    }
    public void DispatchDelayedMessages()
    {
        double currentTime = Time.frameCount;
        while (teleQue[0].DispatchTime < currentTime &&
            teleQue[0].DispatchTime > 0)
        {
            Telegram telegram = teleQue[0];
            BaseGameEntity _receiver = EntityManager.Instance.GetEntityFromID(telegram.Receiver);
            Discharge(_receiver, telegram);
            teleQue.RemoveAt(0);
        }
    }
    
}
}