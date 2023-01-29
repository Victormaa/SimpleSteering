
namespace SimpleSteering { 
public struct Telegram
{
    public int Sender;
    public int Receiver;
    public int Msg;
    public double DispatchTime;

    object ExtraInfo;

    public Telegram(int sender, int receiver, int msg, double dispatchTime, object extrainfo)
    {
        Sender = sender;
        Receiver = receiver;
        Msg = msg;
        DispatchTime = dispatchTime;
        ExtraInfo = extrainfo;
    }
}
}