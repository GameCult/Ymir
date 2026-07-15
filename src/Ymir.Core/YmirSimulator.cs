namespace Ymir.Core;

/// <summary>
/// Compatibility façade for the snapshot-in/snapshot-out Ymir contract.
/// Calls are isolated by design; retained world ownership is not inferred from snapshots.
/// </summary>
public sealed class YmirSimulator
{
    public SimulationStepResult Step(SimulationStepRequest request)
    {
        using var session = new YmirWorldSession();
        return session.Step(request);
    }

    public SoASimulationStepResult Step(SoASimulationStepRequest request)
    {
        using var session = new YmirWorldSession();
        return session.Step(request);
    }
}
