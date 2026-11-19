using Ymir.Box3D;

namespace Ymir.Box3D.Soa.Tests;

public sealed class Box3DSoaConcurrentCreateTests
{
    /// <summary>
    /// Box3D's world pool is a process-global array scanned without a lock; without the native
    /// create/destroy lock, concurrent creation claims one slot twice and aborts the host.
    /// Each thread creates, steps and disposes many worlds while the others do the same.
    /// </summary>
    [Fact]
    public void Worlds_created_stepped_and_disposed_from_many_threads_do_not_crash()
    {
        const int threads = 8;
        const int rounds = 200;
        using var start = new ManualResetEventSlim(false);
        var failures = new List<Exception>();

        var workers = Enumerable.Range(0, threads).Select(t => new Thread(() =>
        {
            try
            {
                start.Wait();
                for (var r = 0; r < rounds; r++)
                {
                    using var world = new Box3DSoaWorld(capacity: 8);
                    world.AddStaticBox(1, (0f, -0.5f, 0f), (10f, 0.5f, 10f));
                    world.AddSphere(2, (0f, 2f, 0f), radius: 0.5f, density: 1f, velocity: (0f, 0f, 0f));
                    for (var s = 0; s < 3; s++) world.Step(1f / 60f);
                    if (world.View.Count != 2) throw new InvalidOperationException("lane count");
                }
            }
            catch (Exception e)
            {
                lock (failures) failures.Add(e);
            }
        })).ToArray();

        foreach (var w in workers) w.Start();
        start.Set();
        foreach (var w in workers) w.Join();

        Assert.Empty(failures);
    }
}
