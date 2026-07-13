using System.Text.Json;
using CultMath;
using Ymir.Core;

namespace Ymir.Daemon;

internal static class Program
{
    private static readonly JsonSerializerOptions JsonOptions = new(JsonSerializerDefaults.Web)
    {
        WriteIndented = true
    };

    public static async Task<int> Main(string[] args)
    {
        var command = args.FirstOrDefault() ?? "help";

        switch (command)
        {
            case "publish-service":
                await PublishServiceAsync(ValueAfter(args, "--path") ?? ".ymir/service.cc");
                return 0;
            case "step-sample":
                WriteJson(new YmirSimulator().Step(SampleRequest()));
                return 0;
            case "save-sample":
                await SaveSampleAsync(ValueAfter(args, "--path") ?? ".ymir/worlds.cc");
                return 0;
            case "step":
                return RunStep(args);
            default:
                PrintHelp();
                return command is "help" or "-h" or "--help" ? 0 : 1;
        }
    }

    private static int RunStep(string[] args)
    {
        var requestPath = ValueAfter(args, "--request");
        if (string.IsNullOrWhiteSpace(requestPath))
        {
            Console.Error.WriteLine("step requires --request <path>.");
            return 1;
        }

        var request = JsonSerializer.Deserialize<SimulationStepRequest>(
            File.ReadAllText(requestPath),
            JsonOptions);

        if (request is null)
        {
            Console.Error.WriteLine("Could not deserialize simulation step request.");
            return 1;
        }

        WriteJson(new YmirSimulator().Step(request));
        return 0;
    }

    private static SimulationStepRequest SampleRequest() => new(
        0.1f,
        new YmirWorld(
            0.0f,
            new[]
            {
                new PhysicsBody("projectile", new Vec2(-2.0f, 0.0f), new Vec2(4.0f, 0.0f), 0.1f, 1.0f, Restitution: 0.0f),
                new PhysicsBody("target", new Vec2(0.5f, 0.0f), Vec2.Zero, 0.5f, 1.0f, IsStatic: true)
            },
            new[]
            {
                new RadialField("gravity-well", new Vec2(0.0f, 1.0f), 2.0f, 8.0f)
            }));

    private static async Task SaveSampleAsync(string path)
    {
        var result = new YmirSimulator().Step(new SoASimulationStepRequest(
            SampleRequest().DeltaTime,
            YmirSoAWorld.FromWorld(SampleRequest().World)));
        await YmirCultCacheStore.SaveWorldAsync(path, "sample", result.World);
        WriteJson(new
        {
            ok = true,
            path,
            record = "ymir:world:sample",
            schema = "gamecult.ymir.world_state.v0",
            bodyCount = result.World.BodyCount,
            fieldCount = result.World.FieldCount
        });
    }

    private static async Task PublishServiceAsync(string path)
    {
        await YmirServicePublicationStore.RegenerateDerivedStoreAsync(path, DateTimeOffset.UtcNow);
        WriteJson(new
        {
            ok = true,
            path,
            records = new[]
            {
                "ymir:provider:ymir.physics",
                "ymir:operator:ymir.physics",
                "surface:ymir.physics.operator"
            }
        });
    }

    private static void WriteJson(object value) => Console.WriteLine(JsonSerializer.Serialize(value, JsonOptions));

    private static string? ValueAfter(string[] args, string key)
    {
        var index = Array.IndexOf(args, key);
        return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
    }

    private static void PrintHelp()
    {
        Console.WriteLine("""
        Ymir physics daemon

        Commands:
          publish-service [--path .ymir/service.cc]  Regenerate the derived service publication.
          step-sample
          save-sample [--path .ymir/worlds.cc]
          step --request <path>
        """);
    }
}
