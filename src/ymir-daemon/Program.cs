using System.Net;
using System.Text;
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
            case "provider":
                WriteJson(ProviderAdvertisement());
                return 0;
            case "operator-state":
                WriteJson(OperatorState());
                return 0;
            case "step-sample":
                WriteJson(new YmirSimulator().Step(SampleRequest()));
                return 0;
            case "save-sample":
                await SaveSampleAsync(ValueAfter(args, "--path") ?? ".ymir/worlds.cc");
                return 0;
            case "step":
                return RunStep(args);
            case "serve":
                await ServeAsync(ParsePort(args, 8877));
                return 0;
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

    private static async Task ServeAsync(int port)
    {
        using var listener = new HttpListener();
        listener.Prefixes.Add($"http://127.0.0.1:{port}/");
        listener.Start();
        Console.WriteLine($"Ymir serving on http://127.0.0.1:{port}/");

        while (true)
        {
            var context = await listener.GetContextAsync();
            _ = Task.Run(() => HandleRequestAsync(context));
        }
    }

    private static async Task HandleRequestAsync(HttpListenerContext context)
    {
        try
        {
            var path = context.Request.Url?.AbsolutePath ?? "/";
            var method = context.Request.HttpMethod;

            if (method == "GET" && path == "/health")
            {
                await WriteResponseAsync(context, new { ok = true, service = "ymir", checkedAt = DateTimeOffset.UtcNow });
                return;
            }

            if (method == "GET" && path == "/provider-advertisement")
            {
                await WriteResponseAsync(context, ProviderAdvertisement());
                return;
            }

            if (method == "GET" && path == "/operator-state")
            {
                await WriteResponseAsync(context, OperatorState());
                return;
            }

            if (method == "GET" && path == "/eve/operator")
            {
                await WriteResponseAsync(context, EveOperatorSurface());
                return;
            }

            if (method == "POST" && path == "/simulate/step")
            {
                using var reader = new StreamReader(context.Request.InputStream, context.Request.ContentEncoding);
                var body = await reader.ReadToEndAsync();
                var request = JsonSerializer.Deserialize<SimulationStepRequest>(body, JsonOptions);
                if (request is null)
                {
                    await WriteErrorAsync(context, 400, "Invalid simulation request.");
                    return;
                }

                await WriteResponseAsync(context, new YmirSimulator().Step(request));
                return;
            }

            await WriteErrorAsync(context, 404, "No such Ymir route.");
        }
        catch (Exception error)
        {
            await WriteErrorAsync(context, 500, error.Message);
        }
    }

    private static object ProviderAdvertisement() => new
    {
        schema = "gamecult.provider_advertisement.v1",
        providerId = "ymir.physics",
        title = "Ymir Physics",
        description = "GameCult physics/world substrate engine built on CultMath.",
        authority = new
        {
            owns = new[]
            {
                "physics simulation truth",
                "body integration",
                "field acceleration",
                "contact generation",
                "collision query semantics",
                "SoA CultCache world-state persistence"
            },
            doesNotOwn = new[]
            {
                "Unity scene truth",
                "rendering",
                "gameplay damage policy",
                "editor authoring truth"
            }
        },
        routes = new
        {
            health = "/health",
            providerAdvertisement = "/provider-advertisement",
            operatorState = "/operator-state",
            operatorSurface = "/eve/operator",
            step = "/simulate/step"
        }
    };

    private static object OperatorState() => new
    {
        schema = "gamecult.ymir.operator_state.v0",
        providerId = "ymir.physics",
        status = "mvp",
        stateOwner = "Ymir.Core step result; JSON is compatibility witness only",
        numericSubstrate = "CultMath",
        batchKernel = $"CultMath.BatchMath lanes={BatchMath.LaneCount}, hardwareAccelerated={BatchMath.IsHardwareAccelerated}",
        persistence = "CultCache MessagePack directory store for gamecult.ymir.world_state.v0",
        updatedAt = DateTimeOffset.UtcNow
    };

    private static object EveOperatorSurface() => new
    {
        schema = "gamecult.eve.surface.v1",
        providerId = "ymir.physics",
        title = "Ymir Physics",
        updatedAt = DateTimeOffset.UtcNow,
        surface = new
        {
            root = new
            {
                type = "panel",
                title = "Ymir Physics",
                children = new object[]
                {
                    new { type = "stat", label = "Status", value = "mvp" },
                    new { type = "stat", label = "Owner", value = "physics simulation truth" },
                    new { type = "stat", label = "Math", value = "CultMath" },
                    new { type = "stat", label = "Batch lanes", value = BatchMath.LaneCount.ToString() },
                    new { type = "stat", label = "Persistence", value = "CultCache SoA world_state.v0" },
                    new { type = "route", label = "Step", value = "POST /simulate/step" }
                }
            }
        }
    };

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

    private static async Task WriteResponseAsync(HttpListenerContext context, object value)
    {
        var json = JsonSerializer.Serialize(value, JsonOptions);
        var bytes = Encoding.UTF8.GetBytes(json);
        context.Response.ContentType = "application/json; charset=utf-8";
        context.Response.ContentLength64 = bytes.Length;
        await context.Response.OutputStream.WriteAsync(bytes);
        context.Response.Close();
    }

    private static Task WriteErrorAsync(HttpListenerContext context, int statusCode, string message)
    {
        context.Response.StatusCode = statusCode;
        return WriteResponseAsync(context, new { ok = false, error = message });
    }

    private static void WriteJson(object value) => Console.WriteLine(JsonSerializer.Serialize(value, JsonOptions));

    private static int ParsePort(string[] args, int defaultPort)
    {
        var raw = ValueAfter(args, "--port");
        return int.TryParse(raw, out var port) ? port : defaultPort;
    }

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
          provider
          operator-state
          step-sample
          save-sample [--path .ymir/worlds.cc]
          step --request <path>
          serve [--port 8877]
        """);
    }
}
