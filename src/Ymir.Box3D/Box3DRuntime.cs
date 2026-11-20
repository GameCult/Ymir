namespace Ymir.Box3D;

public static class Box3DRuntime
{
    public const string NativeBuildId =
        "box3d-8441b4a-gc-6cfeeff-abi5-f32";

    public static string ValidatedBuildId
    {
        get
        {
            Box3DSession.EnsureCompatibleNativeLibrary();
            return NativeBuildId;
        }
    }
}
