namespace Ymir.Box3D;

public static class Box3DRuntime
{
    public const string NativeBuildId =
        "box3d-8441b4a06d6d09dcfb0b0f704df4d847d1437b92-abi5-f32";

    public static string ValidatedBuildId
    {
        get
        {
            Box3DSession.EnsureCompatibleNativeLibrary();
            return NativeBuildId;
        }
    }
}
