param(
    [string] $AetheriaRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\Aetheria")).Path
)

$ErrorActionPreference = "Stop"

function Read-Text([string] $relativePath) {
    $path = Join-Path $AetheriaRoot $relativePath
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required Aetheria file is missing: $relativePath"
    }

    Get-Content -Raw -LiteralPath $path
}

function Assert-Contains([string] $text, [string] $needle, [string] $message) {
    if (-not $text.Contains($needle)) {
        throw $message
    }
}

function Assert-NotContains([string] $text, [string] $needle, [string] $message) {
    if ($text.Contains($needle)) {
        throw $message
    }
}

if (-not (Test-Path -LiteralPath $AetheriaRoot)) {
    throw "Aetheria root does not exist: $AetheriaRoot"
}

$shieldManager = Read-Text "Assets\Scripts\Gameplay\ShieldManager.cs"
$hullCollider = Read-Text "Assets\Scripts\Gameplay\HullCollider.cs"
$projectile = Read-Text "Assets\Scripts\Gameplay\Weapons\Projectile.cs"
$bridge = Read-Text "Assets\Scripts\Gameplay\Physics\AetheriaYmirPhysicsBridge.cs"

Assert-NotContains $shieldManager "OnCollisionEnter" "ShieldManager must not decide gameplay from Unity collision callbacks."
Assert-NotContains $shieldManager "TakeHit(" "ShieldManager must not apply damage directly; damage policy is downstream of Ymir contact truth."
Assert-NotContains $hullCollider "OnCollisionEnter" "HullCollider must not mutate entity physics from Unity collision callbacks."
Assert-NotContains $hullCollider "SendHit(" "HullCollider must not expose a Unity-callback hit publisher as damage authority."

Assert-Contains $projectile "AetheriaYmirPhysicsBridge.Instance.TryStepProjectile" "Projectile travel and direct-hit discovery must route through the Ymir bridge."
Assert-Contains $projectile "projectile killed instead of falling back to Unity physics" "Projectile must fail closed when Ymir stepping is unavailable."
Assert-NotContains $projectile "OnCollisionEnter" "Projectile must not use Unity collision callbacks for hit discovery."
Assert-NotContains $projectile "Physics." "Projectile must not use UnityEngine.Physics as hit-discovery authority."

Assert-Contains $bridge "public bool EnableProjectileCutover = true;" "Ymir projectile cutover must be enabled by default."
Assert-Contains $bridge "YmirPhysicsQueries.Step(request)" "The Aetheria bridge must step projectiles through Ymir physics queries."
Assert-Contains $bridge "result.contacts" "The Aetheria bridge must derive projectile hits from Ymir contact events."
Assert-Contains $bridge "TryResolveTargetDaemonHull" "Ymir contact body ids must resolve to Aetheria hulls through the bridge seam."

$weaponAuthorityFiles = Get-ChildItem -LiteralPath (Join-Path $AetheriaRoot "Assets\Scripts\Gameplay\Weapons") -Filter "*.cs" -File
foreach ($file in $weaponAuthorityFiles) {
    $text = Get-Content -Raw -LiteralPath $file.FullName
    if ($text.Contains("Physics.")) {
        throw "Weapon gameplay file still uses UnityEngine.Physics directly: $($file.FullName)"
    }
}

Write-Host "Aetheria/Ymir cutover proof passed for $AetheriaRoot"
