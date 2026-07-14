# GameCult.Ymir.Box3D

This is Ymir's private managed/native boundary for pinned Box3D v0.1.0. It is
installed transitively by Ymir; products and clients should reference Ymir's
typed session contracts instead of this package.

The package includes the compiled native facade under `runtimes/<rid>/native`.
Package consumers do not need CMake, a C compiler, or the Box3D submodule.
Missing or ABI-incompatible native assets fail closed when Ymir opens a physics
session.

Source builds of Ymir use CMake to compile the pinned submodule. Box3D is MIT
licensed; its license is included in the package.
