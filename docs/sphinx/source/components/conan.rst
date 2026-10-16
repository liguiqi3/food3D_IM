Build with Conan
================

The package recipe in ``conanfile.py`` reads its requirements from ``conandata.yml``.
PCL, Eigen, and nlohmann_json are runtime requirements; pybind11 is included when Python
bindings are enabled. GTest is used by the test package, not by the runtime library.

Build and run the consumer check
--------------------------------

From the repository root:

.. code-block:: console

   conan create . -s build_type=Debug --build=missing -c tools.build:jobs=4

``conan create`` builds the package and runs the test package's consumer executable. The
full CTest suite runs only when ``trigger_tests`` is enabled in ``metadata.json``.
``--build=missing`` permits Conan
to build a dependency from source when it cannot find a compatible binary. PCL can take a
long time on the first run for that reason. A package in the cache is reusable only when its
version, options, compiler settings, and dependency graph match the requested profile.

To avoid a local PCL source build, use a remote with a matching prebuilt PCL package or add
one to the local cache before creating this package. Removing ``--build=missing`` instead
makes Conan fail when the binary is absent; it does not provide that binary. Check the
resolved Conan profile and the PCL package ID when a build is unexpectedly repeated.

How the recipe reaches CMake
----------------------------

The recipe creates a CMake toolchain and dependency files, then passes the C and C++ target
lists to ``CMakeLists.txt``. The C++ library links PCL, Eigen, and nlohmann_json. The public
headers and exported CMake targets are installed with the package. Python bindings are built
on the host when ``enable_python_bindings`` is true in ``metadata.json``; cross builds skip
the host Python module.

For a bare-metal profile, the recipe filters dependencies through
``baremetal_white_list``. The PCL measurement path is unavailable there and returns
``kUnsupportedPlatform``. The cross-build archive check verifies target ISA attributes;
it does not make the host measurement algorithm available on the board.
