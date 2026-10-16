Project metadata
================

``metadata.json`` is read by the Conan recipe, the root CMake project, and the documentation
builder. Change a setting there when you intend to change all of those consumers; edit
``MeasurementConfig`` or a measurement JSON file to change an algorithm parameter.

Settings that affect this library
---------------------------------

.. list-table::
   :header-rows: 1
   :widths: 27 73

   * - Key
     - Effect
   * - ``build_cppstd``
     - Sets the C++ standard; the current value is 17. Module support starts at 23.
   * - ``is_shared``
     - Chooses static or shared library output, subject to the Windows fallback.
   * - ``enable_python_bindings``
     - Builds the host pybind11 module when true.
   * - ``dependencies.cpp``
     - Names CMake targets linked to the C++ library. Package versions are in
       ``conandata.yml``.
   * - ``doc_languages`` / ``doc_versions``
     - Select the Doxygen/Sphinx build variants. These are documentation labels,
       not the package's semantic version.
   * - ``workflow_triggers.docs``
     - Enables the documentation CI trigger. A local ``docs/build.py`` run does not
       depend on this switch.

The dependency map is a build target map, not a list of versions. For example, the ``PCL``
entry names ``PCL::PCL`` while ``conandata.yml`` selects ``pcl/1.14.1``. Keep the two in
sync when changing dependencies.

Documentation sources
---------------------

Doxygen reads the paths and suffixes selected by ``doc_doxygen_folders`` and
``doc_doxygen_suffix``. Sphinx reads pages from ``docs/sphinx/source`` and Chinese
translations from ``docs/sphinx/locales/zh_CN``. Run ``python docs/build.py`` from the
repository root to generate both; the script writes its output under ``docs/doxygen/build``
and ``docs/sphinx/build``.
