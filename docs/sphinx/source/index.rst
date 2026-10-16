food3d_im
===================

``food3d_im`` estimates the volume of food above an oven tray from two kinds of
point cloud: one or more empty-tray frames and a frame containing food. The camera and tray
must stay in the same coordinate frame between captures. The result is a height integral in
cubic centimetres, accompanied by coverage and hole-filling diagnostics.

Start with :doc:`measurement <export>` for a complete C++ example. Read
:doc:`building <components/conan>` if you need to build the package. Doxygen provides the
API reference for ``FoodVolumeMeasurer``, ``MeasurementConfig``, and ``VolumeEstimate``.
After a local documentation build, open ``docs/doxygen/build/docs.html`` to choose a language.

What the estimate means
-----------------------

The library fits a plane to the empty tray, stores the observed tray height in each grid cell,
and compares food points with that measured surface. It never substitutes an ideal flat tray
for cells where the empty capture has no data. Small enclosed gaps in the food surface may be
filled; their volume is reported separately from the volume supported by measured points.

Check ``VolumeEstimate.status`` before using any numeric result. On success,
``volume_cm3`` is the sum of ``raw_volume_cm3`` and ``interpolated_volume_cm3``. A large
``interpolated_cells`` count or a low ``coverage_ratio`` is a reason to inspect the input
clouds and the intermediate PCD files, not a reason to treat the estimate as ground truth.

Where to go next
----------------

.. toctree::
   :maxdepth: 2

   Measure a frame <export>
   Build with Conan <components/conan>
   CMake targets <components/cmake>
   Project metadata <components/meta>

The package targets C++17. The PCL measurement pipeline runs on host platforms; a bare-metal
build reports ``kUnsupportedPlatform`` for this operation.
