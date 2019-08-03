.. _chapter-installation:

============
Installation
============

Getting the source code
=======================
.. _section-source:

If you want the latest version, you can clone the git repository

.. code-block:: bash

       git clone https://gitlab.com/ugv_stereo/sslam_resuse.git -b devel

.. _section-dependencies:

Dependencies
============

  .. NOTE ::

    SSLAM package requires ROS environment and a **fully C++11-compliant**
    compiler. Our code has been fully tested under Ubuntu 16.04, ROS Kinetic.

1. Ceres Solver
---------------

Follow `Ceres Installation
<http://ceres-solver.org/installation.html>`_.

2. OpenCV 3.4
---------

Follow `Opencv Installation
<https://zhanghanduo.github.io/post/new_system>`_.

3. Cereal Serialization
-----------------------

It is used to serialize / deserialize map points swiftly and lightly.
Download it from `github repository <https://github.com/USCiLab/cereal.git>`_.

  .. NOTE ::
    Under Ubuntu 18.04, you have to edit CMakeLists.txt and change line 4 from ``OFF`` to ``ON`` to avoid compiling error.

.. code-block:: bash

        mkdir build && cd build
        cmake ..
        make -j8
        sudo make install

4. Obstacle_msg package


5. Cubicle_detect package


6. Build SSLAM package
----------------------
Clone the repository and catkin build

.. code-block:: bash

        catkin build sslam

.. _section-using-ceres:

Using Ceres with CMake
======================

In order to use Ceres in client code with CMake using `find_package()
<http://www.cmake.org/cmake/help/v3.2/command/find_package.html>`_
then either:

#. Ceres must have been installed with ``make install``.  If the
    install location is non-standard (i.e. is not in CMake's default
    search paths) then it will not be detected by default, see:
    :ref:`section-local-installations`.

    Note that if you are using a non-standard install location you
    should consider exporting Ceres instead, as this will not require
    any extra information to be provided in client code for Ceres to
    be detected.

#. Or Ceres' build directory must have been exported by enabling the
    ``EXPORT_BUILD_DIR`` option when Ceres was configured.


As an example of how to use Ceres, to compile `examples/helloworld.cc
<https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/helloworld.cc>`_
in a separate standalone project, the following CMakeList.txt can be
used:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.5)

    project(helloworld)

    find_package(Ceres REQUIRED)

    # helloworld
    add_executable(helloworld helloworld.cc)
    target_link_libraries(helloworld ${CERES_LIBRARIES})

Irrespective of whether Ceres was installed or exported, if multiple
versions are detected, set: ``Ceres_DIR`` to control which is used.
If Ceres was installed ``Ceres_DIR`` should be the path to the
directory containing the installed ``CeresConfig.cmake`` file
(e.g. ``/usr/local/share/Ceres``).  If Ceres was exported, then
``Ceres_DIR`` should be the path to the exported Ceres build
directory.

  .. NOTE ::

     You do not need to call include_directories(${CERES_INCLUDE_DIRS})
     as the exported Ceres CMake target already contains the definitions
     of its public include directories which will be automatically
     included by CMake when compiling a target that links against Ceres.

Specify Ceres components
-------------------------------------

You can specify particular Ceres components that you require (in order
for Ceres to be reported as found) when invoking
``find_package(Ceres)``.  This allows you to specify, for example,
that you require a version of Ceres built with SuiteSparse support.
By definition, if you do not specify any components when calling
``find_package(Ceres)`` (the default) any version of Ceres detected
will be reported as found, irrespective of which components it was
built with.

The Ceres components which can be specified are:

#. ``LAPACK``: Ceres built using LAPACK (``LAPACK=ON``).

#. ``SuiteSparse``: Ceres built with SuiteSparse (``SUITESPARSE=ON``).

#. ``CXSparse``: Ceres built with CXSparse (``CXSPARSE=ON``).

#. ``AccelerateSparse``: Ceres built with Apple's Accelerate sparse solvers (``ACCELERATESPARSE=ON``).

#. ``EigenSparse``: Ceres built with Eigen's sparse Cholesky factorization
   (``EIGENSPARSE=ON``).

#. ``SparseLinearAlgebraLibrary``: Ceres built with *at least one* sparse linear
   algebra library.  This is equivalent to ``SuiteSparse`` **OR** ``CXSparse``
   **OR** ``AccelerateSparse``  **OR** ``EigenSparse``.

#. ``SchurSpecializations``: Ceres built with Schur specializations
   (``SCHUR_SPECIALIZATIONS=ON``).

#. ``OpenMP``: Ceres built with OpenMP (``CERES_THREADING_MODEL=OPENMP``).

#. ``Multithreading``: Ceres built with *a* multithreading library.
   This is equivalent to (``CERES_THREAD != NO_THREADS``).

#. ``C++11``: Ceres built with C++11.

To specify one/multiple Ceres components use the ``COMPONENTS`` argument to
`find_package()
<http://www.cmake.org/cmake/help/v3.2/command/find_package.html>`_ like so:

.. code-block:: cmake

    # Find a version of Ceres compiled with SuiteSparse & EigenSparse support.
    #
    # NOTE: This will report Ceres as **not** found if the detected version of
    #            Ceres was not compiled with both SuiteSparse & EigenSparse.
    #            Remember, if you have multiple versions of Ceres installed, you
    #            can use Ceres_DIR to specify which should be used.
    find_package(Ceres REQUIRED COMPONENTS SuiteSparse EigenSparse)


Specify Ceres version
---------------------

Additionally, when CMake has found Ceres it can optionally check the package
version, if it has been specified in the `find_package()
<http://www.cmake.org/cmake/help/v3.2/command/find_package.html>`_
call.  For example:

.. code-block:: cmake

    find_package(Ceres 1.2.3 REQUIRED)

.. _section-local-installations:

Local installations
-------------------

If Ceres was installed in a non-standard path by specifying
``-DCMAKE_INSTALL_PREFIX="/some/where/local"``, then the user should
add the **PATHS** option to the ``find_package()`` command, e.g.,

.. code-block:: cmake

   find_package(Ceres REQUIRED PATHS "/some/where/local/")

Note that this can be used to have multiple versions of Ceres
installed.  However, particularly if you have only a single version of
Ceres which you want to use but do not wish to install to a system
location, you should consider exporting Ceres using the
``EXPORT_BUILD_DIR`` option instead of a local install, as exported
versions of Ceres will be automatically detected by CMake,
irrespective of their location.

Understanding the CMake Package System
----------------------------------------

Although a full tutorial on CMake is outside the scope of this guide,
here we cover some of the most common CMake misunderstandings that
crop up when using Ceres.  For more detailed CMake usage, the
following references are very useful:

- The `official CMake tutorial <http://www.cmake.org/cmake-tutorial/>`_

   Provides a tour of the core features of CMake.

- `ProjectConfig tutorial
  <http://www.cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file>`_
  and the `cmake-packages documentation
  <http://www.cmake.org/cmake/help/git-master/manual/cmake-packages.7.html>`_

   Cover how to write a ``ProjectConfig.cmake`` file, discussed below,
   for your own project when installing or exporting it using CMake.
   It also covers how these processes in conjunction with
   ``find_package()`` are actually handled by CMake.  The
   `ProjectConfig tutorial
   <http://www.cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file>`_
   is the older style, currently used by Ceres for compatibility with
   older versions of CMake.

  .. NOTE :: **Targets in CMake.**

    All libraries and executables built using CMake are represented as
    *targets* created using `add_library()
    <http://www.cmake.org/cmake/help/v3.2/command/add_library.html>`_
    and `add_executable()
    <http://www.cmake.org/cmake/help/v3.2/command/add_executable.html>`_.
    Targets encapsulate the rules and dependencies (which can be other
    targets) required to build or link against an object.  This allows
    CMake to implicitly manage dependency chains.  Thus it is
    sufficient to tell CMake that a library target: ``B`` depends on a
    previously declared library target ``A``, and CMake will
    understand that this means that ``B`` also depends on all of the
    public dependencies of ``A``.

When a project like Ceres is installed using CMake, or its build
directory is exported into the local CMake package registry (see
:ref:`section-install-vs-export`), in addition to the public headers
and compiled libraries, a set of CMake-specific project configuration
files are also installed to: ``<INSTALL_ROOT>/share/Ceres`` (if Ceres
is installed), or created in the build directory (if Ceres' build
directory is exported).  When `find_package
<http://www.cmake.org/cmake/help/v3.2/command/find_package.html>`_ is
invoked, CMake checks various standard install locations (including
``/usr/local`` on Linux & UNIX systems), and the local CMake package
registry for CMake configuration files for the project to be found
(i.e. Ceres in the case of ``find_package(Ceres)``).  Specifically it
looks for:

- ``<PROJECT_NAME>Config.cmake`` (or
  ``<lower_case_project_name>-config.cmake``)

   Which is written by the developers of the project, and is
   configured with the selected options and installed locations when
   the project is built and defines the CMake variables:
   ``<PROJECT_NAME>_INCLUDE_DIRS`` & ``<PROJECT_NAME>_LIBRARIES``
   which are used by the caller to import the project.

The ``<PROJECT_NAME>Config.cmake`` typically includes a second file
installed to the same location:

- ``<PROJECT_NAME>Targets.cmake``

   Which is autogenerated by CMake as part of the install process and defines
   **imported targets** for the project in the caller's CMake scope.

An **imported target** contains the same information about a library
as a CMake target that was declared locally in the current CMake
project using ``add_library()``.  However, imported targets refer to
objects that have already been built by a different CMake project.
Principally, an imported target contains the location of the compiled
object and all of its public dependencies required to link against it.
Any locally declared target can depend on an imported target, and
CMake will manage the dependency chain, just as if the imported target
had been declared locally by the current project.

Crucially, just like any locally declared CMake target, an imported target is
identified by its **name** when adding it as a dependency to another target.

Thus, if in a project using Ceres you had the following in your CMakeLists.txt:

.. code-block:: cmake

    find_package(Ceres REQUIRED)
    message("CERES_LIBRARIES = ${CERES_LIBRARIES}")

You would see the output: ``CERES_LIBRARIES = ceres``.  **However**,
here ``ceres`` is an **imported target** created when
``CeresTargets.cmake`` was read as part of ``find_package(Ceres
REQUIRED)``.  It does **not** refer (directly) to the compiled Ceres
library: ``libceres.a/so/dylib/lib``.  This distinction is important,
as depending on the options selected when it was built, Ceres can have
public link dependencies which are encapsulated in the imported target
and automatically added to the link step when Ceres is added as a
dependency of another target by CMake.  In this case, linking only
against ``libceres.a/so/dylib/lib`` without these other public
dependencies would result in a linker error.

Note that this description applies both to projects that are
**installed** using CMake, and to those whose **build directory is
exported** using `export()
<http://www.cmake.org/cmake/help/v3.2/command/export.html>`_ (instead
of `install()
<http://www.cmake.org/cmake/help/v3.2/command/install.html>`_).  Ceres
supports both installation and export of its build directory if the
``EXPORT_BUILD_DIR`` option is enabled, see
:ref:`section-customizing`.

.. _section-install-vs-export:

Installing a project with CMake vs Exporting its build directory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When a project is **installed**, the compiled libraries and headers
are copied from the source & build directory to the install location,
and it is these copied files that are used by any client code.  When a
project's build directory is **exported**, instead of copying the
compiled libraries and headers, CMake creates an entry for the project
in the `user's local CMake package registry
<http://www.cmake.org/cmake/help/v3.2/manual/cmake-packages.7.html#user-package-registry>`_,
``<USER_HOME>/.cmake/packages`` on Linux & OS X, which contains the
path to the project's build directory which will be checked by CMake
during a call to ``find_package()``.  The effect of which is that any
client code uses the compiled libraries and headers in the build
directory directly, **thus not requiring the project to be installed
to be used**.

Installing / Exporting a project that uses Ceres
--------------------------------------------------

As described in `Understanding the CMake Package System`_, the contents of
the ``CERES_LIBRARIES`` variable is the **name** of an imported target which
represents Ceres.  If you are installing / exporting your *own* project which
*uses* Ceres, it is important to understand that:

**Imported targets are not (re)exported when a project which imported them is
exported**.

Thus, when a project ``Foo`` which uses Ceres is exported, its list of
dependencies as seen by another project ``Bar`` which imports ``Foo``
via: ``find_package(Foo REQUIRED)`` will contain: ``ceres``.  However,
the definition of ``ceres`` as an imported target is **not
(re)exported** when Foo is exported.  Hence, without any additional
steps, when processing ``Bar``, ``ceres`` will not be defined as an
imported target.  Thus, when processing ``Bar``, CMake will assume
that ``ceres`` refers only to: ``libceres.a/so/dylib/lib`` (the
compiled Ceres library) directly if it is on the current list of
search paths.  In which case, no CMake errors will occur, but ``Bar``
will not link properly, as it does not have the required public link
dependencies of Ceres, which are stored in the imported target
definition.

The solution to this is for ``Foo`` (i.e., the project that uses
Ceres) to invoke ``find_package(Ceres)`` in ``FooConfig.cmake``, thus
``ceres`` will be defined as an imported target when CMake processes
``Bar``.  An example of the required modifications to
``FooConfig.cmake`` are show below:

.. code-block:: cmake

    # Importing Ceres in FooConfig.cmake using CMake 2.8.x style.
    #
    # When configure_file() is used to generate FooConfig.cmake from
    # FooConfig.cmake.in, @Ceres_DIR@ will be replaced with the current
    # value of Ceres_DIR being used by Foo.  This should be passed as a hint
    # when invoking find_package(Ceres) to ensure that the same install of
    # Ceres is used as was used to build Foo.
    set(CERES_DIR_HINTS @Ceres_DIR@)

    # Forward the QUIET / REQUIRED options.
    if (Foo_FIND_QUIETLY)
       find_package(Ceres QUIET HINTS ${CERES_DIR_HINTS})
    elseif (Foo_FIND_REQUIRED)
       find_package(Ceres REQUIRED HINTS ${CERES_DIR_HINTS})
    else ()
       find_package(Ceres HINTS ${CERES_DIR_HINTS})
    endif()

.. code-block:: cmake

    # Importing Ceres in FooConfig.cmake using CMake 3.x style.
    #
    # In CMake v3.x, the find_dependency() macro exists to forward the REQUIRED
    # / QUIET parameters to find_package() when searching for dependencies.
    #
    # Note that find_dependency() does not take a path hint, so if Ceres was
    # installed in a non-standard location, that location must be added to
    # CMake's search list before this call.
    include(CMakeFindDependencyMacro)
    find_dependency(Ceres)
