# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-src"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-build"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix/tmp"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix/src/rlimgui-populate-stamp"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix/src"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix/src/rlimgui-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix/src/rlimgui-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/rlimgui-subbuild/rlimgui-populate-prefix/src/rlimgui-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
