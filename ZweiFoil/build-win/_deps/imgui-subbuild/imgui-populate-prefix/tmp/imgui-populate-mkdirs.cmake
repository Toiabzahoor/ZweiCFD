# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-src"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-build"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix/tmp"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix/src"
  "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspaces/ZweiFoil/ZweiFoil/build-win/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
