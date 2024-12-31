# Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

from conan import ConanFile
from conan.tools.cmake import cmake_layout, CMake, CMakeDeps, CMakeToolchain

class ptceeRecipe(ConanFile):
    name = "ptcee"
    package_type = "library"

    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "CMakeLists.txt", "cmake/*", "include/*", "src/*", "test/*"
    
    options = {
        "shared": [True, False],
    }

    default_options = {
        "shared": True,
        "gtsam/*:rot3_expmap": True,
        "gtsam/*:use_quaternions": True,
        "gtsam/*:shared": True,
        "gtsam/*:install_cppunitlite": False,
        "gtsam/*:with_TBB": False,
    }

    def set_version(self):
        from conan.tools.files import load
        import os
        import re
        self.version = re.search(r"project\([\S\s]+ VERSION (\d+(\.\d+){1,3})",
                                 load(self, os.path.join(self.recipe_folder, "CMakeLists.txt"))).group(1).strip()
    
    def build_requirements(self):
        self.tool_requires("cmake/[>=3.23.5]")

    def requirements(self):
        self.requires("eigen/3.4.0", transitive_headers=True)
        self.requires("gtsam/4.2", transitive_headers=True, transitive_libs=True)
        self.requires("sophus/1.22.10", transitive_headers=True)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = [self.name]
