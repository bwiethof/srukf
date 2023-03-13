from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout


class UkfLib(ConanFile):
    name = "ukf_lib"
    version = "0.0.1"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def layout(self):
        cmake_layout(self)

    def build_requirements(self):
        self.test_requires("gtest/cci.20210126")

    def requirements(self):
        self.requires("eigen/3.4.0")

# -DCMAKE_TOOLCHAIN_FILE=build/Debug/conan_toolchain.cmake
