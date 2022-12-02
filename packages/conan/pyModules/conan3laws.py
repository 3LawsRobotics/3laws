import os

from conan.tools.cmake import CMake, cmake_layout
from conans.errors import ConanException

from conan import ConanFile, tools


def bool_to_cmake(v: bool) -> str:
    if v:
        return "ON"
    else:
        return "OFF"


class ConanRecipe(ConanFile):

    # Optional metadata
    license = "Exclusive property of 3Laws Robotics Inc."
    author = "Thomas Gurriet"
    url = "https://github.com/3LawsRobotics/3laws"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "scalar_type": ["double", "float"],
        "fPIC": [True, False],
        "verbose_build": [True, False],
        "export_compile_commands": [True, False],
    }
    default_options = {
        "scalar_type": "double",
        "fPIC": True,
        "verbose_build": False,
        "verbose_build": False,
        "export_compile_commands": True,
    }

    generators = ["CMakeDeps", "CMakeToolchain"]

    exports_sources = "CMakeLists.txt", "include/*", "src/*", "config/*"

    requires = ["3laws_library-public/0.4.0"]

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def validate(self):
        tools.build.check_min_cppstd(self, 11)

    def layout(self):
        cmake_layout(self)

    def package_id(self):
        del self.info.options.verbose_build
        del self.info.options.export_compile_commands

    def build(self, variables: dict = {}):
        # Pass CMake module path
        CMAKE_MODULE_PATH = os.getenv('CMAKE_MODULE_PATH')
        if not CMAKE_MODULE_PATH:
            self.output.warn("CMAKE_MODULE_PATH environment variable not set")
        else:
            variables["CMAKE_MODULE_PATH"] = CMAKE_MODULE_PATH

        variables["SCALAR_TYPE_3LAWS"] = self.options.scalar_type
        variables["CMAKE_EXPORT_COMPILE_COMMANDS"] = bool_to_cmake(
            self.options.export_compile_commands
        )

        # Pass proper compiler flag for arm based cross compilation
        if str(self.settings.arch).startswith("arm"):
            variables["CMAKE_NO_SYSTEM_FROM_IMPORTED"] = bool_to_cmake(True)
            variables["CREATE_SHARED_TARGETS"] = bool_to_cmake(False)
            variables["CMAKE_AR"] = "arm-none-eabi-gcc-ar"

        cmake = CMake(self)
        cmake.configure(variables=variables)
        cli_args = []

        if self.options.verbose_build:
            cli_args.append("--verbose")

        cmake.build(cli_args=cli_args)

    def package(self):
        cmake = CMake(self)
        cmake.install()
