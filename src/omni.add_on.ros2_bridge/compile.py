from distutils.core import setup
from distutils.extension import Extension

from Cython.Distutils import build_ext


ext_modules = [
    Extension("_ros2_bridge",
              ["omni/add_on/ros2_bridge/ros2_bridge.py"],
              library_dirs=['/isaac-sim/kit/python/include']),
]

setup(
    name = 'omni.add_on.ros2_bridge',
    cmdclass = {'build_ext': build_ext},
    ext_modules = ext_modules
)
