import os
import sys
import carb
import omni.ext
try:
    from .. import _ros2_bridge
except:
    print(">>>> [DEVELOPMENT] import ros2_bridge")
    from .. import ros2_bridge as _ros2_bridge


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ros2bridge = None
        self._extension_path = None
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("omni.isaac.ros_bridge"):
            carb.log_error("ROS 2 Bridge external extension cannot be enabled if ROS Bridge is enabled")
            ext_manager.set_extension_enabled("semu.robotics.ros2_bridge", False)
            return
        
        self._extension_path = ext_manager.get_extension_path(ext_id)
        sys.path.append(os.path.join(self._extension_path, "semu", "robotics", "ros2_bridge", "packages"))

        if os.environ.get("LD_LIBRARY_PATH"):
            os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH") + ":{}/bin".format(self._extension_path)
        else:
            os.environ["LD_LIBRARY_PATH"] = "{}/bin".format(self._extension_path)

        self._ros2bridge = _ros2_bridge.acquire_ros2_bridge_interface(ext_id)

    def on_shutdown(self):
        if self._extension_path is not None:
            sys.path.remove(os.path.join(self._extension_path, "semu", "robotics", "ros2_bridge", "packages"))
            self._extension_path = None
        if self._ros2bridge is not None:
            _ros2_bridge.release_ros2_bridge_interface(self._ros2bridge)
            self._ros2bridge = None
