## Building form source

### Linux

```bash
cd src/semu.robotics.ros2_bridge
bash compile_extension.bash
```
 
## Removing old compiled files

Get a fresh clone of the repository and follow the next steps

```bash
# remove compiled files _ros2_bridge.cpython-37m-x86_64-linux-gnu.so
git filter-repo --invert-paths --path exts/semu.robotics.ros2_bridge/semu/robotics/ros2_bridge/_ros2_bridge.cpython-37m-x86_64-linux-gnu.so

# add origin
git remote add origin git@github.com:Toni-SM/semu.robotics.ros2_bridge.git

# push changes
git push origin --force --all
git push origin --force --tags
```

## Packaging the extension

```bash
cd src/semu.robotics.ros2_bridge
bash package_extension.bash
```