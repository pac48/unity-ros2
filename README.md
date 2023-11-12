### Install from a Git URL
https://github.com/pac48/unity-ros2.git?path=/com.pac48.unity.unity-ros2

## local build
# build and install custom messages
```bash
cd ~/Downloads/unity-ros2/ROSInterfaceCustomMessages
mkdir build
cd build
cmake ..
make install
```

# generate c++ messages
```bash
python3 ~/Downloads/unity-ros2/ROSInterface/script/generate_messages.py cpp --header_dir ~/Downloads/unity-ros2/ROSInterface/include/generated --src_dir ~/Downloads/unity-ros2/ROSInterface/src/generated
```

# generate c# messages
```bash
python3  ~/Downloads/unity-ros2/ROSInterface/script/generate_messages.py c# --csharp_dir ~/Downloads/unity-ros2/com.pac48.unity.unity-ros2/Editor/ROSInterface
```

# build libROSInterface.so 
```bash
cd ~/Downloads/unity-ros2/ROSInterface
mkdir build
cd build
cmake ..
make install
```
