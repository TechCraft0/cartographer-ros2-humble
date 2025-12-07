执行了这个 sudo mv /usr/local/lib/liblua.a /usr/local/lib/liblua.a.bak


colcon build --symlink-install --event-handlers console_direct+

rosdep install --from-paths src --ignore-src -r -y