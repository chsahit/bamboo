# TODOs

## Sahit

- [ ] tag a v1.0.0 on github
- [ ] MinJerkInterpolator bug: `do_min_jerk_` is always false and never set, so the "MinJerkInterpolator" only does linear interpolation. Either remove the flag and always apply min-jerk transform, or add a way to set it.
- [ ] Remove unused kp/kd fields from TimedWaypoint message - they're sent from client but never used by controller (controller uses fixed gains from JointImpedanceController). Remove from bamboo_messages.h and client.py
- [ ] Controller sends dq, tau_J, time_sec in RobotState but client never uses them (only uses q and O_T_EE). Consider removing these fields or document why they're included.
- [ ] Gripper state returns is_grasped and is_moving but client only uses width. Consider if these fields should be exposed to users or removed.

## Will

- [ ] The bamboo client should fail if it can't connect instead of failing silently and just logging warnings. Also we have try ... except Exception, it'd be nice to explicitly throw known errors if we have them from bamboo. Like there are fallbacks to gripper_state = 0.0 if read fails but I think we should raise that.
- [ ] update README with latest install instructions, pinnochio stuff too

## Not important for now
 
- [x] Seems like gripper still uses JSON instead of msgpack (Sahit did on another PR)

## Done

- [x] add authentication (lower priority, just add note in README)
- [x] controller should fail if any of the joints hit a joint limit
- [x] Bash script for starting controller and gripper in tmux session
- [x] Naming consistency - refactored all C++ code to use q_adjective pattern (q_current, q_goal, dq_current, v_start, v_goal)
- [x] Package client differently for pypi - separated controller/ and bamboo/ directories, only bamboo/ gets packaged
- [x] move gripper logic to bamboo/third_party - moved to controller/third_party/
- [x] pyproject.toml: pre-commit dependencies, project urls, using ruff not black
- [x] Fix example scripts to do package level imports
- [x] Can/should we merge InstallBamboo and InstallPackage?
- [x] InstallBamboo: if conda environment already exists then should warn (to avoid overwrite or breaking people's envs)
- [x] InstallPackage: exit on failure? Have more clear input interface for entering version rather than just empty new line (from UPenn feedback)
- [x] README still has grpc things. Can mention another way to find version of libfranka by just searching through their existing installs? (e.g. locate libfranka.so)
- [x] Claude still has protobuf/grpc mentions lmao: https://github.com/chsahit/bamboo/blob/main/bamboo/CMakeLists.txt#L34
- [x] In bamboo/__init__.py can we load the version from the importlib.metadata that's defined in pyproject.toml so when we upgrade versions there's only one place to do it
- [x] Default gripper port to 5559 to match our other docs? https://github.com/chsahit/bamboo/blob/main/bamboo/gripper_server.py#L19
