# TODOs

## Sahit

- [ ] controller should fail if any of the joints hit a joint limit
- [ ] Bash script for starting controller and gripper in tmux session
- [ ] tag a v1.0.0 on github
- [ ] add authentication (lower priority, just add note in README)
- [ ] MinJerkInterpolator bug: `do_min_jerk_` is always false and never set, so the "MinJerkInterpolator" only does linear interpolation. Either remove the flag and always apply min-jerk transform, or add a way to set it.

## Will

- [ ] The bamboo client should fail if it can't connect instead of failing silently and just logging warnings. Also we have try ... except Exception, it'd be nice to explicitly throw known errors if we have them from bamboo. Like there are fallbacks to gripper_state = 0.0 if read fails but I think we should raise that.
- [ ] update README with latest install instructions, pinnochio stuff too

## Not important for now
 
- [x] Seems like gripper still uses JSON instead of msgpack (Sahit did on another PR)

## Done

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