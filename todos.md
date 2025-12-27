# TODOs

## Sahit

- [ ] tag a v1.0.0 on github (after this PR is merged in)

## Will
 
- [ ] move gripper logic to bamboo/third_party
- [ ] The bamboo client should fail if it can't connect instead of failing silently and just logging warnings. Also we have try ... except Exception, it'd be nice to explicitly throw known errors if we have them from bamboo. Like there are fallbacks to gripper_state = 0.0 if read fails but I think we should raise that.
- [ ] Package client differently for pypi (and have script that builds and pushes that)? Or separate controller into bamboo_controller or something, IDK. We don't really need C++ files for the client build
- [ ] update README with latest install instructions, pinnochio stuff too
- [ ] pyproject.toml: I feel pre-commit can go inside dev dependencies. Also like project urls are off. Are we using black or ruff too?


## Done

- [x] Seems like gripper still uses JSON instead of msgpack
- [x] add authentication (lower priority, just add note in README)
- [x] Bash script for starting controller and gripper in tmux session 
- [x] controller should fail if any of the joints hit a joint limit (#12)
- [x] Fix example scripts to do package level imports
- [x] Can/should we merge InstallBamboo and InstallPackage?
- [x] InstallBamboo: if conda environment already exists then should warn (to avoid overwrite or breaking people's envs)
- [x] InstallPackage: exit on failure? Have more clear input interface for entering version rather than just empty new line (from UPenn feedback)
- [x] README still has grpc things. Can mention another way to find version of libfranka by just searching through their existing installs? (e.g. locate libfranka.so)
- [x] Claude still has protobuf/grpc mentions lmao: https://github.com/chsahit/bamboo/blob/main/bamboo/CMakeLists.txt#L34
- [x] In bamboo/__init__.py can we load the version from the importlib.metadata that's defined in pyproject.toml so when we upgrade versions there's only one place to do it
- [x] Default gripper port to 5559 to match our other docs? https://github.com/chsahit/bamboo/blob/main/bamboo/gripper_server.py#L19
