# Publishing to PyPI

## Setup (one-time)

1. Create a PyPI account at https://pypi.org/account/register/
2. Get collaborator access to the project (ask an existing maintainer)
3. Create an API token at https://pypi.org/manage/account/token/ (scope it to this project)

Install dev dependencies and configure credentials:

```bash
pip install -e ".[dev]"

export TWINE_USERNAME=__token__
export TWINE_PASSWORD=pypi-xxxx  # your API token
```

## Release

1. Update `version` in `pyproject.toml`

2. Build and upload:

```bash
rm -rf dist/
python -m build
twine check dist/*
twine upload dist/*
```

3. Tag the release:

```bash
git tag v0.x.x
git push origin v0.x.x
```

## Testing Locally

Before uploading, you can verify the build works:

```bash
pip install dist/bamboo_franka_controller-*.whl --force-reinstall
python -c "from bamboo import BambooFrankaClient; print('OK')"
```

## Adding Maintainers

Go to https://pypi.org/manage/project/bamboo-franka-controller/collaboration/ and add their PyPI username. They'll then create their own API token.
