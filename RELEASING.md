# Publishing to PyPI

## Setup (one-time)

### Accounts and Tokens

1. Create a PyPI account at https://pypi.org/account/register/
2. Create a TestPyPI account at https://test.pypi.org/account/register/
3. Get collaborator access to the project on PyPI (ask an existing maintainer)
4. Create API tokens:
   - PyPI: https://pypi.org/manage/account/token/
   - TestPyPI: https://test.pypi.org/manage/account/token/

### Configure Credentials

Create `~/.pypirc` with your tokens:

```ini
[pypi]
username = __token__
password = pypi-xxxx

[testpypi]
repository = https://test.pypi.org/legacy/
username = __token__
password = pypi-xxxx
```

Protect the file: `chmod 600 ~/.pypirc`

### Install Dev Dependencies

```bash
pip install -e ".[dev]"
```

## Release

1. Update `version` in `pyproject.toml`

2. Build:

   ```bash
   rm -rf dist/
   python -m build
   twine check dist/*
   ```

3. Test locally:

   ```bash
   pip install dist/bamboo_franka_client-*.whl --force-reinstall
   python -c "from bamboo import BambooFrankaClient; print('OK')"
   ```

4. Upload to TestPyPI (recommended for major releases):

   ```bash
   twine upload --repository testpypi dist/*

   # Test install (--extra-index-url pulls deps from real PyPI)
   pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ bamboo-franka-client
   ```

5. Upload to PyPI:

   ```bash
   twine upload dist/*
   ```

6. Tag the release:

   ```bash
   git tag v0.x.x
   git push origin v0.x.x
   ```

## Adding Maintainers

Add their PyPI username at https://pypi.org/manage/project/bamboo-franka-client/collaboration/
