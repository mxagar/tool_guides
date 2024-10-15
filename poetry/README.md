# Poetry Python Package Manager

[Python Poetry](https://python-poetry.org/) = Python dependency and package manager.

This guide shows how to use it.

Source links:

- [Dependency Management With Python Poetry](https://realpython.com/dependency-management-python-poetry/)
- [Poetry: Finally an all-in-one tool to manage Python packages](https://medium.com/analytics-vidhya/poetry-finally-an-all-in-one-tool-to-manage-python-packages-3c4d2538e828)
- [Poetry: Basic Usage](https://python-poetry.org/docs/basic-usage/)

Other links with pre-requisite information:

- [Using Python's pip to Manage Your Projects' Dependencies](https://realpython.com/what-is-pip/)
- [Python Virtual Environments: A Primer](https://realpython.com/python-virtual-environments-a-primer/)
- [Python Modules and Packages – An Introduction](https://realpython.com/python-modules-packages/)

Table of contents:

- [Poetry Python Package Manager](#poetry-python-package-manager)
  - [1. Introduction and Setup](#1-introduction-and-setup)
    - [Pre-Requisite: Virtual Environments](#pre-requisite-virtual-environments)
      - [Which one should we use?](#which-one-should-we-use)
      - [Usage of venv](#usage-of-venv)
    - [Installation of Poetry](#installation-of-poetry)
  - [2. Creating a New Python Project with Poetry](#2-creating-a-new-python-project-with-poetry)
  - [3. Working With Poetry](#3-working-with-poetry)
    - [Use Poetry’s Virtual Environment](#use-poetrys-virtual-environment)
    - [Declare Your Dependencies](#declare-your-dependencies)
    - [Install a Package With Poetry](#install-a-package-with-poetry)
  - [4. Handle Dependencies Correctly](#4-handle-dependencies-correctly)
    - [Pin Dependencies in poetry.lock](#pin-dependencies-in-poetrylock)
    - [Update Dependencies](#update-dependencies)
  - [5. Add Poetry to an Existing Project](#5-add-poetry-to-an-existing-project)
    - [Add pyproject.toml to a Scripts Folder](#add-pyprojecttoml-to-a-scripts-folder)
    - [Create requirements.txt From poetry.lock](#create-requirementstxt-from-poetrylock)
  - [6. Distribution: Publishing the Package](#6-distribution-publishing-the-package)
  - [7. Most Common Commands](#7-most-common-commands)
  - [8. Quick Setup Summary (Windows)](#8-quick-setup-summary-windows)

## 1. Introduction and Setup

Poetry works with `pyproject.toml` files, which will be the standard for defining build requirements.

### Pre-Requisite: Virtual Environments

A virtual environment is an independent python installation in which we can install the package version we need for a given project. Two popular virtual environment managers are **conda** and **venv**. However, note that `conda` does two things: environment and package management.

One alternative to `conda` is using the two Python native tools:

- `venv` for environment managing
- `pip` for package managing

However, note that `pip` handles only python packages, while `conda` is language agnostic. In fact, `conda` emerged because of the need to install data science packages such as `numpy` which are developed in another language than Python.

We can use `pip` within `conda`, but we need to add `pip` to the creation of our environment:

`conda create --name env_name pip`

Otherwise, any `pip install` we do applies to the global Anaconda Python installation, not the environment we're in, even if we do `conda install pip`!

#### Which one should we use?

- Conda works well with data science projects, because packagees for that are well curated and handled; however, it is more difficult for general package development.
- `pip` and `venv` are for general python development.
- It seems `venv` cannot handle different Python versions that easily. It uses the default Python distribution installed on the computer or in the environment where Python is called.

#### Usage of venv

```bash
# conda, for reference
# the environment my_env is not installed in the local folder
# but in the Anaconda3 envs folder
conda create --name my_env pip
source activate my_env
conda install numpy
pip istall package # installed in my_env BECAUSE pip added when creation of my_env 

# venv
python -m venv .venv # .venv folder is created locally and a python kernel installed there
# Commonly, the names venv, env, .venv are used, but we can use any name!
source .venv/bin/activate # activate, Mac/Linux
.\.venv\Scripts\activate # activate, Windows
python -m pip install numpy # pip installs locally always in .venv
# if we remove the my_env folder the virtual env is removed
# nothing bad happens happens

# Install dependendencies from requirements.txt
python -m pip install -r requirements.txt

# Deactivate env: Win/Mac/Linux
deactivate

# Delete a virtual environment
rm -fr .venv # Mac/Linux
rm .venv # Windows

# Get dependencies
python -m pip freeze # Windows, display
python -m pip freeze > requirements.txt # Windows, to file

# Python and pip versions
which python
python --version
python -m pip list
```

### Installation of Poetry

Source: [Installation](https://python-poetry.org/docs/#installing-with-the-official-installer).

```bash
# Windows, PW
(Invoke-WebRequest -Uri https://install.python-poetry.org -UseBasicParsing).Content | py -
# By default, installed to
# %APPDATA%\pypoetry

# Linux/Mac/WSL
curl -sSL https://install.python-poetry.org | python3 -
# By default, installed to
# ~/Library/Application Support/pypoetry
# ~/.local/share/pypoetry
```

Then, we need to add poetry to `PATH`:

```bash
# Linux
$HOME/.local/bin
# Windows
%APPDATA%\Python\Scripts
C:\Users\Msagardi\AppData\Roaming\Python\Scripts

# Check
poetry --version
# Update if required
poetry self update
```

## 2. Creating a New Python Project with Poetry

Create a new package in the local folder `rp-poetry`:

```bash
# This creates an empty project template
# If the projet already exists and we want to add poetry to it
# we should use `poetry init`; see section 5
poetry new rp-poetry
cd rp-poetry

# Alternative creation commands
poetry new rp-poetry --name realpoetry # control package name
poetry new --src rp-poetry # place source code in a src/ parent folder
```

Default contents (note that `-` is replaced by `_`):

```
│   pyproject.toml
│   README.md
│
├───rp_poetry
│       __init__.py
│
└───tests
        __init__.py
```

All files will be probably empty, except the `pyproject.toml` configuration file, which is the new PEP 518 standard for python package definition.

[TOML](https://toml.io/en/) is a config file format for humans, similar to YAML, but ideally easier; the official site contains useful examples.

Contents of `pyproject.toml`:

```toml
# pyproject.toml

[tool.poetry]
name = "rp-poetry"
version = "0.1.0"
description = ""
authors = ["Mikel Sagardia <mxagar@gmail.com>"]
readme = "README.md"
packages = [{include = "rp_poetry"}]

[tool.poetry.dependencies]
python = "^3.9"

[tool.poetry.dev-dependencies]
pytest = "^5.2"

[build-system]
requires = ["poetry-core>=1.0.0"]
build-backend = "poetry.core.masonry.api"
```

Each section in the TOML file is called a *table*:

- If a table is tool-specific, it must be pre-fixed by `tool`
- So, we can create *subtables*: `[tool.poetry]`, `[tool.pytest.ini_options]`, etc.

Importat tables (of poetry and in general) and their fields (they should always appear):

- `[tool.poetry]`: name, version, description, authors, etc.
- `[tool.poetry.dependencies]`: package dependencies
- `[tool.poetry.dev-dependencies]`: development dependencies, which can be different from the package dependencies...
- `[build-system]`: information for building; it can be used by poetry or other tools, following [PEP 517](https://peps.python.org/pep-0517/#source-trees):
  - `requires`: a list of dependencies that are required to build the package, making this key mandatory
  - `build-backend`: the Python object used to perform the build process

The idea is that we modify/extend this template `pyproject.toml` file with more dependencies, etc.

## 3. Working With Poetry

### Use Poetry’s Virtual Environment

Poetry doesn't create an environment automatically, we need to create one manually

```bash
# ... in rp-poetry
# Check envs
poetry env list
# Nothing is returned

# We can manually create an env as follows
# Here, we’re using the same Python version that we used to install Poetry
# The env is stored in 
# C:\Users\<username>\AppData\Local\pypoetry\Cache
# ~/Library/Caches/pypoetry
# ~/.cache/pypoetry
poetry env use python # Unix
poetry env use py # Windows

# Now, there is an env
poetry env list
# rp-poetry-Gwlh5_RN-py3.9

# Get info of our env
poetry env info
poetry env info --path # where is the env

# After we set our environment, we can run an interactive shell with
poetry run python
# >>> ...

# Or we can run our project scripts on the environment
poetry run python my_script.py

# Run pytest
poetry run pytest

# Enter the shell in the environment!
poetry shell
```

### Declare Your Dependencies

We make sure we have in the TOML file (add if we don't - it's an example):

```toml
[tool.poetry.dependencies]
python = "^3.9"

[tool.poetry.dev-dependencies]
pytest = "^5.2"
```

The most common [versioning constraint operators](https://python-poetry.org/docs/dependency-specification/#version-constraints) are:

        Caret ^
            ^1.2.3      >=1.2.3 <2.0.0
        Tilde ~
            ~1.2.3      >=1.2.3 <1.3.0
        Wildcard *
            *           >=0.0.0
            1.2.*   	>=1.2.0 <1.3.0
        Exact ==
            ==1.2.3
        Inequality
            != 1.2.3
        At @: only via poetry add; equivalent to ==, but we can add operators
            poetry add django@^4.0.0

And then, create a `tests/test_rp_poetry.py` file with this content:

```python
def test_idle():
    assert 1 == 1
```

Then, we install the package and run the tests:

```bash
# Dependencies are checked from pyproject.toml,
# all versions RESOLVED, and installed
# Besides the requirements, the project itself is also installed
# This way, we can import rp_project
poetry install

# If we want to generate bytecode
poetry install --compile

# Run pytest
poetry run pytest
```

### Install a Package With Poetry

We can also manually install packages; if done so, the `pyproject.toml` file is automatically updated. Additionally, whenever we run `poetry add`, the `poetry.lock` file is created/updated, which keeps track of all packages and the exact versions we're using in the project.

```bash
# Equivalent to pip install requests
# Also, pyproject.toml and poetry.lock are updated
poetry add requests

# Install packages only in the dev environment: 
# Typical candidates: Pytest, Black, Sphinx, Pylint, Flake8, mypy
poetry add --dev black
poetry add -D black # same as --dev

# To install all non-dev packages
poetry install --no-dev

# We can have a group of *optional* packages
# When we add/install them, we assign them the optional label
poetry add --group optional <package>

# To install all packages, also the optional
poetry install --with optional
```

## 4. Handle Dependencies Correctly

### Pin Dependencies in poetry.lock

Whenever we run `poetry add`, the `poetry.lock` file is created/updated, which keeps track of all packages and the exact versions we're using in the project: `poetry.lock` saves all the found versions and the resolving process. If `poetry.lock` and `pyproject.toml` are not synced (aka. pinned), we get an error, thus, we should run `poetry lock` before any `poetry install`:

```bash
# Sync poetry.lock & pyproject.toml
# Also known as pinning: parse pyproject.toml for dependencies and update pyproject.toml
poetry lock

# Install dependencies & package
poetry install
```

While the version requirement in the `pyproject.toml` file can be loose, Poetry locks the versions we're actually using in the `poetry.lock` file. Therefore, we should always try to sync them and commit both to git!

### Update Dependencies

```bash
# Show all dependencies as a tree
poetry show --tree

# Show current and latest version of packages
poetry show --latest

# Look for new dependency versions
# and if there are some compliant with the restrictions
# install them
poetry update

# Dry run: see what would be installed/updated
poetry update --dry-run

# Update only specific packages
poetry update requests beautifulsoup4

# Latest version added/installed
poetry add pytest@latest --dev

# Remove a package
poetry remove requests
```

## 5. Add Poetry to an Existing Project

### Add pyproject.toml to a Scripts Folder

With `poetry new <project-name>` we create an empty project template; but in case we have a project already, we can use poetry with it as follows:

```bash
cd project_folder
ls # *.py

# Initialize
poetry init
# An interactive session is started to create the pyproject.toml file

# Add the requirements.txt file contents
# if any
poetry add `cat requirements.txt`

# When finished, we can run our project scripts
# on the associated environment;
# the first time, the env will be created first
poetry run python my_script.py
```

### Create requirements.txt From poetry.lock

We can export the requirements in `poerty.lock` as follows:

```bash
poetry export --output requirements.txt
# Export also the dev dependencies
poetry export --output requirements.txt --dev
```

## 6. Distribution: Publishing the Package

For building a package that can be distributed, we use `poetry build` and the `poetry publish`.

```bash
# Build the source and wheels archives
# At the momento, only pure python wheels are supported
poetry build

# Publish a build package to a remote repository
# BUT username, credetials & repo must be defined
poetry publish
# Options:
# --repository (-r): The repository to register the package to (default: pypi). Should match a repository name set by the config command.
# --username (-u): The username to access the repository.
# --password (-p): The password to access the repository.
# --cert: Certificate authority to access the repository.
# --client-cert: Client certificate to access the repository.
# --build: Build the package before publishing.
# --dry-run: Perform all actions except upload the package.
# --skip-existing: Ignore errors from files already existing in the repository.
```

More information:

- [Poetry: Finally an all-in-one tool to manage Python packages](https://medium.com/analytics-vidhya/poetry-finally-an-all-in-one-tool-to-manage-python-packages-3c4d2538e828).
- [Poetry Documentation: Publishable Repositories](https://python-poetry.org/docs/repositories/#publishable-repositories)

## 7. Most Common Commands

[Commands Reference](https://python-poetry.org/docs/cli/).

```bash
$ poetry --version	# Show the version of your Poetry installation.
$ poetry new	# Create a new Poetry project.
$ poetry init	# Add Poetry to an existing project.
$ poetry run	# Execute the given command with Poetry.
$ poetry add	# Add a package to pyproject.toml and install it.
$ poetry update	# Update your project’s dependencies.
$ poetry install	# Install the dependencies.
$ poetry show	# List installed packages.
$ poetry lock	# Pin the latest version of your dependencies into poetry.lock.
$ poetry lock --no-update	# Refresh the poetry.lock file without updating any dependency version.
$ poetry check	# Validate pyproject.toml.
$ poetry config --list	# Show the Poetry configuration.
$ poetry env list	# List the virtual environments of your project.
$ poetry export	# Export poetry.lock to other formats.
```

## 8. Quick Setup Summary (Windows)

Here's a quick recipe to start using Poetry:

1. Install desired Python executable(s): [https://www.python.org/downloads/windows/](https://www.python.org/downloads/windows/) 
     - Add location of python executables to environment variables (`Path` or `PATH`).
     - On Windws, check the different Python versions with the Python launcher `py`.

2. Install poetry; e.g., via Powershell command:
    ```
    (Invoke-WebRequest -Uri https://install.python-poetry.org -UseBasicParsing).Content | py -
    ```
    - Add Location of python executables to environment variables (`Path` or `PATH`)

3. Then, open new Powershell/Shell (no Conda) and

    ```bash 
    # --- Go to desired folder
    cd .../my_project

    # --- Create pyproject.toml
    # See example below

    # --- Initialize Poetry (Windows)
    py --list # list all available Python versions, select one, e.g., 3.11
    py -3.11 -c "import sys; print(sys.executable)" # get Python executable path: PYTHON_PATH
    poetry env use PYTHON_PATH # replace with Python executable path; environment will be created
    poetry env list # our environment should be there
    poetry shell # activate environment, i.e., venv with desired Python executable linked

    # --- Install packages; poetry.lock should be created
    poetry install --no-root

    # --- Test env works: 
    poetry run python # import a package, check python/package version, etc.

    # --- If you re-open your project later
    cd .../my_project
    poetry shell # activate environment
    # or...
    poetry run python .../my_script.py
    # or...
    poetry run jupyter-lab
    # or ...
    code . # and select environment

    # --- Install new packages, if required
    poetry add requests # pyproject.toml and poetry.lock are updated; alternatively add them to pyproject.toml
    poetry lock # pin = parse pyproject.toml for dependencies and update pyproject.toml
    poetry install --no-root
    ```

A very simple `pyproject.toml`:

```bash
[tool.poetry]
name = "my-basic-project"
version = "0.1.0"
description = "A basic project using Poetry"
authors = ["Your Name <your.email@example.com>"]
python = "^3.11"

[tool.poetry.dependencies]
python = "^3.11"
numpy = "^1.26.3"

[tool.poetry.group.dev.dependencies]
pytest = "^7.0"

[build-system]
requires = ["poetry-core>=1.0.0"]
build-backend = "poetry.core.masonry.api"
```