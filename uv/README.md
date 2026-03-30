# uv

[uv](https://docs.astral.sh/uv/) is an extremely fast Python package and project manager written in Rust, created by [Astral](https://astral.sh/) (the team behind [Ruff](https://docs.astral.sh/ruff/)). It is designed to be a single tool that replaces pip, pip-tools, pipx, pyenv, virtualenv, and even Poetry — with speeds 10–100x faster than pip.

## Table of Contents

1. [Introduction](#1-introduction)
2. [Installation](#2-installation)
3. [Creating a New Python Project](#3-creating-a-new-python-project)
4. [Python Version Management](#4-python-version-management)
5. [Working with Virtual Environments](#5-working-with-virtual-environments)
6. [Declaring and Installing Dependencies](#6-declaring-and-installing-dependencies)
7. [Lock File and Pinning](#7-lock-file-and-pinning)
8. [Adding uv to an Existing Project](#8-adding-uv-to-an-existing-project)
9. [Building and Publishing](#9-building-and-publishing)
10. [Running Tools (uvx / uv tool)](#10-running-tools-uvx--uv-tool)
11. [Most Common Commands](#11-most-common-commands)
12. [Quick Setup Summary (Windows)](#12-quick-setup-summary-windows)
13. [Comparison: Poetry vs uv](#13-comparison-poetry-vs-uv)

---

## 1. Introduction

### What is a Virtual Environment?

A virtual environment is an isolated Python installation that keeps your project's dependencies separate from other projects and from the system Python. This prevents version conflicts and ensures reproducibility.

uv manages virtual environments automatically — you rarely need to create or activate them manually.

### What Does uv Replace?

| Old tool | uv equivalent |
|---|---|
| `pip` | `uv pip` |
| `pip-tools` | `uv pip compile` / `uv lock` |
| `pipx` | `uvx` / `uv tool` |
| `pyenv` | `uv python` |
| `virtualenv` / `venv` | `uv venv` (auto-managed) |
| `poetry` / `hatch` | `uv` (projects) |

uv supports the standard `pyproject.toml` (PEP 517/518/621) so it is compatible with existing Python packaging conventions.

---

## 2. Installation

### Windows (PowerShell)

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### macOS / Linux / WSL

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Alternative: via pip

```bash
pip install uv
```

### Verify installation

```bash
uv --version
```

uv is a single binary — no additional PATH configuration is usually needed.

---

## 3. Creating a New Python Project

### Scaffold a new project

```bash
uv init rp-uv
cd rp-uv
```

Or initialise inside an existing (empty) directory:

```bash
mkdir rp-uv && cd rp-uv
uv init
```

### Generated project structure

```
rp-uv/
├── .gitignore
├── .python-version       # pins the Python version for this project
├── README.md
├── main.py               # entry-point script
└── pyproject.toml        # project metadata and dependency config
```

> Unlike Poetry (`poetry new`), uv does **not** create a package sub-directory by default. It creates a flat `main.py`. You can add a package directory manually or use `uv init --lib` for a library layout.

### pyproject.toml anatomy

```toml
[project]
name = "rp-uv"
version = "0.1.0"
description = ""
authors = [{ name = "Your Name", email = "you@example.com" }]
readme = "README.md"
requires-python = ">=3.9"
dependencies = []           # production dependencies (PEP 621)

[project.optional-dependencies]
# optional extras (e.g. for published libraries)
plot = ["matplotlib>=3.7"]

[dependency-groups]
# development-only dependencies (PEP 735, not published to PyPI)
dev = ["pytest>=7", "black>=23"]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.uv]
default-groups = ["dev"]    # groups included by default in uv sync
```

---

## 4. Python Version Management

uv can download and manage Python versions itself — no pyenv required.

### Install a Python version

```bash
uv python install 3.12
uv python install 3.9 3.10 3.11   # install multiple at once
uv python install ">=3.8,<3.10"   # install matching a range
```

### List available and installed versions

```bash
uv python list
uv python list --all-versions      # show all patch versions
```

### Pin a version to the current project

```bash
uv python pin 3.12
```

This writes (or updates) `.python-version` in the current directory. uv respects this file automatically.

### Find a Python executable

```bash
uv python find
uv python find ">=3.11"
```

### Upgrade installed versions

```bash
uv python upgrade 3.12    # upgrade to latest 3.12.x
uv python upgrade          # upgrade all installed versions
```

---

## 5. Working with Virtual Environments

### Automatic management (recommended)

uv creates and manages a `.venv/` directory inside your project automatically. You almost never need to create it manually.

Running any of the following commands will create the `.venv` if it does not exist:

```bash
uv sync          # install all dependencies (reads pyproject.toml + uv.lock)
uv run main.py   # run a script inside the project environment
```

### Create a virtual environment explicitly

```bash
uv venv                       # creates .venv/ using the pinned Python version
uv venv --python 3.11         # specify a version
uv venv --python 3.11.6       # specify an exact patch version
uv venv myenv                 # custom directory name
```

### Activate the environment (optional)

You do **not** need to activate the environment to run commands — use `uv run` instead.

```bash
# macOS / Linux / WSL
source .venv/bin/activate

# Windows (CMD)
.venv\Scripts\activate.bat

# Windows (PowerShell)
.venv\Scripts\Activate.ps1
```

### Run commands inside the environment

```bash
uv run python          # open a Python REPL
uv run main.py         # run a script
uv run pytest          # run tests
uv run -- flask run -p 3000   # pass flags to the command
```

`uv run` is the uv equivalent of `poetry run` and does not require shell activation.

---

## 6. Declaring and Installing Dependencies

### Version constraint operators (PEP 508)

| Operator | Meaning | Example |
|---|---|---|
| `>=` | Greater than or equal | `requests>=2.28` |
| `<=` | Less than or equal | `numpy<=1.26` |
| `~=` | Compatible release | `~=1.4` → `>=1.4,<2` |
| `==` | Exact version | `beautifulsoup4==4.10.0` |
| `!=` | Exclude version | `django!=4.0` |
| `==X.*` | Wildcard | `pytest==7.*` |
| `,` | Combine constraints | `requests>=2.28,<3` |

### Add a production dependency

```bash
uv add requests
uv add "requests>=2.28,<3"
uv add "beautifulsoup4==4.10.0"
uv add git+https://github.com/psf/requests   # from a git repo
```

`uv add` automatically:
1. Adds the package to `[project].dependencies` in `pyproject.toml`
2. Updates `uv.lock`
3. Installs the package into `.venv/`

### Add development dependencies

Dev dependencies go under `[dependency-groups]` and are **not** included when the package is published to PyPI.

```bash
uv add --dev pytest black
uv add --group lint ruff mypy   # custom named group
```

Resulting `pyproject.toml`:

```toml
[dependency-groups]
dev = ["pytest>=7.4", "black>=23.3"]
lint = ["ruff>=0.1", "mypy>=1.0"]
```

Groups can include other groups:

```toml
[dependency-groups]
dev = [
    "pytest>=7.4",
    { include-group = "lint" },   # inherit lint group
]
lint = ["ruff>=0.1"]
```

### Add optional (extra) dependencies

Extras reduce the default dependency tree for published libraries (e.g. `pip install mypkg[plot]`).

```bash
uv add --optional plot matplotlib
```

Resulting `pyproject.toml`:

```toml
[project.optional-dependencies]
plot = ["matplotlib>=3.7"]
```

### Install all dependencies

```bash
uv sync                  # install production + default groups (dev)
uv sync --no-dev         # install production only
uv sync --group lint     # include a specific extra group
uv sync --all-groups     # include every group
```

### Remove a dependency

```bash
uv remove requests
uv remove --dev black
```

---

## 7. Lock File and Pinning

### uv.lock

`uv.lock` is a cross-platform lock file that pins the exact version (and hashes) of every dependency — including transitive ones. It is generated automatically and should be committed to version control.

```
# Example uv.lock entry (simplified)
[[package]]
name = "requests"
version = "2.31.0"
source = { registry = "https://pypi.org/simple" }
dependencies = [
    { name = "certifi" },
    { name = "charset-normalizer" },
    { name = "idna" },
    { name = "urllib3" },
]
```

### Regenerate the lock file without installing

```bash
uv lock
```

### Upgrade a single package

```bash
uv lock --upgrade-package requests
```

### Upgrade all packages

```bash
uv lock --upgrade
```

### Show the dependency tree

```bash
uv tree
uv tree --package requests   # tree rooted at a specific package
```

---

## 8. Adding uv to an Existing Project

### Option A — initialise in place

If the project has no `pyproject.toml` yet:

```bash
cd my-existing-project
uv init
```

### Option B — import from requirements.txt

```bash
uv add -r requirements.txt
```

This reads `requirements.txt` and adds each package to `[project].dependencies`.

### Option C — adopt an existing pyproject.toml

If `pyproject.toml` already exists (e.g. from pip-tools or setuptools), just run:

```bash
uv sync
```

uv will generate `uv.lock` and create `.venv/` automatically.

### Export back to requirements.txt (for compatibility)

```bash
uv pip compile pyproject.toml -o requirements.txt
uv pip compile pyproject.toml --extra dev -o requirements-dev.txt
```

---

## 9. Building and Publishing

### Build a distribution

```bash
uv build
```

Creates `dist/` containing a wheel (`.whl`) and a source distribution (`.tar.gz`).

```bash
uv build --no-sources   # verify the build without custom index sources
```

### Bump the version

```bash
uv version              # show current version
uv version 1.2.0        # set exact version
uv version --bump minor  # 0.1.0 → 0.2.0
uv version --bump patch  # 0.1.0 → 0.1.1
uv version --bump major  # 0.1.0 → 1.0.0
```

### Publish to PyPI

```bash
uv publish
```

Credentials (PyPI no longer accepts username+password — use a token):

```bash
# via environment variable (recommended for CI)
UV_PUBLISH_TOKEN=pypi-... uv publish

# via flag
uv publish --token pypi-...
```

### Publish to a custom / test index

Add to `pyproject.toml`:

```toml
[[tool.uv.index]]
name = "testpypi"
url = "https://test.pypi.org/simple/"
publish-url = "https://test.pypi.org/legacy/"
explicit = true
```

Then:

```bash
uv publish --index testpypi
```

### Verify the published package

```bash
uv run --with rp-uv --no-project -- python -c "import rp_uv"
```

---

## 10. Running Tools (uvx / uv tool)

uv replaces [pipx](https://pipx.pypa.io/) for running and installing CLI tools.

### Run a tool without installing it (ephemeral)

```bash
uvx ruff check .
uvx black --check src/
uvx pytest
```

`uvx` is an alias for `uv tool run`. The tool is downloaded to an isolated environment and discarded after execution.

### Install a tool globally

```bash
uv tool install ruff
uv tool install black
```

### List installed tools

```bash
uv tool list
```

### Uninstall a tool

```bash
uv tool uninstall ruff
```

### Upgrade installed tools

```bash
uv tool upgrade ruff
uv tool upgrade --all
```

---

## 11. Most Common Commands

| Command | Description |
|---|---|
| `uv init <name>` | Create a new project |
| `uv python install 3.12` | Download and install Python 3.12 |
| `uv python pin 3.12` | Pin Python version for this project |
| `uv python list` | List available Python versions |
| `uv venv` | Create virtual environment explicitly |
| `uv sync` | Install all dependencies (prod + dev) |
| `uv sync --no-dev` | Install production dependencies only |
| `uv run <script>` | Run a script in the project environment |
| `uv add <pkg>` | Add a production dependency |
| `uv add --dev <pkg>` | Add a dev dependency |
| `uv add --group <grp> <pkg>` | Add a dependency to a named group |
| `uv add --optional <extra> <pkg>` | Add an optional/extra dependency |
| `uv remove <pkg>` | Remove a dependency |
| `uv lock` | Regenerate uv.lock without installing |
| `uv lock --upgrade-package <pkg>` | Upgrade a single package in the lock file |
| `uv lock --upgrade` | Upgrade all packages in the lock file |
| `uv tree` | Show dependency tree |
| `uv build` | Build wheel and sdist into dist/ |
| `uv version --bump minor` | Bump minor version |
| `uv publish` | Publish package to PyPI |
| `uvx <tool>` | Run a tool ephemerally |
| `uv tool install <tool>` | Install a tool globally |
| `uv pip compile pyproject.toml` | Export to requirements.txt |

---

## 12. Quick Setup Summary (Windows)

A typical workflow when setting up a new project on Windows:

```powershell
# 1. Install uv (once, globally)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# 2. Install the Python version you want (once per version)
uv python install 3.12

# 3. Create a new project
uv init rp-uv
cd rp-uv

# 4. Pin the Python version for this project
uv python pin 3.12

# 5. Add dependencies
uv add requests "beautifulsoup4==4.10.0"
uv add --dev pytest black

# 6. Install everything and create .venv automatically
uv sync

# 7. Run your code (no activation needed)
uv run main.py
uv run pytest

# -- OR -- activate manually if preferred
.venv\Scripts\Activate.ps1
python main.py
pytest
```

---

## 13. Comparison: Poetry vs uv

The table below maps every concept and command introduced in the [Poetry guide](../poetry/README.md) to its uv equivalent.

| Feature / Action | Poetry | uv |
|---|---|---|
| **Installation** | `(Invoke-WebRequest ... \| iex)` | `powershell -c "irm ... \| iex"` |
| **New project** | `poetry new <name>` | `uv init <name>` |
| **Config file** | `pyproject.toml` | `pyproject.toml` |
| **Python version — declare** | `requires-python` in `pyproject.toml` | `requires-python` in `pyproject.toml` |
| **Python version — install** | Not built-in (use pyenv) | `uv python install 3.12` |
| **Python version — pin** | Not built-in | `uv python pin 3.12` → `.python-version` |
| **Python version — list** | Not built-in | `uv python list` |
| **Create virtual env** | `poetry env use python` | Auto on `uv sync` / `uv run`; or `uv venv` |
| **Env location** | Poetry cache folder (global) | `.venv/` inside the project |
| **Activate shell** | `poetry shell` | `source .venv/bin/activate` (or `.venv\Scripts\Activate.ps1`) |
| **Run command in env** | `poetry run <cmd>` | `uv run <cmd>` |
| **Install all deps** | `poetry install` | `uv sync` |
| **Install prod only** | `poetry install --no-dev` | `uv sync --no-dev` |
| **Add production dep** | `poetry add <pkg>` | `uv add <pkg>` |
| **Add dev dep** | `poetry add --dev <pkg>` | `uv add --dev <pkg>` |
| **Add optional group dep** | `poetry add --group <grp> <pkg>` | `uv add --group <grp> <pkg>` |
| **Add optional/extra dep** | (not a direct concept) | `uv add --optional <extra> <pkg>` |
| **Remove dep** | `poetry remove <pkg>` | `uv remove <pkg>` |
| **Version constraint — caret** | `^1.2` (>=1.2,<2) | `~=1.2` (compatible release, PEP 508) |
| **Version constraint — tilde** | `~1.2` (>=1.2,<1.3) | `>=1.2,<1.3` |
| **Version constraint — exact** | `==1.2.3` | `==1.2.3` |
| **Version constraint — wildcard** | `1.2.*` | `==1.2.*` |
| **Lock file** | `poetry.lock` | `uv.lock` |
| **Regenerate lock (no install)** | `poetry lock` | `uv lock` |
| **Update single package** | `poetry update <pkg>` | `uv lock --upgrade-package <pkg>` |
| **Update all packages** | `poetry update` | `uv lock --upgrade` |
| **Show dependency tree** | `poetry show --tree` | `uv tree` |
| **Build distribution** | `poetry build` | `uv build` |
| **Bump version** | Manual edit | `uv version --bump minor/patch/major` |
| **Publish to PyPI** | `poetry publish` | `uv publish` |
| **Publish credentials** | `poetry config pypi-token.pypi <token>` | `UV_PUBLISH_TOKEN=<token> uv publish` |
| **Init existing project** | `poetry init` | `uv init` (or just `uv sync`) |
| **Import from requirements.txt** | `poetry add` (manual) | `uv add -r requirements.txt` |
| **Export to requirements.txt** | `poetry export -f requirements.txt` | `uv pip compile pyproject.toml -o requirements.txt` |
| **Run tool without installing** | Not built-in (use pipx run) | `uvx <tool>` |
| **Install global CLI tool** | Not built-in (use pipx) | `uv tool install <tool>` |
| **Speed** | Moderate | 10–100x faster (Rust) |
| **Python version management** | No (rely on pyenv) | Yes — `uv python` |
| **Cross-platform lock file** | Yes | Yes |
| **Workspace / monorepo support** | Limited | Yes — `uv` workspaces |
