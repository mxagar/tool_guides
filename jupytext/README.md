# Jupytext

Jupytext keeps Jupyter notebooks and plain text files in sync. We use it to pair each `.ipynb` notebook with a `.py` file in `py:percent` format.

The `.py` copy is much easier to review in Git than notebook JSON: diffs are readable, merge conflicts are smaller, and code review tools can show normal Python changes. The `.ipynb` file remains useful for interactive notebook work, outputs, and the Jupyter UI.

## What the Script Does

[`sync_notebooks.py`](sync_notebooks.py) is a small wrapper around the Jupytext CLI.

- `add` converts one notebook to a paired `.py` file and records the pairing metadata.
- `sync` updates paired `.ipynb` and `.py` files so the newer side is copied to the older side.

The script expects to be run from the directory that contains the notebooks you want to manage.

## Installation With uv

If this repository already has a `pyproject.toml`, add Jupytext as a development dependency:

```bash
uv add --dev jupytext
```

Then run the helper through `uv`:

```bash
uv run python sync_notebooks.py sync
```

If you do not want to add Jupytext to the project dependencies, use `--with` for a one-off run:

```bash
uv run --with jupytext python sync_notebooks.py sync
```

This is enough because the helper itself only uses the Python standard library, but it calls the `jupytext` command internally.

## Add a New Notebook

When adding a notebook to the repository, first create the paired Python file:

```bash
cd path/to/notebooks
uv run --with jupytext python sync_notebooks.py add example_notebook.ipynb
```

This creates `example_notebook.py` in `py:percent` format and configures the notebook to stay paired with it.

Commit both files:

```bash
git add example_notebook.ipynb example_notebook.py
```

## Sync Existing Notebooks

After editing either the notebook or the paired Python file, sync before committing:

```bash
cd path/to/notebooks
uv run --with jupytext python sync_notebooks.py sync
```

If Jupytext is already in the project dependencies, the shorter command is:

```bash
uv run python sync_notebooks.py sync
```

## Typical Workflow

1. Create or edit a notebook in Jupyter.
2. Run `uv run --with jupytext python sync_notebooks.py add notebook.ipynb` the first time the notebook is added.
3. Run `uv run --with jupytext python sync_notebooks.py sync` before committing future changes.
4. Review and commit both the `.ipynb` and `.py` files.


