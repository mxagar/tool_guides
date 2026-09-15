"""
Synchronize Jupyter notebooks with their Python script counterparts using Jupytext.

Usage:

    # 1. Convert a single .ipynb to .py
    # ALWAYS do it to add the notebook as .py to the repository
    cd .../notebooks
    uv run --with jupytext python sync_notebooks.py add example_notebook.ipynb

    # 2. Sync all notebooks with their .py scripts
    cd .../notebooks
    uv run --with jupytext python sync_notebooks.py sync

"""

import pathlib
import argparse
import subprocess


def sync_notebooks() -> None:
    """
    Synchronize all Jupyter notebooks in the current notebooks directory with their paired Python scripts.
    It uses jupytext's --sync option to ensure both formats are up to date.
    """
    notebooks_dir = pathlib.Path(".")
    notebooks_dir_posix = notebooks_dir.as_posix()
    subprocess.run(["jupytext", "--sync", f"{notebooks_dir_posix}/*.py"], check=False)


def add_notebook(filepath: str) -> None:
    """
    Convert a single Jupyter notebook to a Python script in py:percent format using jupytext.

    Args:
        filepath: Path to the .ipynb file to convert, e.g.: my_notebook.ipynb
    """
    # 1. Convert to py:percent
    subprocess.run(["jupytext", "--to", "py:percent", filepath], check=False)

    # 2. Set formats using JSON-style metadata update (correct usage)
    subprocess.run([
        "jupytext",
        "--set-formats", "ipynb,py:percent",
        "--update-metadata", '{"jupytext":{"formats":"ipynb,py:percent"}}',
        filepath
    ], check=False)

    # 3. Sync to ensure paired .py and .ipynb are aligned
    subprocess.run(["jupytext", "--sync", filepath], check=False)
    
def main():
    # Create the argument parser
    parser = argparse.ArgumentParser(
        description="Sync Jupyter notebooks with their .py script counterparts using jupytext."
    )
    
    # Define subcommands for sync and add
    subparsers = parser.add_subparsers(dest="command", required=True)
    sync_parser = subparsers.add_parser("sync", help="Sync all notebooks with their paired .py scripts")
    add_parser = subparsers.add_parser("add", help="Convert a single notebook to py:percent format")
    add_parser.add_argument("notebook", help="Path to the .ipynb notebook")

    # Parse
    args = parser.parse_args()

    if args.command == "sync":
        sync_notebooks()
    elif args.command == "add":
        add_notebook(args.notebook)


if __name__ == "__main__":
    main()
