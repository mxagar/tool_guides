# Python DOCX Quotes

Small `python-docx` project for generating budget and receipt documents from JSON data.

## Setup

From this folder, create a local uv virtual environment in `.venv`. The folder is `.venv`, but the prompt/kernel can still be called `docx`:

```bash
cd python_docx
uv venv .venv --prompt docx
source .venv/bin/activate
uv sync --no-install-project
```

This installs only the dependencies from `pyproject.toml`. The project modules are not installed as a package; the notebook imports them directly from `src/`.

Register the environment as a Jupyter kernel:

```bash
python -m ipykernel install --user --name docx --display-name "Python (docx)"
```

Then open the notebook:

```bash
jupyter notebook python_docx_tests.ipynb
```

The generated `.docx` and `.pdf` files are written to `output/` when the notebook runs.

PDF export requires either:

- Microsoft Word, used through `docx2pdf`.
- LibreOffice available on `PATH` as `soffice` or `libreoffice`.

If neither converter is installed, the notebook still generates the Word files and prints a clear PDF export message.

## Project Layout

```text
python_docx/
├── data/
│   ├── budget.json
│   └── receipt.json
├── src/
│   ├── documents.py
│   └── models.py
├── output/
├── pyproject.toml
├── SPEC.md
└── python_docx_tests.ipynb
```

## Usage

The notebook demonstrates two paths:

1. Direct `python-docx` API usage with a minimal document.
2. A reusable quote/receipt workflow where JSON is validated with Pydantic and rendered through `Budget` and `Receipt` classes imported directly from `src/`.
