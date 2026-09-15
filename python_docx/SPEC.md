# Python DOCX Quotes Project

## Goal

Build a small, reusable Python project that demonstrates `python-docx` basics and generates business budget and receipt documents from JSON data.

## Environment

The project uses `uv` and a local virtual environment stored in `.venv`. The environment and Jupyter kernel may be displayed as `docx`, but the folder name is `.venv`.

Core dependencies:

- `python-docx` for Word document creation.
- `pydantic` for JSON validation.
- `qrcode` and `Pillow` for receipt QR code generation.
- `numpy`, `pandas`, and `matplotlib` as common data science packages.
- `docx2pdf` for PDF conversion when Microsoft Word is available.
- `ipykernel` and `jupyter` for notebook usage.

PDF export is platform-dependent. The implementation tries `docx2pdf` first, then the `soffice`/LibreOffice command if available.

The project code is not installed as a package. The notebook adds `src/` to `sys.path` and imports `documents.py` and `models.py` directly.

## Deliverables

- `pyproject.toml`: project metadata and dependencies.
- `README.md`: setup and usage instructions.
- `python_docx_tests.ipynb`: notebook with:
  - Basic `python-docx` API usage.
  - Budget and receipt generation from JSON.
- `src/documents.py`: reusable document rendering code.
- `src/models.py`: Pydantic input models.
- `data/budget.json`: mock budget input.
- `data/receipt.json`: mock receipt input.
- Generated files under `output/` when the notebook is executed.

## Data Model

Shared models:

- `Address`: street, city, postal code, country.
- `CompanyInfo`: company name, tax id, address, email, phone, optional website, optional logo path.
- `ClientInfo`: client name, surname, tax id, address, email, optional phone.
- `LineItem`: description, unit price, quantity, optional unit.
- `DocumentBaseData`: company, client, document date, notes.

Budget-specific model:

- `budget_id`.
- `valid_until`.
- `items`.
- `iva_rate`.

Receipt-specific model:

- `receipt_uuid`.
- `budget_id`.
- `payment_due_days`.
- `bank_account`.
- `items`.
- `iva_rate`.

## Document Classes

- `BaseDocument`
  - Builds a common document skeleton.
  - Adds header content with company/client/date details.
  - Adds footer content.
  - Provides helpers for item tables and totals.
  - Saves `.docx` and attempts `.pdf` export.
- `Budget`
  - Extends `BaseDocument`.
  - Adds budget id and validity date.
  - Adds products/services table and totals.
- `Receipt`
  - Extends `BaseDocument`.
  - Adds receipt UUID, linked budget id, payment deadline, bank account, QR code, table, and totals.

## Implementation Plan

1. Create the `uv`-compatible project scaffold with dependencies.
2. Define Pydantic models and money/tax calculations.
3. Implement `BaseDocument`, `Budget`, and `Receipt` classes.
4. Add JSON mocks for budget and receipt data.
5. Create a notebook that:
   - Creates a minimal Word file with direct `python-docx` API usage.
   - Adds `src/` to `sys.path`.
   - Loads the JSON mocks.
   - Validates them with Pydantic.
   - Generates budget and receipt `.docx` files.
   - Attempts PDF export.
6. Update README setup and execution instructions.
7. Validate TOML, Python syntax, and notebook JSON structure.
