Look at the folder @python_docx. Do the following:

- Create a uv environment called docx where you install the basic data science packages (matplotlib, pandas, numpy, etc.) as well as the python-docx package. To that end, you need a pyproject.toml. Also, extend the README.md with a section "Setup" where you explain how to install the packages in the env.
- Create in the same folder a notebook calles python_docx_textx.ipynb.
- In the notebook, I want these sections:
  - 1. Basic Usage: test basic API usage
  - 2. Use Case: Quotes. The idea is to create budget and quotes/receipts from a JSON. To that end, I have thought of the following:
    - There can be a base class like BaseDocument which defines a header with all the elements you would expect in a receipt: company address, lient name & surnames, company logo, date, etc.
    - The BaseDocument should have also a footer.
    - Then, I need to derive from BaseDocument other 2 classes: Budget, Receipt.
    - Both are very similar: they should contain a list of products in a table, specifying price per unit, number of units, subtotal per row, and a total + taxes (IVA, in Spain), plus gross total.
    - The budget should have a date until which the budget is valid, and a budget id.
    - The receipt should have a bank account for the transference, as well as a QR code + receipt uuid, and a budget id to refer to. It should say the number of day the client has to pay.
    - Create Pydantic classes for the JSON models, as well as 2 JSON mocks, one for receipt, one for budget.
    - Then, instantiate the Budget and Receipt, classes, read the JSONs and create the Word documents.
    - Finally export them to docx and PDF.
- Create a SPEC.md file which contains the project definition, as well as a plan to implement it.
- Then, implement the project following SPEC.md
