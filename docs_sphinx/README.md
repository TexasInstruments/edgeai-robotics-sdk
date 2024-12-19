# Robotics SDK Sphinx Doc Generation

Generate the SDK documentation using Sphinx with MyST parser.

## How to Generate the Documentation

1. The gscam package should be installed before building the SDK documentation.
    ```bash
    SDK_DIR=$(pwd) scripts/install_gscam.sh
    ```

2. (One-time only) Setup a Python virtual environment for the Sphinx documentation. Side Note: We found issues if a venv is set up directly under `docs_sphinx` folder.
    ```bash
    python3 -m venv ./.venv_sphinx
    unset PYTHONPATH
    source ./.venv_sphinx/bin/activate
    pip install -r docs_sphinx/requirements.txt
    deactivate
    ```

3. Activate the Python virtual environment
    ```bash
    unset PYTHONPATH
    source ./.venv_sphinx/bin/activate
    ```

4. Change to working directory
    ```bash
    cd docs_sphinx
    ```

5. Update `index.rst` and `conf.py`:
    - For documentation for the J7x family, update `index.rst` and `conf.py` under `j7x` folder.
    - For documentation for AM62A, update `index.rst` and `conf.py` under `am62a` folder.

6. Run the following script
    ```bash
    ./gen_html.sh
    ```

The output documentation:
    - HTML documentation: under `_build_<device>/docs`
    - Zipped documentation: `robotics_sdk_doc_<data>_<device>.zip`

## Known Issue

1. Several "reference target not found" warnings as shown below pop up. These warnings come from text portions that should be ideally disabled with `{only}` directives in MyST markdown. Therefore, it is safe to ignore the warnings.
