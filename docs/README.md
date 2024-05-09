# Beluga documentation

## Prerequisites

First, install base system dependencies (e.g. Doxygen). On Ubuntu:

``` sh
sudo apt install doxygen git graphviz make python3-pip texlive-latex-base
```

Then, satisfy Python requirements:

``` sh
pip install -r requirements.txt
```

> [!TIP]
> Use a [virtual environment](https://docs.python.org/3/library/venv.html) to avoid polluting your system.

## Build HTML documentation

Simply make it!

```sh
make
```
