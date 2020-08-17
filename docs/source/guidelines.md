Coding Guidelines
================================
We use the Google Style Guides for Python and C++ as a reference.


## Coding Guidelines C++ Code

For C++ code, we use cpplint.
It is installed automatically within your virtual environment.
However, to use it in VS Code, we use the VS Code Extension cppline (Developer: mine, version 0.1.3).
You can install it in the market place. When installed, rightclick on the extension and select "Configure Extension Settings".
You now need to define the path to cpplint as local user.

```
Cpplint: Cpplint Path
The path to the cpplint executable. If not set, the default location will be used.
/(YOUR LOCAL PATH FOR BARK)/bark/python/venv/bin/cpplint
```


## Coding Guidelines Python

Pylint and autopep8 are installed automatically to your virtual environment.
When sourced in your VS Code terminal, you should be able to only click "Format Document".
In case it asks for formatting guide, select pep8.
However, this should come automatically with .vscode/settings.json.
