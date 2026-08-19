"""Domain-oriented benchmark test cases.

The compatibility runner imports the domain modules explicitly. Keeping this
initializer free of TestCase objects prevents unittest discovery from loading
the same classes once through the package and once through the runner.
"""
