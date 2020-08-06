# Contributing

We would encourage anyone to contribute to BARK to push research in the field of autonomous driving!

## Pull Requests

We gladly welcome [pull requests](
https://help.github.com/articles/about-pull-requests/).

Before making any changes, we recommend opening an issue (if it
doesn't already exist) and discussing your proposed changes. This will
let us give you advice on the proposed changes. If the changes are
minor, then feel free to make them without discussion.

Want to contribute but not sure of what? Here are a few suggestions:

1. Add new blueprints, examples, and more. These are a great way to familiarize
   yourself and others with BARK.


2. Solve an [existing issue](https://github.com/bark-simulator/bark/issues).
   These range from low-level software bugs to higher-level design problems.

All submissions, including submissions by project members, require review. After
a pull request is approved, we merge it.
## Style

See the [style guide](STYLE_GUIDE.md).

## Unit tests

All BARK code-paths must be unit-tested. See existing unit tests for
recommended test setup.

Unit tests ensure new features (a) work correctly and (b) guard against future
breaking changes (thus lower maintenance costs).

To run existing unit-tests, use the command:


```shell
bazel test //...
```

from the root of the `BARK` repository and inside the BARK specific virtualenv.


## Contributor License Agreement

All contributions are made under MIT license.
