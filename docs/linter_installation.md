## Linter Installation

### Linter Dependencies

 * `pip install requests pyline yapf`
 * **clang-format**: Compatible with `clang-format-3.8 - 6.0`
   * Ubuntu:
    ```bash
    sudo apt install clang-format-${VERSION}
    ```

   * macOS:
    ```bash
    brew install clang-format
    ln -s /usr/local/share/clang/clang-format-diff.py /usr/local/bin/clang-format-diff
    ```

### Install Linter

```bash
cd SparkVIO
git submodule update --init
echo "source $(realpath ./dev_tools/linter/setup_linter.sh)" >> ~/.bashrc
# Or the matching file for your shell
bash
```

Finally, initialize linter:
```bash
cd SparkVIO
init_linter_git_hooks
```

> For more information about the linter, check [here](https://github.com/ToniRV/linter).

Go back to [Readme](../README.md).