# FAQ

### 1. Gflags
  Q: I have added/changed a gflag but it has no effect?

  A: Mind that according to gflags specs: 
  > If a flag is specified more than once, only the last specification is used; the others are ignored.

### 2. Euroc dataset
  Q: Frame mismatch in Euroc? Seeing this error msg:
  ```
    W0911 10:22:30.347504 27725 ETH_parser.cpp:520] Different number of images in left and right camera!
    Left: 2033
    Right: 2032
  ```

  A: Euroc has datasets with mismatching number of frames, use instead our dataset files [here](https://drive.google.com/open?id=1_kwqHojvBusHxilcclqXh6haxelhJW0O).

### 3. Linter

  Q: Getting this error:
  ```bash
    Traceback (most recent call last):
      File ".git/hooks/pre-commit", line 55, in <module>
        main()
      File ".git/hooks/pre-commit", line 49, in main
        import linter
    ImportError: No module named linter
  ```
  A:
    Make sure you ran
    ```bash
      cd linter
      echo ". $(realpath setup_linter.sh)" >> ~/.bashrc
      bash
    ```

  Q: Getting this error:
  ```
    Traceback (most recent call last):
      File ".git/hooks/pre-commit", line 55, in <module>
        main()
      File ".git/hooks/pre-commit", line 51, in main
        linter.linter_check(repo_root, linter_folder)
      File "/home/tonirv/Code/Kimera-VIO/dev_tools/linter/linter.py", line 483, in linter_check
        ascii_art, repo_root)
      File "/home/tonirv/Code/Kimera-VIO/dev_tools/linter/linter.py", line 124, in check_cpp_lint
        cpplint = imp.load_source('cpplint', cpplint_file)
    IOError: [Errno 2] No such file or directory
  ```

  A: Make sure you initialized the linter:
  ```bash
    init_linter_git_hooks 
    # Success, githooks initialized!

  ```

