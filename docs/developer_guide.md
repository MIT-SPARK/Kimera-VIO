# Developer Guide

In general, we follow [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). 

But, in particular, we follow [aslam_cv](https://github.com/ethz-asl/aslam_cv2/wiki/Aslam-cv-specific-code-style), which is given below:

In this repository we follow the ASL coding guidelines with some modifications:

### Programming style
* **Don't be fancy**: If you want to show off weird code that exploits a specific particularity of a compiler or the c++ language try it somewhere else. The code should be efficient but also readable by humans. There is a place for SSA and inline assembly, but 99.9% of the time, a normal implementation is as efficient as a complicated one, or it just doesn't matter anyway.
* **Be efficient at the appropriate time**: Think about if a particular optimization just degrades readability and shows off what a great programmer you are or if it is actually improving performance.
* **Watch your algorithmic complexity and memory access**: On most modern machines the memory access is the limiting factor, watch memory layout, pointer indirection and loop-ordering as these are most strongly influencing performance.

### Variable and parameter naming
* Prefer writing out a human understandable name and avoid abbreviations where possible.
* Strictly keep the correct mathematical notation for frames and transformations as discussed [here](Expressing frame transformations in code.). Add short comments if the single letter frame specification is not self explanatory, but do not write the full name for a frame.

### Naming of variables and methods
* Name your variables s.t. other people can directly understand what is going on.
```
bool projection_successful = camera->Project(/*...*/)
```
* Methods should contain a verb s.t. it is clear what a method does:
```
bool UndistortImage(const cv::Mat& input_image, cv::Mat* output_image) const {
// ...
}
```

### Commenting
* **Do not** comment on obvious parts of the code.
  * Don't do this:
  ```
   // Allocate an empty image.
   cv::Mat img;
   // Load the image from the file specified by file_name.
   img.load(file_name);
   ```
* Comment on parts of the code which are non-standard or where to document where ever a non-standard approach was taken for a non-obvious reason.
  * But do this:
   ```
   // O_DIRECT writing uses DMA and to achieve this needs the input memory to be
   // memory aligned to multiples of 4096.  See man 2 memalign for details.
   #ifdef ANDROID
   #define posix_memalign(a, b, c) (((*a) = memalign(b, c)) == NULL)
   #endif
   ```

### Function signatures
#### Parameter passing
* Input values of primitive types (int, double, bool etc.) are **not** passed by value, but by `const &`. Although this code design choice is suboptimal, the performance improve of not doing so is minimal compared with the potential cost of missing a `const &` on a complex object.
* Input values of complex types (classes etc) are passed by reference to const.
* Input/Output values are passed by pointer.
* Output values are passed by pointer.

#### Passing shared pointers
* Input shared pointers are passed by reference to const.
* Shared pointers where the content (the object that the pointer points to) is modified are passed by value
* Shared pointers which are modified themselves are passed by pointer.

#### Parameter ordering
* Input, Input/Output, Output

#### Default values
* Do not put default values for methods. They are too often introducing bugs when parameters get reordered. Instead use overloads. [Discussion](https://google.github.io/styleguide/cppguide.html#Default_Arguments)

#### Parameter checking
* All parameters passed by pointer need to be checked for null. Use the GLog ```CHECK_NOTNULL(pointer);``` macro for raw pointers and the GLog ```CHECK(shared_ptr)``` macro for shared_ptrs.

#### Example of function signature

```cpp
void myFunctionSignature(const Input& input, InputOutput* input_output, Output* output) {
 CHECK_NOTNULL(input_output);
 CHECK_NOTNULL(output);

 // Your code
 *output = ...
}
```

### Value Checks
Use the glog ```CHECK(x)``` macros instead of assertions. http://google-glog.googlecode.com/svn/trunk/doc/glog.html### 

Unlike assert, it is *not* controlled by NDEBUG, so the check will be executed regardless of compilation mode.
Nevertheless, you can use `DCHECK(x)` for assertions that are only ran when in Debug mode.

Prefer the specific macros where possible:
```
CHECK_NOTNULL(some_ptr);
some_ptr->DoSomething();

CHECK_EQ(2, 1 + 1)
CHECK_EQ(std::vector<int>().size(), 0u);

CHECK(bool(1), true);

CHECK_LT(1, 2);
CHECK_NEAR(1.0, 1.0000001, 1e-4);
```


#### Other considerations:

* Keep lines of code to 80 characters: this is to make it easy to review code
from a terminal and for coding in split view mode in an IDE.
 For example: here `is everything ok?` wouldn't be visible.
![a](https://github.mit.edu/storage/user/11308/files/2c46bca6-0ac1-11e9-93e0-471aff017d9d)

* Use spaces around all arithmetic operators.
* Comments should start with a capital letter and end with a period.
* We use `glog` for assertions in code and to have fine-grained control over the pipeline logging.
A great guide on how to use `glog` is available [here](http://rpg.ifi.uzh.ch/docs/glog.html). Please, read it at least once.
* We use 'gflags` for command-line parameters (flags). Please, read at least once this great [guide](https://gflags.github.io/gflags/) so that you understand what are these `DEFINE_...` and `DECLARE_` macros that are at the top of the cpp files.
* Do not hardcode values! The only allowable values in the code should be either 0 or 1, any different value can be considered a hardcoded value, these are evil. Other values should be parsed in one way or another or set as parameters. Default values are allowed but discouraged as they are easily omitted and misused by lazy developers/users.

## Includes
Only include what you strictly need! Don't include randomly and hope for your compile errors to go away... Badly used includes make compilation times way longer than they should! If you value your time, please think deeply when to use an include:
Here some guidelines:
1) Keep all your implementation strictly in .cpp files so that ideally all your includes are in the .cpp file so you don't add dependencies to people including your header file.
2) Most likely your .h also have dependencies, but these can be kept at a minimum by:

i) Stick to **PIMPL approach**: Pointer to Implementation, which suggests that, having your class members, if complex, might be instead changed for pointers! This is extensively used in the VIO pipeline with the std::unique_ptr (smart pointers that cannot be copied) for things such as the Frontend, the mesher, the Backend etc...

ii) Only ever include files for members inside the class. Those you cannot avoid, because the compiler needs to know the size of your classes in your .h!
Following these guidelines and common sense ones like not adding random includes and remembering to remove those that are no longer needed should remove 80% of the includes we have.
On top of that, use as many .h as possible to allow others and yourself to just include the strictly necessary things: for example, the imu definitions, used by many, are set in a separate header file, so that not everyone includes the imu_frontend!

iii) Altough not recommended (see [google cpp styleguide](https://google.github.io/styleguide/cppguide.html#Forward_Declarations)), you can use **Fast-forward definitions**: this is typically the case for your .h functions that ask for arguments of a given type. Note that the compiler does not need to know the exact type of these arguments, so you can, at the top of your .h just tell the compiler that your argument is indeed a type, using: `class your_type;` That's it!! Neverthe
We use git and stick to a strict "branch, pull-request, review, merge" workflow.

### Commits Guidelines:
* Each commit should strive to be **compilable**.
* Each commit should pass all tests: `make test`.
* Each commit message should start with a capital letter, and end with a period. Keep commit titles short (80 characters) but informative.
* Each commit should be self-contained in functionality so that it can be easily reverted. One conceptual function/feature modified at a time if possible.
* Commit often.
