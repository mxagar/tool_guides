# GTest Usage Examples

Usage examples derived mainly from [Google Test Primer](https://github.com/google/googletest/blob/master/googletest/docs/primer.md)

This project builds up on the `gtest_example_01` project, which downloads googletest from GitHub and compiles its together with our existing CMake project.
It is the recommented way; see [generic build instructions of googletest](https://github.com/google/googletest/tree/master/googletest).

Files in the folder:

- `CMakeLists.txt.in`: the GTest repository download commands are specified,

- `CMakeLists.txt`: related to the project. The first lines of this file include `CMakeLists.txt.in` and prepare GTest; the last lines add the binaries of our project and link them against GTest.

- `example.cpp`: the source code in which the tests are implemented.

## Introduction

Google tests are designed to be:
- isolated (independent from other test results) and repeatable
- organized: grouped into test suites
- cross-platform and reusable
- information-rich: they should give info on the cause for the result and continue even if a test fails
- trackable: they are tracked for the user, so he/she doesn't need to enumerated them
- fast

GTest is based on xUnit architecture -> similar to JUnit, PyUnit.

## Nomenclature

**Test (GTest)** = `TEST()` = 'Test Case', according to the *International Software Testing Qualifications Board*: **assertions** to check if the code works.

**Test Case/Suite (GTest)** = 'Test Suite', according to the *International Software Testing Qualifications Board*: one or many **tests** grouped.

**Test Program (GTest)** = it can contain multiple **test suites**.
## Assertions

Assertions are macris that resemble function calls. They check whether a condition is true; 3 results are possible:
- success
- nonfatal failure: message output
- fatal failure: message output + current function aborted, otherwise continued

Two types:
- `ASSERT_*`: if condition not met, **fatal failure**
- `EXPECT_*`: if condition not met, **nonfatal failure**

**Basic General** (fatal - nonfatal - verification condition):\
ASSERT_TRUE(condition);     EXPECT_TRUE(condition);     actual == expected, actual >= min, ...\
ASSERT_FALSE(condition);    EXPECT_FALSE(condition);	actual == expected, actual >= min, ...\

**Binary** (fatal - nonfatal - verification condition):\
ASSERT_EQ(val1, val2);  EXPECT_EQ(val1, val2);  val1 == val2;\
ASSERT_NE(val1, val2);  EXPECT_NE(val1, val2);  val1 != val2;\
ASSERT_LT(val1, val2);  EXPECT_LT(val1, val2);  val1 < val2;\
ASSERT_LE(val1, val2);  EXPECT_LE(val1, val2);  val1 <= val2;\
ASSERT_GT(val1, val2);  EXPECT_GT(val1, val2);  val1 > val2;\
ASSERT_GE(val1, val2);  EXPECT_GE(val1, val2);  val1 >= val2;\
If user-defined types used, operators must be defined.\
ASSERT_EQ(actual, expected) preferred to ASSERT_TRUE(actual == expected), since it notifies actual and expected's values on failure.\

**Strings** (fatal - nonfatal - verification condition):\
ASSERT_STREQ(str1,str2);    EXPECT_STREQ(str1,str2);	both C strings same content\
ASSERT_STRNE(str1,str2);    EXPECT_STRNE(str1,str2);	both C strings different content\
ASSERT_STRCASEEQ(str1,str2);	EXPECT_STRCASEEQ(str1,str2);	both C strings same content, ignoring case\
ASSERT_STRCASENE(str1,str2);	EXPECT_STRCASENE(str1,str2);	both C strings same content, ignoring case\

## Simple Tests

They have the following structure:

```C++
TEST(TestSuiteName, TestName) {
  ... test body ...
}
```

They are macros that create functions without return values, only the test result handled by GTest.
In the test body, we can regularly write C++ code and use GTest assertions.
Arguments (naming convention: try to use CamelCase instead of _):
- `TestSuiteName`: name of the test suite
- `TestName`: name of the specific test (the full name is suite + test name)

Tests of the same suite can check different params of the same function, e.g.: `inv(R)`, `inv(R^(-1))`.

## Test Fixtures

If we write 2+ tests that operate on similar data, we can use a test fixture: a class is derived from `::testing::Test` which uses our tested data (-structure). This class can have custom constructor/initialization `void SetUp()` and descrutctor/cleanup `void TearDown()`. Then, the test fixture is defined as follows:


```C++
TEST_F(TestFixtureClass, TestName) {
  ... test body ...
}
```

Inside `TEST_F` we use `ASSERT_` and `EXPECT_` as always, according to what we defined in the TestFixtureClass. We can also access the variables defined inside TestFixtureClass.

See `example.cpp`.

## Invoking Tests

Usually, we execute the binary/ies in which we have defined the tests, and that's it.

`TEST()` and `TEST_F()` register their tests, so no re-listing needs to be performed.

**Optionally**, `RUN_ALL_TESTS()` can be executed (only once) in a `main()` file/function, which is linked against `gtest` instead of the regular `gtest_main`. `RUN_ALL_TESTS()` executes all tests defined in different source files of the project. It returns `0` if all tests successful, `1` otherwise. This output must be the return value of the `main()` where `RUN_ALL_TESTS()` is run. However, most users should not use `main()` & `RUN_ALL_TESTS()`; see the documentation for more information.

