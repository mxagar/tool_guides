/*
 * example.cpp
 * Examples of usage of Google Test.
 * To execute the tests, simply execute the binary: ./example
 * Read the comparion README.md for a more complete understanding.
 * These examples were derived from
 * https://github.com/google/googletest/blob/master/googletest/docs/primer.md
 */

#include <iostream>
#include <vector>

#include "gtest/gtest.h"

//
// 1. ASSERTIONS AND SIMPLE TESTS
//


// Simple test (simple_test_1) that belongs to a Test Suite (sample_test_suite)
// Arguments: TestSuiteName, TestName -> full test name: TestSuiteName.TestName
// Naming convention: try to use CamelCase instead of _
TEST(sample_test_suite, simple_test_1)
{
    EXPECT_EQ(1, 1);
}

// Another simple test (simple_test_2) that belongs to the same test suite (sample_test_suite)
// Tests of the same suite can check different params of the same function, eg: inv(R), inv(R^(-1))
TEST(sample_test_suite, simple_test_2)
{
    EXPECT_EQ(2, 2);
}

// Third simple test (simple_test_3) that belongs to the same test suite (sample_test_suite)
// ASSERT_ and EXPECT_ used
TEST(sample_test_suite, simple_test_3)
{
    std::vector<int> x, y;
    for (unsigned int i = 0; i < 3; ++i) {
        //x.push_back(i); y.push_back(i);
        x.push_back(i); y.push_back(i + 1);
    }

    // Assertion with fatal failure if not true (test function exited)
    // Custom message in case of failure passed with '<<'
    ASSERT_EQ(x.size(), y.size()) << "Vectors x and y are of unequal length";

    for (int i = 0; i < x.size(); ++i)
    {
        // Assertion with nonfatal failure if not true (test function continues)
        // Custom message in case of failure passed with '<<'
        EXPECT_EQ(x[i], y[i]) << "Vectors x and y differ at index " << i;
    }
}

/*

CATALOGUE OF ASSERTIONS:

1. Basic General (fatal - nonfatal - verification condition):
ASSERT_TRUE(condition);     EXPECT_TRUE(condition);     actual == expected, actual >= min, ...
ASSERT_FALSE(condition);    EXPECT_FALSE(condition);	actual == expected, actual >= min, ...

2. Binary (fatal - nonfatal - verification condition):
ASSERT_EQ(val1, val2);  EXPECT_EQ(val1, val2);  val1 == val2;
ASSERT_NE(val1, val2);  EXPECT_NE(val1, val2);  val1 != val2;
ASSERT_LT(val1, val2);  EXPECT_LT(val1, val2);  val1 < val2;
ASSERT_LE(val1, val2);  EXPECT_LE(val1, val2);  val1 <= val2;
ASSERT_GT(val1, val2);  EXPECT_GT(val1, val2);  val1 > val2;
ASSERT_GE(val1, val2);  EXPECT_GE(val1, val2);  val1 >= val2;
If user-defined types used, operators must be defined.
ASSERT_EQ(actual, expected) preferred to ASSERT_TRUE(actual == expected), since it notifies actual and expected's values on failure

3. Strings (fatal - nonfatal - verification condition):
ASSERT_STREQ(str1,str2);    EXPECT_STREQ(str1,str2);	both C strings same content
ASSERT_STRNE(str1,str2);    EXPECT_STRNE(str1,str2);	both C strings different content
ASSERT_STRCASEEQ(str1,str2);	EXPECT_STRCASEEQ(str1,str2);	both C strings same content, ignoring case
ASSERT_STRCASENE(str1,str2);	EXPECT_STRCASENE(str1,str2);	both C strings same content, ignoring case

*/

//
// 2. TEST FIXTURES
//

// First, we deine our data structure to be tested:
// a very simple container
class SimpleContainer
{
public:
    int size(void) { return m_values.size(); };
    void add(int value) { m_values.push_back(value); };
    bool get(int index, int& value) {
        if (index < size()) {
            value = m_values[index];
            return true;
        } else {
            value = -1;
            return false;
        }
    }
private:
    std::vector<int> m_values;
};

// Then, we define the test fixture
// TestFixtureClass: Class derived from '::testing::Test' that uses our data structure
// The tests are going to be defined based on this class
// We can define/override the initialization method Setup() and the clearing method TearDown(), if needed
class SimpleContainerTest : public ::testing::Test
{
protected:

    // Initializations, if required
    void SetUp() override
    {
        c1.add(1);
        c2.add(2);
        c2.add(3);
    }

    // Clearing, if required
    // void TearDown() override {}

    // 
    SimpleContainer c0;
    SimpleContainer c1;
    SimpleContainer c2;

};

// TestFixtureClass, TestName = SimpleContainerTest, IsEmptyInitially
// We pass the test fixture class (TestFixtureClass) and the name of the test on that fixture
// Inside TEST_F we use ASSERT_ and EXPECT_ as always according to what we defined in the TestFixtureClass (= SimpleContainerTest)
// We can access to the variables defined inside TestFixtureClass (= SimpleContainerTest)
TEST_F(SimpleContainerTest, IsEmptyInitially)
{
    EXPECT_EQ(c0.size(), 0);
}

TEST_F(SimpleContainerTest, AddGetWorks)
{
    ASSERT_EQ(c1.size(), 1);
    ASSERT_EQ(c2.size(), 2);
    int v;
    if (c2.get(1,v)) {
        EXPECT_EQ(v,3);
    }
}