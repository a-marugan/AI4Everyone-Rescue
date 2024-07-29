# Simplified Style Guide

The following is a basic style guide for the Social AI / AI-4-Everyone VIP Project (UNSW, Sydney). The style guide was written for **Python** and **CPP**. The style guide serves to ensure that the style of the code developed for the project is consistent. For an official style guide for Python, go to this [link](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/blob/robot_team1/Style%20Guides/Other%20Style%20Guides/Python.pdf). For an official style guide for CPP, go to this [link](https://nw-syd-gitlab.cseunsw.tech/z9600614/VIP-AI4Everyone-Rescue/-/blob/robot_team1/Style%20Guides/Other%20Style%20Guides/CPP__MTRN2500_.pdf).

Please do your utmost to adhere to the guide. Note that this style guide may be updated at a future date.

## Structure

The structure of the code should be written in the following order.

| Python | CPP |
| --- | --- |
| <ul><li> File header comment describing the contents of the file, author, and date last updated. </li><li> Import Python Packages. </li><li> Declaration of global constants. </li><li> Forward declarations for classes and functions (optional but recommended). </li><li> Definition for class methods and functions. </li><li> Main function. </li></ul> | <ul><li> File header comment describing the contents of the file, author, and date last updated. </li><li> Import CPP libraries. </li><li> Declaration of namespaces, global constants, typedefs, etc. </li><li> Forward declarations for classes and functions (optional but recommended). </li><li> Definition for class methods and functions. </li><li> Main function. </li></ul> |

Regarding constants, avoid non-constant global variables.
* Global variables for constants is acceptable, but should be written in completely uppercase characters.
* For CPP, use `constexpr` or `const` to instantiate these constants.

## Naming Conventions

When naming variables, classes, functions, etc., make sure to use meaningful names. Moreover, use the correct conventions when naming variables, classes, functions, etc.
* Use uppercase (e.g., `UPPERCASE`) for global constants.
* Use pascal case (e.g., `PascalCase`) for class names.
* Use camel case (e.g., `camelCase`) for variable and function names.

For CPP only, make sure to instantiate them in the same line when declaring variables.
* Use the curly brackets to instantiate the variables with initial values.
* As an example, follow the following structure: `int variableName {1}`.
* For classes, functions etc. that involve curly brackets (i.e., '{' and '}'), make sure to include the opening curly bracket on the same line of the statement, and the closing curly bracket on a new line directly after the last line inside the statement/class/function. Note that this applies to conditional and looping statements in CPP as well.

## Classes and Functions

Functions and classes should be developed with the intention of adhering to various software design principles, such as SRP (Single Responsibility Principle), DRY (Don't Repeat Yourself), and encapsulation. More information about these design principles can be accessed via [bit.ly/COMP1531](bit.ly/COMP1531).

To summarise the design principles for classes and functions:
* Each function should only achieve one objective.
* Prefer many short functions over one long function.
* Consider using classes if the object has separate logic.
* The main function should be very short, and all logic should be contained in previously defined functions or classes. This lets you run it as both a script and used as a custom package.
* For classes, ensure that its internal representation is hidden from the users. 
* In each class method and function, make sure that it contains only one return statement.

## Conditional and Looping Statements

The following conventions for conditional and looping statements (e.g., `if`, `for`, and `while`) should be adhered to.

| Python | CPP |
| --- | --- |
| <ul> Assignments in conditional statements must be avoided. Assignments in looping statements, specifically in the `for` loop, is acceptable. <li> Do not use `break` statements in loops. </li></ul> | <ul> Assignments in conditional statements must be avoided. Assignments in looping statements, specifically in the `for` loop, is acceptable. <li> Do not use `break` statements in loops. </li><li> When writing conditional and looping statements, do not include a space between the statement and the opening parenthesis of the condition. As an example, follow the following structure: `if(condition) { ...`. </li><li> Curly brackets are optional for single line conditional and looping statements. </li></ul> |

## Documentation and Comments

Documentation and comments are immensely useful for helping others to understand the code that you have written, and possibly continute your work. Try to comment with an appropriate frequency and at the right locations.

For file header comments, make sure to include the following information.
* File description and any notes.
* Author(s) names.
* File modification date (date that the last modification occured).
* File's relationship to package if relevant.
* Any references or OS platform if relevant.

Above every function/class method, write a docstring outlining the following information.
* Purpose of function
* Input parameters: expected type and explanation
* Return value: expected type and explanation
* Other preconditions/postconditions if necessary

Write short comments where necessary to summarise/explain complex logic.
* Make sure large blocks of code have comments above them summarising their purpose. 
* For smaller comments (e.g., notes), you can make a comment at the end of the line of code.
* Don't write comments for trivial operations. Assume the reader of the code knows how to program.
* Make sure to add a space after the comment symbol, and begin with an uppercase letter.

The following table contains which characters to use in Python and CPP, to write different types of comments.

| Documentation Type | Python | CPP |
| --- | --- | --- |
| File Header | `# ...` | `// ...` |
| Docstring | `""" ... """` | `/* ... */` |
| Short Comments | `# ...` | `// ...` |

## Miscellaneous

The following are some miscellaneous points.

* Use a proper and consistent indentation. For nested blocks of code, use an indentation of 4 spaces.
* Use spaces between operators (i.e., do `sum = x + y`, not `sum=x+y`).

## Example Code for Python

The following contains example code for Python that uses the concepts discussed in the sections above.

```python
# File:            exampleCode.py
# Description:     To provide an example
# Authors:         Mickey Mouse
# Platform:        Windows 10 (optional)
# Last Modified:   04/05/2021
# Notes:           The code is just an example.

import os
import numpy as np

CONST_NAME = "Social AI"

'''
 Purpose: Print given phrase, num times. 
 Args:
 - phrase (string). Phrase to be printed
 - num (int): number of times to print
 Return: null
'''
def printPhrase(phrase, num):
    for i in range(num):
        print(f"{i + 1}: {phrase}")

'''
 Main Function
'''
if __name__ == "__main__":
    num = int(input("Enter a positive integer: "))
    printPhrase(CONST_NAME, num)
```

## Example Code for CPP

The following contains example code for CPP that uses the concepts discussed in the sections above.

```cpp
// File:            exampleCode.cpp
// Description:     To provide an example
// Authors:         Mickey Mouse
// Platform:        Windows 10 (optional)
// Last Modified:   04/05/2021
// Notes:           The code is just an example.

#include <iostream>
#include <string>

const std::string CONST_NAME {"Social AI"};

/*
 * Purpose: Print given phrase, num times
 * Args:
 * - Phrase (std::string) - Phrase to be printed
 * - num (int) - Number of times to print
 * Return: void
 */
void printPhrase(std::string phrase, int num) {
    for(int i = 0; i < num; i++)
        std::cout << "(" << i+1 << "): " << phrase << std::endl;
    return;
}

/*
 * Main function
 */
int main(int argc, char** argv) {
    int num = 0;
    std::cout << "Enter a positive integer: ";
    std::cin >> num;
    printPhrase(CONST_NAME, num);
}

```
