#include "TEMP.h"

//#include all needed modules/packages

TEMP::TEMP()
{
    //list all functions
    function1();
    function2();
}

void TEMP::function1()
{
    //change value of private variable, no return needed
    var1 += 1;
}

int TEMP::function2(int x)
{
    //change value of var2 depending on input
    var2 = var1 * x * x
}
