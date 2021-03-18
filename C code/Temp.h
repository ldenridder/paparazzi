#ifndef TEMP_H
#define TEMP_H

//#include ... //Very rarely needed in header, mostly in source

class TEMP
{
private:
    //Make the input and output variables of functions private
    // so that you do not need to return them, but can change them within a function
    int var1;
    int var2;
 
public:
    TEMP(int img, int mtx, int dist); //create contructor

    void function1();    //Declare functions, voids return nothing
    void function2(int x); //Function that needs input

    int getVar1() { return var1;}   //Int function returns an int, can use to extract variables from other classes
    //these functions do not need to be created within the source
};
 
#endif