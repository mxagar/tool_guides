/**
*\file factorial.cpp
*\author Mikel Sagardia
*\date 16.02.2010
*/

#include "factorial.h"

int factorial(int n){
    if(n!=1){
		return(n * factorial(n-1));
    }
    else return 1;
}
