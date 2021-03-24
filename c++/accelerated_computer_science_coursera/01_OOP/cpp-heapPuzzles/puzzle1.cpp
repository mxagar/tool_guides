#include <iostream>

using std::cout;
using std::endl;

int main() {

  // Since we have no new keyword, everything is in the stack

  int  i =  2,  j =  4,  k =  8;
  int *p = &i, *q = &j, *r = &k;

  k = i; // k = i = 2
  cout << i << j << k << *p << *q << *r << endl; // 2 4 2, 2 4 2

  p = q; // the adress in p is the one in q -> j = 4
  cout << i << j << k << *p << *q << *r << endl; // 2 4 2, 4 4 2

  *q = *r; // the value pointed by q (j) is the one pointed by r (k) -> j = k = 2
  cout << i << j << k << *p << *q << *r << endl; // 2 2 2, 2 2 2

  return 0;
}
