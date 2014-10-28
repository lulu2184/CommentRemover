#include <iostream>
#include <cstring>
using namespace std;

int main()
{
	char *p = "abc";
	if (strncmp("abc", p, 3)) printf("a");
	else printf("b");

	return 0;
}