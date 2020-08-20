#include <stdio.h>
#include <unistd.h>
#include <string.h>

int main()
{
	char jsonString[512];
	strcpy(jsonString, "['abc':'xx','bb':'yy']");
	write(STDOUT_FILENO, jsonString, strlen(jsonString)); 
	return 0;
}
