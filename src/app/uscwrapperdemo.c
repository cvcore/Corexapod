#include "uscengine.h"

int main() {
	initialize("params.in");
	walk(50, 0, 1, 1000);
	printf("--------------------\n");
	roll(PI / 15, 1, 800);
	return 0;
}
