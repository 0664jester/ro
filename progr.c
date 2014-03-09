#include <stdio.h>


int main() {

	int c;
	char array[16];
	int position = 0;



	while(c=getchar()) {

    if (c=='.') {
      return 1;       //ist ein exit damit ich das dosfenster nicht immer starten muss
    } 

		if (c != (0x09|0x20)) { 
			array[position] = c; 
			putchar(c); 
			position++; 
		}
		else {
			printf("Beer");
		}




	}

	return 0;
}